def run_policy(sdk, params):
    order_id = params.get("order_id")
    prefix = params.get("command_prefix")

    def escalate(reason):
        command_root = prefix or "coffee-policy-invalid-request"
        retract = sdk.retract_safe_pose(
            command_id=command_root + ":retract"
        )
        escalation = sdk.escalate_to_human_operator(
            order_id,
            reason,
            command_id=command_root + ":escalate"
        )
        return {
            "status": "escalated_to_human",
            "reason": reason,
            "retract_result": retract,
            "escalation_result": escalation
        }

    def observation_ready(scene):
        return bool(scene) and scene.get("control_ready") is True

    def reject_observation():
        return {"status": "rejected_observation_not_ready"}

    def pause(reason, candidates):
        sdk.request_operator_confirmation(reason, candidates)
        return {"status": "paused_for_operator", "reason": reason}

    def valid_cups(scene):
        threshold = scene.get("confidence_threshold")
        if threshold is None:
            return []
        return [
            obj for obj in scene.get("objects", [])
            if obj.get("id")
            and obj.get("cup_type")
            and obj.get("cup_type") != "flexible_bag"
            and obj.get("confidence", -1.0) >= threshold
        ]

    def cup_still_identified(scene, cup_id):
        matches = [
            obj for obj in valid_cups(scene)
            if obj["id"] == cup_id
        ]
        confirmed = scene.get("operator_confirmed_cup_id")
        return (
            len(matches) == 1
            and (not confirmed or confirmed == cup_id)
        )

    if not order_id or not prefix:
        return escalate("Missing order_id or command_prefix.")

    order = sdk.get_order(order_id)
    if not order or order.get("order_id") != order_id:
        return escalate("Order is missing or its identity does not match.")

    target_slot = params.get("target_slot") or order.get("destination_slot")
    if not target_slot:
        return escalate("No pickup destination was supplied.")

    normalized_slot = target_slot.upper()
    if "NOGO" in normalized_slot or "STEAM" in normalized_slot:
        return escalate("The requested destination is a forbidden zone.")

    slot_prefix = "$SLOT_PICKUP_"
    if not target_slot.startswith(slot_prefix):
        return escalate("The destination is not a pickup slot.")

    slot_number = target_slot[len(slot_prefix):]
    if (
        not slot_number
        or any(character not in "0123456789" for character in slot_number)
        or not any(character != "0" for character in slot_number)
    ):
        return escalate("The pickup slot identifier is invalid.")

    if order.get("special_request"):
        return escalate("The order requires unsupported manual operations.")

    if not order.get("drink"):
        return escalate("The order has no drink specification.")

    if order.get("status") in (
        "completed", "delivered", "cancelled", "canceled", "rejected"
    ):
        return escalate("The order is already terminal; do not brew again.")

    brew_command_id = prefix + ":brew"
    scene = sdk.observe_scene()
    if not observation_ready(scene):
        return reject_observation()

    machine = scene.get("coffee_machine", {})

    for attempt in range(5):
        if machine.get("status") not in ("busy", "cleaning"):
            break
        scene = sdk.observe_scene()
        if not observation_ready(scene):
            return reject_observation()
        machine = scene.get("coffee_machine", {})

    if machine.get("status") == "fault":
        return escalate("The coffee machine reports a fault.")

    resume_brew = machine.get("status") in ("running", "completed")
    if resume_brew:
        brew = sdk.query_command_status(brew_command_id)
        if brew.get("receipt") not in ("running", "done", "completed"):
            return escalate(
                "The existing brew cannot be reconciled with this command."
            )
    elif machine.get("status") != "idle" or machine.get("cup_filled"):
        return escalate("The machine is not empty and idle.")

    candidates = valid_cups(scene)
    confirmed_id = scene.get("operator_confirmed_cup_id")

    if confirmed_id:
        selected = [
            obj for obj in candidates if obj["id"] == confirmed_id
        ]
        if len(selected) != 1:
            return pause(
                "The confirmed cup is not uniquely visible at sufficient confidence.",
                candidates
            )
        cup_id = selected[0]["id"]
    elif len(candidates) == 1:
        cup_id = candidates[0]["id"]
    else:
        return pause("Cup identity is missing or ambiguous.", candidates)

    if not resume_brew:
        placement = sdk.place_cup_under_machine(
            cup_id, command_id=prefix + ":place"
        )
        if placement.get("receipt") != "done":
            return escalate("Cup placement was not confirmed.")

        scene = sdk.observe_scene()
        if not observation_ready(scene):
            sdk.retract_safe_pose(command_id=prefix + ":retract")
            return reject_observation()

        if not cup_still_identified(scene, cup_id):
            sdk.retract_safe_pose(command_id=prefix + ":retract")
            return pause(
                "The selected cup cannot be confidently identified after placement.",
                valid_cups(scene)
            )

        machine = scene.get("coffee_machine", {})
        if machine.get("status") != "idle" or machine.get("cup_filled"):
            return escalate("The machine changed state before brewing.")

        if scene.get("l2_pouring_safety", {}).get("safe_to_pour") is not True:
            recenter = sdk.recenter_cup_under_spout(
                cup_id, command_id=prefix + ":recenter"
            )
            if recenter.get("receipt") != "done":
                return escalate("Unsafe pouring alignment could not be corrected.")

            scene = sdk.observe_scene()
            if not observation_ready(scene):
                sdk.retract_safe_pose(command_id=prefix + ":retract")
                return reject_observation()

            if not cup_still_identified(scene, cup_id):
                sdk.retract_safe_pose(command_id=prefix + ":retract")
                return pause(
                    "Cup identity is uncertain after recentering.",
                    valid_cups(scene)
                )

        machine = scene.get("coffee_machine", {})
        if machine.get("status") != "idle" or machine.get("cup_filled"):
            return escalate("The machine is not ready for a new brew.")

        if scene.get("l2_pouring_safety", {}).get("safe_to_pour") is not True:
            return escalate("Pouring safety is not affirmatively established.")

        brew = sdk.start_coffee_machine(command_id=brew_command_id)
        receipt = brew.get("receipt")
        if receipt not in ("running", "done", "completed", "rejected"):
            brew = sdk.query_command_status(brew_command_id)
            receipt = brew.get("receipt")

        if receipt not in ("running", "done", "completed"):
            return escalate("The brew command was rejected or remains unresolved.")

    ready = sdk.wait_for_coffee_ready(
        command_id=prefix + ":wait-ready"
    )
    if ready.get("receipt") != "done":
        return escalate("Coffee readiness was not confirmed.")

    scene = sdk.observe_scene()
    if not observation_ready(scene):
        return escalate("Perception became unavailable after brewing.")

    machine = scene.get("coffee_machine", {})
    if (
        machine.get("status") not in ("idle", "completed")
        or machine.get("cup_filled") is not True
    ):
        return escalate("The machine did not confirm a safely finished brew.")

    if not cup_still_identified(scene, cup_id):
        sdk.retract_safe_pose(command_id=prefix + ":retract")
        return pause(
            "The filled cup cannot be confidently identified for delivery.",
            valid_cups(scene)
        )

    delivery = sdk.retrieve_and_deliver_cup(
        cup_id,
        target_slot,
        command_id=prefix + ":deliver"
    )
    if delivery.get("receipt") != "done":
        return escalate("Physical delivery was not confirmed.")

    completion = sdk.complete_order_delivery(
        order_id,
        cup_id,
        target_slot,
        command_id=prefix + ":complete"
    )
    if completion.get("receipt") != "done":
        return escalate(
            "The cup was delivered, but order completion was not confirmed. "
            "Do not brew a replacement."
        )

    return {
        "status": "completed",
        "order_id": order_id,
        "cup_id": cup_id,
        "destination_slot": target_slot
    }
