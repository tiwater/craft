def run_policy(sdk, params):
    order_id = params.get("order_id")
    prefix = params.get("command_prefix")

    if not order_id or not prefix:
        return {"status": "rejected_invalid_parameters"}

    def command(step):
        return str(prefix) + ":" + step

    def escalate(reason):
        sdk.retract_safe_pose(command_id=command("retract"))
        sdk.escalate_to_human_operator(
            order_id, reason, command_id=command("escalate")
        )
        return {"status": "escalated_to_human", "reason": reason}

    def observation_ready(scene):
        if not scene.get("control_ready", False):
            return False
        return all(
            value is not False
            for value in scene.get("pipeline_health", {}).values()
        )

    def valid_cups(scene):
        threshold = scene.get("confidence_threshold")
        if threshold is None:
            return []
        return [
            obj
            for obj in scene.get("objects", [])
            if obj.get("id")
            and obj.get("cup_type")
            and obj.get("cup_type") != "flexible_bag"
            and obj.get("confidence", 0) >= threshold
        ]

    def selected_cup_visible(scene, cup_id):
        return any(obj["id"] == cup_id for obj in valid_cups(scene))

    def pause(reason, candidates):
        sdk.request_operator_confirmation(reason, candidates)
        return {"status": "paused_for_operator", "reason": reason}

    order = sdk.get_order(order_id)
    if not order:
        return escalate("Order was not found.")

    target = params.get("target_slot") or order.get("destination_slot")
    if not target:
        return escalate("A pickup destination is required.")

    target_text = str(target)
    if "NOGO" in target_text.upper() or "STEAM" in target_text.upper():
        return escalate("The requested destination is a forbidden zone.")

    slot_prefix = "$SLOT_PICKUP_"
    suffix = target_text[len(slot_prefix):]
    if (
        not target_text.startswith(slot_prefix)
        or not suffix
        or not all(character in "0123456789" for character in suffix)
        or not any(character != "0" for character in suffix)
    ):
        return escalate("The requested destination is not a valid pickup slot.")

    if order.get("special_request"):
        return escalate("The order requires unsupported manual operations.")

    if not order.get("drink"):
        return escalate("The order does not specify an automated drink.")

    scene = sdk.observe_scene()
    if not observation_ready(scene):
        return {"status": "rejected_observation_not_ready"}

    machine = scene.get("coffee_machine", {})
    if machine.get("status") == "fault":
        return escalate("The coffee machine reports a fault.")

    candidates = valid_cups(scene)
    confirmed_id = scene.get("operator_confirmed_cup_id")

    if confirmed_id:
        matching = [
            candidate for candidate in candidates
            if candidate["id"] == confirmed_id
        ]
        if len(matching) != 1:
            return pause(
                "The confirmed cup cannot be identified reliably.", candidates
            )
        cup_id = matching[0]["id"]
    elif len(candidates) == 1:
        cup_id = candidates[0]["id"]
    else:
        return pause(
            "Cup identity is missing or ambiguous.", candidates
        )

    if machine.get("status") != "idle" or machine.get("cup_filled", False):
        return escalate("The machine is not available for a fresh brew.")

    # Keep a journal in the supplied parameter object so reuse of that object
    # cannot resubmit a brew whose outcome was lost.
    journal = params.setdefault("coffee_policy_brew_journal", {})
    brew_id = command("brew")
    if brew_id in journal:
        return escalate(
            "This brew command was already attempted; recovery requires "
            "operator reconciliation."
        )

    placement = sdk.place_cup_under_machine(
        cup_id, command_id=command("place")
    )
    if placement.get("receipt") != "done":
        return escalate("Cup placement was not confirmed.")

    scene = sdk.observe_scene()
    if not observation_ready(scene):
        sdk.retract_safe_pose(command_id=command("retract"))
        return {"status": "rejected_observation_not_ready"}

    if not selected_cup_visible(scene, cup_id):
        sdk.retract_safe_pose(command_id=command("retract"))
        return pause(
            "The selected cup cannot be verified after placement.",
            valid_cups(scene),
        )

    machine = scene.get("coffee_machine", {})
    if machine.get("status") != "idle" or machine.get("cup_filled", False):
        return escalate("The machine is not safe to start after cup placement.")

    if not scene.get("l2_pouring_safety", {}).get("safe_to_pour", False):
        recenter = sdk.recenter_cup_under_spout(
            cup_id, command_id=command("recenter")
        )
        if recenter.get("receipt") != "done":
            return escalate("The cup could not be centered safely.")

        scene = sdk.observe_scene()
        if not observation_ready(scene):
            sdk.retract_safe_pose(command_id=command("retract"))
            return {"status": "rejected_observation_not_ready"}

        if not selected_cup_visible(scene, cup_id):
            sdk.retract_safe_pose(command_id=command("retract"))
            return pause(
                "The selected cup cannot be verified after recentering.",
                valid_cups(scene),
            )

    machine = scene.get("coffee_machine", {})
    if machine.get("status") != "idle" or machine.get("cup_filled", False):
        return escalate("The machine is not available for brewing.")

    if not scene.get("l2_pouring_safety", {}).get("safe_to_pour", False):
        return escalate("Pouring safety is not satisfied.")

    # There is exactly one start submission. Unknown receipts are queried,
    # never resolved by resubmitting this non-idempotent command.
    journal[brew_id] = "attempted"
    receipt = sdk.start_coffee_machine(command_id=brew_id).get("receipt")
    journal[brew_id] = receipt

    for attempt in range(3):
        if receipt != "unknown" and receipt is not None:
            break
        receipt = sdk.query_command_status(brew_id).get("receipt")
        journal[brew_id] = receipt

    if receipt not in ("running", "done", "completed"):
        return escalate("The brew command was rejected or remains unresolved.")

    ready = sdk.wait_for_coffee_ready(command_id=command("wait_ready"))
    if ready.get("receipt") != "done":
        return escalate("Coffee readiness was not confirmed.")

    scene = sdk.observe_scene()
    if not observation_ready(scene):
        sdk.retract_safe_pose(command_id=command("retract"))
        return {"status": "rejected_observation_not_ready"}

    machine = scene.get("coffee_machine", {})
    if machine.get("status") not in ("idle", "completed"):
        return escalate("The machine did not reach a safe finished state.")

    if not machine.get("cup_filled", False):
        return escalate("The machine did not confirm a filled cup.")

    if not selected_cup_visible(scene, cup_id):
        sdk.retract_safe_pose(command_id=command("retract"))
        return pause(
            "The filled cup cannot be identified reliably.", valid_cups(scene)
        )

    delivered = sdk.retrieve_and_deliver_cup(
        cup_id, target, command_id=command("deliver")
    )
    if delivered.get("receipt") != "done":
        return escalate("Cup delivery was not confirmed.")

    completed = sdk.complete_order_delivery(
        order_id, cup_id, target, command_id=command("complete")
    )
    if completed.get("receipt") != "done":
        return escalate(
            "The cup was delivered, but order completion was not confirmed."
        )

    journal[brew_id] = "delivered"
    return {
        "status": "completed",
        "order_id": order_id,
        "cup_id": cup_id,
        "destination_slot": target,
    }
