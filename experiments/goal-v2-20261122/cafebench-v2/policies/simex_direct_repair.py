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

    def ready(scene):
        return scene.get("control_ready") is True

    def observation_failure():
        sdk.retract_safe_pose(command_id=command("observation_retract"))
        return {"status": "rejected_observation_not_ready"}

    def pause(reason, candidates):
        sdk.request_operator_confirmation(reason, candidates)
        return {"status": "paused_for_operator", "reason": reason}

    def valid_slot(slot):
        if not slot or "NOGO" in slot.upper() or "STEAM" in slot.upper():
            return False
        stem = "$SLOT_PICKUP_"
        if not slot.startswith(stem):
            return False
        suffix = slot[len(stem):]
        return (
            bool(suffix)
            and all(character in "0123456789" for character in suffix)
            and any(character != "0" for character in suffix)
        )

    def resolve_brew(receipt):
        for attempt in range(3):
            if receipt in ("running", "done", "completed", "rejected"):
                break
            receipt = sdk.query_command_status(command("brew")).get("receipt")
        return receipt

    order = sdk.get_order(order_id)
    if not order:
        return escalate("Requested order does not exist.")

    target = params.get("target_slot") or order.get("destination_slot")
    if not valid_slot(target):
        return escalate("Delivery destination is forbidden or is not a pickup slot.")
    if order.get("special_request"):
        return escalate("Order requires manual preparation.")
    if not order.get("drink"):
        return escalate("Order has no drink specification.")
    if order.get("status") in ("cancelled", "canceled", "rejected"):
        return escalate("Order is no longer eligible for fulfillment.")

    commit = sdk.query_command_status(command("complete"))
    if commit.get("receipt") in ("done", "completed"):
        return {"status": "completed"}
    if order.get("status") in ("completed", "delivered"):
        return escalate("Order was already fulfilled; delivery ownership is unclear.")

    scene = sdk.observe_scene()
    if not ready(scene):
        return {"status": "rejected_observation_not_ready"}

    machine = scene.get("coffee_machine", {})
    for attempt in range(5):
        if machine.get("status") not in ("busy", "cleaning"):
            break
        scene = sdk.observe_scene()
        if not ready(scene):
            return observation_failure()
        machine = scene.get("coffee_machine", {})

    if machine.get("status") == "fault":
        return escalate("Coffee machine reports a fault.")

    resume_brew = (
        machine.get("status") == "running"
        or (
            machine.get("status") in ("idle", "completed")
            and machine.get("cup_filled") is True
        )
    )

    previous_receipt = sdk.query_command_status(command("brew")).get("receipt")
    recovery_needs_wait = False
    if resume_brew:
        if (
            machine.get("status") == "running"
            and scene.get("l2_pouring_safety", {}).get("safe_to_pour") is not True
        ):
            return escalate("An existing brew has unverified pouring safety.")
        if previous_receipt == "rejected":
            return escalate("The prior brew command was rejected.")
        if previous_receipt not in ("running", "done", "completed", "unknown"):
            return escalate("Prior brew receipt remains unresolved.")
        # A restart can lose command history while retaining physical progress.
        # Recover through readiness acknowledgement; never issue another start.
        recovery_needs_wait = previous_receipt == "unknown"
    else:
        if previous_receipt == "rejected":
            return escalate("The prior brew command was rejected.")
        if previous_receipt != "unknown":
            return escalate("Prior brew receipt conflicts with the machine state.")
        if (
            machine.get("status") != "idle"
            or machine.get("cup_filled") is not False
        ):
            return escalate("Machine is not idle and empty for a new brew.")

    threshold = scene.get("confidence_threshold")
    if threshold is None:
        return observation_failure()

    candidates = [
        obj
        for obj in scene.get("objects", [])
        if obj.get("id")
        and obj.get("cup_type")
        and obj.get("cup_type") != "flexible_bag"
        and obj.get("confidence", -1) >= threshold
    ]

    confirmed_id = scene.get("operator_confirmed_cup_id")
    if confirmed_id:
        selected = [
            candidate for candidate in candidates
            if candidate["id"] == confirmed_id
        ]
        if len(selected) != 1:
            return pause(
                "Confirmed cup is not uniquely visible with sufficient confidence.",
                candidates,
            )
        cup = selected[0]
    else:
        if len(candidates) != 1:
            return pause("Select and confirm one rigid cup for this order.", candidates)
        cup = candidates[0]

    cup_id = cup["id"]

    if not resume_brew:
        placed = sdk.place_cup_under_machine(cup_id, command_id=command("place"))
        if placed.get("receipt") != "done":
            return escalate("Cup placement was not acknowledged.")

        scene = sdk.observe_scene()
        if not ready(scene):
            return observation_failure()

        machine = scene.get("coffee_machine", {})
        if machine.get("status") != "idle" or machine.get("cup_filled") is not False:
            return escalate("Machine state changed before brewing.")

        if scene.get("l2_pouring_safety", {}).get("safe_to_pour") is not True:
            recentered = sdk.recenter_cup_under_spout(
                cup_id, command_id=command("recenter")
            )
            if recentered.get("receipt") != "done":
                return escalate("Cup recentering was not acknowledged.")
            scene = sdk.observe_scene()
            if not ready(scene):
                return observation_failure()

        machine = scene.get("coffee_machine", {})
        if machine.get("status") != "idle" or machine.get("cup_filled") is not False:
            return escalate("Machine is not ready immediately before brewing.")
        if scene.get("l2_pouring_safety", {}).get("safe_to_pour") is not True:
            return escalate("Pouring safety remains unverified after positioning.")

        threshold = scene.get("confidence_threshold")
        if threshold is None:
            return observation_failure()

        visible_cup = [
            obj
            for obj in scene.get("objects", [])
            if obj.get("id") == cup_id
            and obj.get("cup_type")
            and obj.get("cup_type") != "flexible_bag"
            and obj.get("confidence", -1) >= threshold
        ]
        if len(visible_cup) != 1:
            sdk.retract_safe_pose(command_id=command("identity_retract"))
            return pause("Selected cup identity was lost before brewing.", visible_cup)

        brew = sdk.start_coffee_machine(command_id=command("brew"))
        receipt = resolve_brew(brew.get("receipt"))
        if receipt not in ("running", "done", "completed"):
            return escalate("Brew command was rejected or its receipt remains unresolved.")

    if recovery_needs_wait or not (
        resume_brew
        and machine.get("status") in ("idle", "completed")
        and machine.get("cup_filled") is True
    ):
        finished = sdk.wait_for_coffee_ready(command_id=command("wait"))
        if finished.get("receipt") != "done":
            return escalate("Coffee readiness was not acknowledged.")

    scene = sdk.observe_scene()
    if not ready(scene):
        return observation_failure()

    machine = scene.get("coffee_machine", {})
    if (
        machine.get("status") not in ("idle", "completed")
        or machine.get("cup_filled") is not True
    ):
        return escalate("Machine did not confirm a finished, filled cup.")

    delivered = sdk.retrieve_and_deliver_cup(
        cup_id, target, command_id=command("deliver")
    )
    if delivered.get("receipt") != "done":
        return escalate("Physical delivery was not acknowledged.")

    completed = sdk.complete_order_delivery(
        order_id, cup_id, target, command_id=command("complete")
    )
    if completed.get("receipt") != "done":
        return escalate("Delivery occurred, but order completion was not acknowledged.")

    return {"status": "completed"}
