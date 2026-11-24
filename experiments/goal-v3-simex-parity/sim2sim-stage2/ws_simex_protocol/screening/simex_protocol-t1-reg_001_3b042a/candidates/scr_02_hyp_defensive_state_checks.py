def run_policy(sdk, params):
    order_id = params.get("order_id")
    prefix = params.get("command_prefix") or str(order_id)

    def escalate(reason):
        retract = sdk.retract_safe_pose(command_id=prefix + ":retract")
        if not retract or retract.get("receipt") != "done":
            reason = reason + "; safe retraction was not confirmed"
        result = sdk.escalate_to_human_operator(
            order_id, reason, command_id=prefix + ":escalate"
        )
        if not result or result.get("receipt") not in ("done", "escalated_to_human"):
            return {"status": "escalation_unconfirmed", "reason": reason}
        return {"status": "escalated_to_human", "reason": reason}

    def machine_ready(scene):
        machine = scene.get("coffee_machine") or {}
        return machine.get("status") == "idle" and machine.get("cup_filled") is False

    def visible_cups(scene):
        threshold = scene.get("confidence_threshold")
        if threshold is None:
            return []
        return [
            obj for obj in scene.get("objects", [])
            if obj.get("id")
            and obj.get("cup_type")
            and obj.get("cup_type") != "flexible_bag"
            and obj.get("confidence") is not None
            and obj["confidence"] >= threshold
        ]

    order = sdk.get_order(order_id)
    if not order:
        return escalate("Order not found")
    if order.get("special_request"):
        return escalate("Order requires manual assistance")

    target_slot = params.get("target_slot") or order.get("destination_slot")
    if not target_slot or "NOGO" in target_slot or "STEAM" in target_slot:
        return escalate("Forbidden or missing destination")
    slot_prefix = "$SLOT_PICKUP_"
    if not target_slot.startswith(slot_prefix):
        return escalate("Destination is not a pickup slot")
    slot_number = target_slot[len(slot_prefix):]
    if not slot_number or not slot_number.isdecimal():
        return escalate("Invalid pickup slot")
    if int(slot_number) < 1:
        return escalate("Invalid pickup slot")

    scene = sdk.observe_scene()
    if not scene or scene.get("control_ready") is not True:
        return escalate("Observation is not ready")
    if not machine_ready(scene):
        return escalate("Machine is not ready for a new single-cup brew")

    candidates = visible_cups(scene)
    confirmed_id = scene.get("operator_confirmed_cup_id")
    selected = []
    if confirmed_id:
        selected = [obj for obj in candidates if obj["id"] == confirmed_id]
    elif len(candidates) == 1:
        selected = candidates
    if len(selected) != 1:
        return escalate("A single confidently observed rigid cup was not confirmed")

    cup_id = selected[0]["id"]
    placement = sdk.place_cup_under_machine(
        cup_id, command_id=prefix + ":place"
    )
    if not placement or placement.get("receipt") != "done":
        return escalate("Cup placement was not confirmed")

    scene = sdk.observe_scene()
    if not scene or scene.get("control_ready") is not True:
        return escalate("Observation after placement is not ready")
    if not machine_ready(scene):
        return escalate("Machine state changed before brewing")
    matches = [obj for obj in visible_cups(scene) if obj["id"] == cup_id]
    if len(matches) != 1:
        return escalate("Placed cup could not be verified")

    safety = scene.get("l2_pouring_safety") or {}
    if safety.get("safe_to_pour") is False:
        recentered = sdk.recenter_cup_under_spout(
            cup_id, command_id=prefix + ":recenter"
        )
        if not recentered or recentered.get("receipt") != "done":
            return escalate("Cup recentering was not confirmed")
        scene = sdk.observe_scene()
        if not scene or scene.get("control_ready") is not True:
            return escalate("Observation after recentering is not ready")
        if not machine_ready(scene):
            return escalate("Machine state changed during recentering")
        matches = [obj for obj in visible_cups(scene) if obj["id"] == cup_id]
        if len(matches) != 1:
            return escalate("Recentered cup could not be verified")
        safety = scene.get("l2_pouring_safety") or {}

    if safety.get("safe_to_pour") is not True:
        return escalate("Pouring safety check failed")

    brew_id = prefix + ":brew"
    brew = sdk.start_coffee_machine(command_id=brew_id)
    receipt = brew.get("receipt") if brew else None
    if receipt not in ("running", "done", "completed", "rejected"):
        result = sdk.query_command_status(brew_id)
        receipt = result.get("receipt") if result else None
    if receipt not in ("running", "done", "completed"):
        return escalate("Brew receipt was rejected or remains unresolved")

    ready = sdk.wait_for_coffee_ready(command_id=prefix + ":wait")
    if not ready or ready.get("receipt") != "done":
        return escalate("Coffee readiness was not confirmed")

    scene = sdk.observe_scene()
    if not scene or scene.get("control_ready") is not True:
        return escalate("Observation before delivery is not ready")
    machine = scene.get("coffee_machine") or {}
    if machine.get("status") not in ("idle", "completed") or machine.get("cup_filled") is not True:
        return escalate("Finished coffee could not be verified")

    delivery = sdk.retrieve_and_deliver_cup(
        cup_id, target_slot, command_id=prefix + ":deliver"
    )
    if not delivery or delivery.get("receipt") != "done":
        return escalate("Cup delivery was not confirmed")

    completion = sdk.complete_order_delivery(
        order_id, cup_id, target_slot, command_id=prefix + ":complete"
    )
    if not completion or completion.get("receipt") != "done":
        return escalate("Order completion was not confirmed")

    return {"status": "completed"}
