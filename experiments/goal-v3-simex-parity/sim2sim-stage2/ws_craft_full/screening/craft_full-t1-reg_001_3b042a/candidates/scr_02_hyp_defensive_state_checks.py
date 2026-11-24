def run_policy(sdk, params):
    order_id = params.get("order_id")
    prefix = params.get("command_prefix") or str(order_id)

    def escalate(reason):
        retraction = sdk.retract_safe_pose(command_id=prefix + ":retract")
        escalation = sdk.escalate_to_human_operator(
            order_id, reason, command_id=prefix + ":escalate"
        )
        if not escalation or escalation.get("receipt") not in (
            "done", "escalated_to_human"
        ):
            return {"status": "escalation_unconfirmed", "reason": reason}
        if not retraction or retraction.get("receipt") != "done":
            return {
                "status": "escalated_to_human",
                "reason": reason,
                "retraction_confirmed": False,
            }
        return {"status": "escalated_to_human", "reason": reason}

    def valid_cups(scene):
        return [
            obj for obj in scene.get("objects", [])
            if obj.get("cup_type")
            and obj.get("cup_type") != "flexible_bag"
            and obj.get("confidence", 0) >= scene.get("confidence_threshold", 1)
        ]

    def cup_confirmed(scene, cup_id):
        candidates = valid_cups(scene)
        matches = [obj for obj in candidates if obj.get("id") == cup_id]
        confirmed_id = scene.get("operator_confirmed_cup_id")
        if len(matches) != 1:
            return False
        if confirmed_id:
            return confirmed_id == cup_id
        return len(candidates) == 1

    def machine_idle(scene):
        machine = scene.get("coffee_machine") or {}
        return (
            machine.get("status") == "idle"
            and machine.get("cup_filled") is False
        )

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
    if not scene or not scene.get("control_ready"):
        return escalate("Observation is not ready")
    if not machine_idle(scene):
        return escalate("Machine is not ready for a new single-cup brew")

    candidates = valid_cups(scene)
    confirmed_id = scene.get("operator_confirmed_cup_id")
    if confirmed_id:
        selected = [obj for obj in candidates if obj.get("id") == confirmed_id]
    else:
        selected = candidates
    if len(selected) != 1 or not selected[0].get("id"):
        return escalate("One confidently observed rigid cup must be confirmed")
    cup_id = selected[0]["id"]

    placement = sdk.place_cup_under_machine(
        cup_id, command_id=prefix + ":place"
    )
    if not placement or placement.get("receipt") != "done":
        return escalate("Cup placement was not confirmed")

    scene = sdk.observe_scene()
    if not scene or not scene.get("control_ready"):
        return escalate("Observation after placement is not ready")
    if not cup_confirmed(scene, cup_id):
        return escalate("Cup identity after placement is ambiguous")
    if not machine_idle(scene):
        return escalate("Machine state changed before brewing")

    safety = scene.get("l2_pouring_safety") or {}
    if safety.get("safe_to_pour") is not True:
        recenter = sdk.recenter_cup_under_spout(
            cup_id, command_id=prefix + ":recenter"
        )
        if not recenter or recenter.get("receipt") != "done":
            return escalate("Cup recentering was not confirmed")
        scene = sdk.observe_scene()
        if not scene or not scene.get("control_ready"):
            return escalate("Observation after recentering is not ready")
        if not cup_confirmed(scene, cup_id):
            return escalate("Cup identity after recentering is ambiguous")
        if not machine_idle(scene):
            return escalate("Machine state changed before brewing")
        safety = scene.get("l2_pouring_safety") or {}
        if safety.get("safe_to_pour") is not True:
            return escalate("Pouring safety check failed after recentering")

    brew_id = prefix + ":brew"
    brew = sdk.start_coffee_machine(command_id=brew_id)
    receipt = (brew or {}).get("receipt")
    if receipt not in ("running", "done", "completed", "rejected"):
        result = sdk.query_command_status(brew_id)
        receipt = (result or {}).get("receipt")
    if receipt not in ("running", "done", "completed"):
        return escalate("Brew receipt was rejected or remains unresolved")

    ready = sdk.wait_for_coffee_ready(command_id=prefix + ":wait")
    if not ready or ready.get("receipt") != "done":
        return escalate("Coffee readiness was not confirmed")

    scene = sdk.observe_scene()
    if not scene or not scene.get("control_ready"):
        return escalate("Observation before delivery is not ready")
    if not cup_confirmed(scene, cup_id):
        return escalate("Cup identity before delivery is ambiguous")
    machine = scene.get("coffee_machine") or {}
    if (
        machine.get("status") not in ("idle", "completed")
        or machine.get("cup_filled") is not True
    ):
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
