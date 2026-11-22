def run_policy(sdk, params):
    order_id = params.get("order_id", "A042")
    target_slot = params.get("target_slot", "$SLOT_PICKUP_1")
    command_prefix = str(params.get("command_prefix", "cap"))
    command_base = command_prefix + ":" + str(order_id)

    def escalate(reason):
        retract = sdk.retract_safe_pose(
            command_id=command_base + ":retract"
        )
        escalation = sdk.escalate_to_human_operator(
            order_id, reason, command_id=command_base + ":escalate"
        )
        return {
            "status": "escalated_to_human",
            "reason": reason,
            "retract_receipt": retract.get("receipt"),
            "escalation_receipt": escalation.get("receipt"),
        }

    order = sdk.get_order(order_id)
    if order is None:
        return escalate("Order not found")

    slot = str(target_slot)
    slot_prefix = "$SLOT_PICKUP_"
    slot_number = slot[len(slot_prefix):]
    valid_slot = (
        slot.startswith(slot_prefix)
        and len(slot_number) > 0
        and all(character in "0123456789" for character in slot_number)
        and any(character != "0" for character in slot_number)
    )
    if "NOGO" in slot.upper() or "STEAM" in slot.upper() or not valid_slot:
        return escalate("Invalid or forbidden delivery target")

    automated_drinks = (
        "hot_americano",
        "takeout_americano",
        "iced_americano",
        "espresso",
        "hot_espresso",
        "takeout_espresso",
    )
    if order.get("special_request") or order.get("drink") not in automated_drinks:
        return escalate("Order requires unsupported or manual preparation")

    scene = sdk.observe_scene()
    if scene.get("control_ready") != True:
        return {"status": "rejected_observation_not_ready"}

    threshold = scene["confidence_threshold"]
    candidates = [
        obj
        for obj in scene["objects"]
        if obj["cup_type"] != "flexible_bag"
        and obj["confidence"] >= threshold
    ]
    confirmed_id = scene.get("operator_confirmed_cup_id")
    if confirmed_id is not None:
        selected = [
            obj for obj in candidates if obj["id"] == confirmed_id
        ]
    else:
        selected = candidates

    if len(selected) != 1:
        confirmation = sdk.request_operator_confirmation(
            "A single eligible rigid cup must be uniquely identified",
            candidates,
        )
        return {
            "status": "paused_for_operator",
            "confirmation": confirmation,
        }

    cup_id = selected[0]["id"]
    placement = sdk.place_cup_under_machine(
        cup_id, command_id=command_base + ":place"
    )
    if placement.get("receipt") != "done":
        return {"status": "rejected_placement"}

    scene = sdk.observe_scene()
    if scene.get("control_ready") != True:
        return {"status": "rejected_observation_not_ready"}

    if scene.get("l2_pouring_safety", {}).get("safe_to_pour") != True:
        recenter = sdk.recenter_cup_under_spout(
            cup_id, command_id=command_base + ":recenter"
        )
        if recenter.get("receipt") != "done":
            return {"status": "rejected_recentering"}

        scene = sdk.observe_scene()
        if scene.get("control_ready") != True:
            return {"status": "rejected_observation_not_ready"}
        if scene.get("l2_pouring_safety", {}).get("safe_to_pour") != True:
            return {"status": "rejected_pouring_safety"}

    brew_command = command_base + ":brew"
    brew = sdk.start_coffee_machine(command_id=brew_command)
    receipt = brew.get("receipt")
    if receipt == "unknown":
        brew = sdk.query_command_status(brew_command)
        receipt = brew.get("receipt")

    if receipt not in ("running", "done", "completed"):
        return {
            "status": (
                "rejected_brew" if receipt == "rejected"
                else "brew_status_unresolved"
            ),
            "command_id": brew_command,
        }

    ready = sdk.wait_for_coffee_ready(
        command_id=command_base + ":wait"
    )
    if ready.get("receipt") != "done":
        return {"status": "rejected_brew_completion"}

    delivery = sdk.retrieve_and_deliver_cup(
        cup_id, target_slot, command_id=command_base + ":deliver"
    )
    if delivery.get("receipt") != "done":
        return {"status": "rejected_delivery"}

    completion = sdk.complete_order_delivery(
        order_id,
        cup_id,
        target_slot,
        command_id=command_base + ":complete",
    )
    if completion.get("receipt") != "done":
        return {
            "status": "delivered_finalization_pending",
            "order_id": order_id,
            "cup_id": cup_id,
            "target_slot": target_slot,
        }

    return {
        "status": "completed",
        "order_id": order_id,
        "cup_id": cup_id,
        "target_slot": target_slot,
    }
