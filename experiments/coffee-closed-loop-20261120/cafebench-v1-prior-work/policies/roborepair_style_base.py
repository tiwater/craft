def run_policy(sdk, params):
    order_id = params.get("order_id", "A042")
    target_slot = params.get("target_slot", "$SLOT_PICKUP_1")
    command_prefix = str(params.get("command_prefix", "cap"))
    command_base = command_prefix + ":" + str(order_id)

    def escalate(reason):
        sdk.retract_safe_pose(command_id=command_base + ":retract")
        sdk.escalate_to_human_operator(
            order_id, reason, command_id=command_base + ":escalate"
        )
        return {
            "status": "escalated_to_human",
            "order_id": order_id,
            "reason": reason,
        }

    order = sdk.get_order(order_id)
    if order is None:
        return escalate("Order not found")

    slot = str(target_slot)
    suffix = slot[len("$SLOT_PICKUP_"):]
    valid_slot = (
        slot.startswith("$SLOT_PICKUP_")
        and len(suffix) > 0
        and all(character in "0123456789" for character in suffix)
        and any(character != "0" for character in suffix)
        and "NOGO" not in slot.upper()
        and "STEAM" not in slot.upper()
    )
    if not valid_slot:
        return escalate("Delivery target is not a valid pickup slot")

    supported_drinks = (
        "hot_americano",
        "takeout_americano",
        "iced_americano",
        "espresso",
        "hot_espresso",
        "takeout_espresso",
    )
    if order.get("special_request") or order.get("drink") not in supported_drinks:
        return escalate("Order requires unsupported or manual preparation")

    scene = sdk.observe_scene()
    if scene.get("control_ready") != True:
        return {"status": "rejected_observation_not_ready"}

    threshold = scene["confidence_threshold"]
    candidates = [
        obj for obj in scene["objects"]
        if obj["cup_type"] != "flexible_bag"
        and obj["confidence"] >= threshold
    ]
    confirmed_id = scene.get("operator_confirmed_cup_id")
    selected = candidates
    if confirmed_id is not None:
        selected = [obj for obj in candidates if obj["id"] == confirmed_id]

    if len(selected) != 1:
        confirmation = sdk.request_operator_confirmation(
            "A single eligible rigid cup could not be uniquely identified",
            candidates,
        )
        return {
            "status": "paused_for_operator",
            "order_id": order_id,
            "confirmation": confirmation,
        }

    cup_id = selected[0]["id"]
    placement = sdk.place_cup_under_machine(
        cup_id, command_id=command_base + ":place"
    )
    if placement.get("receipt") != "done":
        return {"status": "rejected_placement", "order_id": order_id}

    scene = sdk.observe_scene()
    if scene.get("control_ready") != True:
        return {"status": "rejected_observation_not_ready"}

    if scene.get("l2_pouring_safety", {}).get("safe_to_pour") != True:
        recentering = sdk.recenter_cup_under_spout(
            cup_id, command_id=command_base + ":recenter"
        )
        if recentering.get("receipt") != "done":
            return escalate("Cup could not be safely centered")

        scene = sdk.observe_scene()
        if scene.get("control_ready") != True:
            return {"status": "rejected_observation_not_ready"}
        if scene.get("l2_pouring_safety", {}).get("safe_to_pour") != True:
            return escalate("Pouring safety remains unsatisfied")

    brew_command_id = command_base + ":brew"
    brewing = sdk.start_coffee_machine(command_id=brew_command_id)
    receipt = brewing.get("receipt", "unknown")
    if receipt == "unknown":
        brewing = sdk.query_command_status(brew_command_id)
        receipt = brewing.get("receipt", "unknown")

    if receipt == "rejected":
        return {"status": "rejected_brew", "order_id": order_id}
    if receipt not in ("running", "done", "completed"):
        return {
            "status": "brew_status_unresolved",
            "order_id": order_id,
            "command_id": brew_command_id,
        }

    readiness = sdk.wait_for_coffee_ready(
        command_id=command_base + ":wait"
    )
    if readiness.get("receipt") != "done":
        return {"status": "rejected_brew_wait", "order_id": order_id}

    delivery = sdk.retrieve_and_deliver_cup(
        cup_id, target_slot, command_id=command_base + ":deliver"
    )
    if delivery.get("receipt") != "done":
        return {"status": "rejected_delivery", "order_id": order_id}

    completion = sdk.complete_order_delivery(
        order_id,
        cup_id,
        target_slot,
        command_id=command_base + ":complete",
    )
    if completion.get("receipt") != "done":
        return {
            "status": "delivery_finalization_failed",
            "order_id": order_id,
            "cup_id": cup_id,
            "destination_slot": target_slot,
        }

    return {
        "status": "completed",
        "order_id": order_id,
        "cup_id": cup_id,
        "destination_slot": target_slot,
    }
