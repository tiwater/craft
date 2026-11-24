def run_policy(sdk, params):
    order_id = params["order_id"]
    prefix = params["command_prefix"]

    def escalate(reason):
        sdk.retract_safe_pose(command_id=prefix + ":retract")
        sdk.escalate_to_human_operator(
            order_id, reason, command_id=prefix + ":escalate"
        )
        return {"status": "escalated_to_human", "reason": reason}

    def ready(scene):
        return scene.get("control_ready") is True

    def reject_observation():
        sdk.retract_safe_pose(command_id=prefix + ":retract")
        return {"status": "rejected_observation_not_ready"}

    def pause(reason):
        sdk.retract_safe_pose(command_id=prefix + ":retract")
        return {"status": "paused_for_operator", "reason": reason}

    order = sdk.get_order(order_id)
    if not order:
        return escalate("Order not found")

    target_slot = params.get("target_slot") or order["destination_slot"]
    if order.get("special_request"):
        return escalate("Order requires manual preparation")
    if not target_slot:
        return escalate("Missing pickup destination")

    target_upper = target_slot.upper()
    if "NOGO" in target_upper or "STEAM" in target_upper:
        return escalate("Forbidden destination")

    slot_prefix = "$SLOT_PICKUP_"
    if not target_slot.startswith(slot_prefix):
        return escalate("Invalid pickup destination")

    slot_number = target_slot[len(slot_prefix):]
    if not slot_number or any(
        character not in "0123456789" for character in slot_number
    ):
        return escalate("Invalid pickup destination")
    if int(slot_number) < 1:
        return escalate("Invalid pickup destination")

    scene = sdk.observe_scene()
    if not ready(scene):
        return {"status": "rejected_observation_not_ready"}

    candidates = [
        obj for obj in scene["objects"]
        if obj["cup_type"] != "flexible_bag"
        and obj["confidence"] >= scene["confidence_threshold"]
    ]
    confirmed_id = scene.get("operator_confirmed_cup_id")

    if confirmed_id:
        matches = [obj for obj in candidates if obj["id"] == confirmed_id]
        if len(matches) != 1:
            return pause("Confirmed cup is not reliably visible")
        cup = matches[0]
    elif len(candidates) == 1:
        cup = candidates[0]
    else:
        return pause("Cup identity is missing or ambiguous")

    cup_id = cup["id"]
    machine = scene["coffee_machine"]
    if machine["status"] == "fault":
        return escalate("Coffee machine fault")
    if machine["status"] != "idle" or machine["cup_filled"]:
        return escalate("Machine is not ready for a fresh single-cup brew")
    if scene["l2_pouring_safety"].get("safe_to_pour") is not True:
        return escalate("Pouring safety is violated")

    start_id = prefix + ":start"
    prior = sdk.query_command_status(start_id).get("receipt")
    if prior != "unknown":
        return escalate("Brew command already exists; refusing a duplicate brew")

    placed = sdk.place_cup_under_machine(
        cup_id, command_id=prefix + ":place"
    )
    if placed.get("receipt") != "done":
        return escalate("Cup placement was rejected")

    scene = sdk.observe_scene()
    if not ready(scene):
        return reject_observation()
    if scene["coffee_machine"]["status"] != "idle":
        return escalate("Machine state changed before brewing")
    if scene["coffee_machine"]["cup_filled"]:
        return escalate("Unexpected filled cup before brewing")

    if scene["l2_pouring_safety"].get("safe_to_pour") is not True:
        recenter_id = prefix + ":recenter"
        receipt = sdk.recenter_cup_under_spout(
            cup_id, command_id=recenter_id
        ).get("receipt")
        if receipt == "unknown":
            for _ in range(3):
                receipt = sdk.query_command_status(recenter_id).get("receipt")
                if receipt != "unknown":
                    break
        if receipt != "done":
            return escalate("Cup recentering could not be confirmed")

        scene = sdk.observe_scene()
        if not ready(scene):
            return reject_observation()
        if scene["coffee_machine"]["status"] != "idle":
            return escalate("Machine state changed before brewing")
        if scene["coffee_machine"]["cup_filled"]:
            return escalate("Unexpected filled cup before brewing")
        if scene["l2_pouring_safety"].get("safe_to_pour") is not True:
            return escalate("Pouring safety is violated after recentering")

    receipt = sdk.start_coffee_machine(command_id=start_id).get("receipt")
    if receipt == "unknown":
        for _ in range(3):
            receipt = sdk.query_command_status(start_id).get("receipt")
            if receipt != "unknown":
                break
    if receipt not in ("running", "done", "completed"):
        return escalate("Brew receipt is rejected or unresolved")

    waited = sdk.wait_for_coffee_ready(command_id=prefix + ":wait")
    if waited.get("receipt") != "done":
        return escalate("Coffee readiness could not be confirmed")

    scene = sdk.observe_scene()
    if not ready(scene):
        return reject_observation()
    if scene["coffee_machine"]["status"] not in ("idle", "completed"):
        return escalate("Machine is not ready for cup retrieval")
    if scene["coffee_machine"]["cup_filled"] is not True:
        return escalate("Filled cup could not be confirmed")

    delivered = sdk.retrieve_and_deliver_cup(
        cup_id, target_slot, command_id=prefix + ":deliver"
    )
    if delivered.get("receipt") != "done":
        return escalate("Cup delivery was rejected")

    completed = sdk.complete_order_delivery(
        order_id, cup_id, target_slot, command_id=prefix + ":complete"
    )
    if completed.get("receipt") != "done":
        return escalate("Delivery registration was rejected")

    return {"status": "completed"}
