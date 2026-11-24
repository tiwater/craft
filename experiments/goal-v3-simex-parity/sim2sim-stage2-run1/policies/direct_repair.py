def run_policy(sdk, params):
    order_id = params["order_id"]
    prefix = params["command_prefix"]
    start_id = prefix + ":start"

    def escalate(reason):
        sdk.retract_safe_pose(command_id=prefix + ":retract")
        sdk.escalate_to_human_operator(
            order_id, reason, command_id=prefix + ":escalate"
        )
        return {"status": "escalated_to_human", "reason": reason}

    def pause(reason, candidates):
        sdk.request_operator_confirmation(reason, candidates)
        return {"status": "paused_for_operator"}

    def observation_rejected():
        sdk.retract_safe_pose(command_id=prefix + ":retract")
        return {"status": "rejected_observation_not_ready"}

    order = sdk.get_order(order_id)
    if not order:
        return escalate("Order not found")
    if order.get("special_request"):
        return escalate("Order requires manual preparation")

    target_slot = params.get("target_slot") or order["destination_slot"]
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
    if scene.get("control_ready") is not True:
        return {"status": "rejected_observation_not_ready"}

    machine = scene["coffee_machine"]
    if machine["status"] == "fault":
        return escalate("Coffee machine fault")

    candidates = [
        obj for obj in scene["objects"]
        if obj["cup_type"] != "flexible_bag"
        and obj["confidence"] >= scene["confidence_threshold"]
    ]
    confirmed_id = scene.get("operator_confirmed_cup_id")
    if confirmed_id:
        matches = [obj for obj in candidates if obj["id"] == confirmed_id]
        if len(matches) != 1:
            return pause("Confirmed cup is not reliably visible", candidates)
        cup = matches[0]
    elif len(candidates) == 1:
        cup = candidates[0]
    else:
        return pause("Cup identity is missing or ambiguous", candidates)

    cup_id = cup["id"]
    receipt = sdk.query_command_status(start_id).get("receipt")
    fresh_machine = (
        machine["status"] == "idle" and not machine["cup_filled"]
    )

    if receipt == "unknown" and not fresh_machine:
        for attempt in range(3):
            receipt = sdk.query_command_status(start_id).get("receipt")
            if receipt != "unknown":
                break

    if receipt == "unknown":
        if not fresh_machine:
            return escalate("Existing brew ownership could not be confirmed")

        needs_recenter = (
            scene["l2_pouring_safety"].get("safe_to_pour") is not True
        )
        placed = sdk.place_cup_under_machine(
            cup_id, command_id=prefix + ":place"
        )
        if placed.get("receipt") != "done":
            return escalate("Cup placement was rejected")

        scene = sdk.observe_scene()
        if scene.get("control_ready") is not True:
            return observation_rejected()
        machine = scene["coffee_machine"]
        if machine["status"] != "idle" or machine["cup_filled"]:
            return escalate("Machine state changed before brewing")

        if needs_recenter or (
            scene["l2_pouring_safety"].get("safe_to_pour") is not True
        ):
            recentered = sdk.recenter_cup_under_spout(
                cup_id, command_id=prefix + ":recenter"
            )
            if recentered.get("receipt") != "done":
                return escalate("Cup recentering was rejected")

            scene = sdk.observe_scene()
            if scene.get("control_ready") is not True:
                return observation_rejected()
            machine = scene["coffee_machine"]
            if machine["status"] != "idle" or machine["cup_filled"]:
                return escalate("Machine state changed during cup recentering")

        if scene["l2_pouring_safety"].get("safe_to_pour") is not True:
            return escalate("Pouring safety remains violated after cup alignment")

        receipt = sdk.start_coffee_machine(command_id=start_id).get("receipt")
        if receipt == "unknown":
            for attempt in range(3):
                receipt = sdk.query_command_status(start_id).get("receipt")
                if receipt != "unknown":
                    break
    elif receipt in ("running", "done", "completed"):
        if machine["status"] == "cleaning":
            return escalate("Machine is cleaning during brew recovery")
        if machine["status"] in ("running", "busy"):
            if scene["l2_pouring_safety"].get("safe_to_pour") is not True:
                return escalate("Pouring safety violated during brew recovery")

    if receipt not in ("running", "done", "completed"):
        return escalate("Brew receipt is rejected or unresolved")

    if not (
        scene["coffee_machine"]["status"] in ("idle", "completed")
        and scene["coffee_machine"]["cup_filled"] is True
        and receipt in ("done", "completed")
    ):
        waited = sdk.wait_for_coffee_ready(command_id=prefix + ":wait")
        if waited.get("receipt") != "done":
            return escalate("Coffee readiness could not be confirmed")

    scene = sdk.observe_scene()
    if scene.get("control_ready") is not True:
        return observation_rejected()
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
