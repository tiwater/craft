def run_policy(sdk, params):
    order_id = params.get("order_id", "A042")
    target_slot = params.get("target_slot", "$SLOT_PICKUP_1")
    prefix = str(params.get("command_prefix", "cap"))
    order = sdk.get_order(order_id)
    if not order or order.get("destination_slot") != target_slot:
        return {"status": "rejected_order_destination"}

    scene = sdk.observe_scene()
    if not scene.get("control_ready"):
        return {"status": "rejected_observation_not_ready"}

    threshold = scene.get("confidence_threshold", 1.0)
    confirmed = scene.get("operator_confirmed_cup_id")
    candidates = [
        obj for obj in scene.get("objects", [])
        if obj.get("id")
        and obj.get("cup_type") == "dine_in_red"
        and (
            obj.get("id") == confirmed
            or obj.get("confidence", 0.0) >= threshold
        )
    ]
    if confirmed:
        candidates = [
            obj for obj in candidates if obj.get("id") == confirmed
        ]
    if len(candidates) != 1:
        return {"status": "rejected_observation_not_ready"}

    cup_id = candidates[0]["id"]
    receipt = sdk.place_cup_under_machine(
        cup_id, command_id=prefix + ":place"
    )
    if receipt.get("receipt") != "done":
        return {"status": "failed_place", "detail": receipt}

    scene = sdk.observe_scene()
    if not scene.get("control_ready"):
        return {"status": "rejected_observation_not_ready"}

    for attempt in range(6):
        receipt = sdk.recenter_cup_under_spout(
            cup_id,
            command_id=prefix + ":recenter:" + str(attempt)
        )
        if receipt.get("receipt") != "done":
            return {"status": "failed_recenter", "detail": receipt}
        scene = sdk.observe_scene()
        if not scene.get("control_ready"):
            return {"status": "rejected_observation_not_ready"}

    if not scene.get("l2_pouring_safety", {}).get("safe_to_pour"):
        return {"status": "failed_recenter"}

    brew_command_id = prefix + ":brew"
    receipt = sdk.start_coffee_machine(command_id=brew_command_id)
    if not receipt or receipt.get("receipt") not in (
        "running", "done", "rejected"
    ):
        receipt = sdk.query_command_status(brew_command_id)
    if not receipt or receipt.get("receipt") not in ("running", "done"):
        return {"status": "failed_brew_start", "detail": receipt}

    receipt = sdk.wait_for_coffee_ready(command_id=prefix + ":wait")
    if receipt.get("receipt") != "done":
        return {"status": "failed_coffee_ready", "detail": receipt}

    scene = sdk.observe_scene()
    if not scene.get("control_ready"):
        return {"status": "rejected_observation_not_ready"}
    if not scene.get("coffee_machine", {}).get("cup_filled"):
        return {"status": "failed_cup_not_filled"}

    receipt = sdk.retrieve_and_deliver_cup(
        cup_id, target_slot, command_id=prefix + ":deliver"
    )
    if receipt.get("receipt") != "done":
        return {"status": "failed_delivery", "detail": receipt}

    receipt = sdk.complete_order_delivery(
        order_id, cup_id, target_slot, command_id=prefix + ":complete"
    )
    if receipt.get("receipt") != "done":
        return {"status": "failed_completion", "detail": receipt}
    return {"status": "done", "detail": receipt}
