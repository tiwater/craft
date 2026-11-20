def run_policy(sdk, params):
    order_id = params.get("order_id", "A042")
    target_slot = params.get("target_slot", "$SLOT_PICKUP_1")
    prefix = params.get("command_prefix", "cap")
    order = sdk.get_order(order_id)
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
            obj.get("confidence", 0.0) >= threshold
            or obj.get("id") == confirmed
        )
    ]
    confirmed_candidates = [
        obj for obj in candidates if obj.get("id") == confirmed
    ]
    if len(confirmed_candidates) == 1:
        candidates = confirmed_candidates

    if len(candidates) != 1:
        sdk.escalate_to_human_operator(
            order_id,
            "Cannot uniquely identify a rigid dine-in cup.",
            command_id=prefix + ":cup_selection",
        )
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

    if not scene.get("l2_pouring_safety", {}).get("safe_to_pour"):
        receipt = sdk.recenter_cup_under_spout(
            cup_id, command_id=prefix + ":recenter"
        )
        if receipt.get("receipt") != "done":
            return {"status": "failed_recenter", "detail": receipt}
        scene = sdk.observe_scene()
        if (
            not scene.get("control_ready")
            or not scene.get("l2_pouring_safety", {}).get("safe_to_pour")
        ):
            return {"status": "failed_recenter"}

    brew_cmd_id = prefix + ":brew"
    receipt = sdk.start_coffee_machine(command_id=brew_cmd_id)
    if receipt.get("receipt") not in ("running", "done", "rejected"):
        receipt = sdk.query_command_status(brew_cmd_id)
    if receipt.get("receipt") not in ("running", "done"):
        return {"status": "failed_brew_start", "detail": receipt}

    receipt = sdk.wait_for_coffee_ready(command_id=prefix + ":wait")
    if receipt.get("receipt") != "done":
        return {"status": "failed_wait", "detail": receipt}

    scene = sdk.observe_scene()
    if (
        not scene.get("control_ready")
        or not scene.get("coffee_machine", {}).get("cup_filled")
    ):
        return {"status": "failed_coffee_ready"}

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

    return {
        "status": "done",
        "order_id": order_id,
        "cup_id": cup_id,
        "destination_slot": target_slot,
    }
