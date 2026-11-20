def run_policy(sdk, params):
    order_id = params.get("order_id", "A042")
    target_slot = params.get("target_slot", "$SLOT_PICKUP_1")
    prefix = params.get("command_prefix", "cap")
    order = sdk.get_order(order_id)
    if not order or order.get("order_id") != order_id:
        return {"status": "rejected_order"}

    if order.get("destination_slot") != target_slot:
        return {"status": "rejected_destination_mismatch"}

    scene = sdk.observe_scene()
    if not scene or scene.get("control_ready") is not True:
        return {"status": "rejected_observation_not_ready"}

    threshold = scene.get("confidence_threshold")
    if threshold is None:
        return {"status": "rejected_observation_not_ready"}

    confirmed_id = scene.get("operator_confirmed_cup_id")
    cups = [
        obj for obj in scene.get("objects", [])
        if obj.get("id")
        and obj.get("cup_type") != "flexible_bag"
        and obj.get("cup_type") == "dine_in_red"
    ]
    if confirmed_id:
        candidates = [obj for obj in cups if obj.get("id") == confirmed_id]
    else:
        candidates = [
            obj for obj in cups
            if obj.get("confidence", 0) >= threshold
        ]

    if len(candidates) != 1:
        sdk.escalate_to_human_operator(
            order_id,
            "Cannot uniquely bind a suitable dine-in cup from live scene evidence.",
            command_id=prefix + ":confirm_cup",
        )
        return {"status": "rejected_single_cup_not_bound"}

    cup_id = candidates[0]["id"]
    receipt = sdk.place_cup_under_machine(
        cup_id, command_id=prefix + ":place"
    )
    if not receipt or receipt.get("receipt") != "done":
        return {"status": "failed_place", "receipt": receipt}

    scene = sdk.observe_scene()
    if not scene or scene.get("control_ready") is not True:
        return {"status": "rejected_observation_not_ready"}

    if scene.get("l2_pouring_safety", {}).get("safe_to_pour") is not True:
        receipt = sdk.recenter_cup_under_spout(
            cup_id, command_id=prefix + ":recenter"
        )
        if not receipt or receipt.get("receipt") != "done":
            return {"status": "failed_recenter", "receipt": receipt}
        scene = sdk.observe_scene()
        if (
            not scene
            or scene.get("control_ready") is not True
            or scene.get("l2_pouring_safety", {}).get("safe_to_pour") is not True
        ):
            return {"status": "failed_recenter"}

    brew_cmd_id = prefix + ":brew"
    receipt = sdk.start_coffee_machine(command_id=brew_cmd_id)
    brew_status = receipt.get("receipt") if receipt else None
    if brew_status not in ("running", "done", "rejected"):
        receipt = sdk.query_command_status(brew_cmd_id)
        brew_status = receipt.get("receipt") if receipt else None
    if brew_status not in ("running", "done"):
        return {
            "status": "failed_brew_start",
            "command_id": brew_cmd_id,
            "receipt": receipt,
        }

    receipt = sdk.wait_for_coffee_ready(command_id=prefix + ":wait")
    if not receipt or receipt.get("receipt") != "done":
        return {"status": "failed_wait", "receipt": receipt}

    scene = sdk.observe_scene()
    if (
        not scene
        or scene.get("control_ready") is not True
        or scene.get("coffee_machine", {}).get("cup_filled") is not True
    ):
        return {"status": "failed_cup_filled_verification"}

    receipt = sdk.retrieve_and_deliver_cup(
        cup_id, target_slot, command_id=prefix + ":deliver"
    )
    if not receipt or receipt.get("receipt") != "done":
        return {"status": "failed_delivery", "receipt": receipt}

    receipt = sdk.complete_order_delivery(
        order_id, cup_id, target_slot, command_id=prefix + ":complete"
    )
    if not receipt or receipt.get("receipt") != "done":
        return {"status": "failed_completion", "receipt": receipt}

    return {
        "status": "done",
        "order_id": order_id,
        "cup_id": cup_id,
        "destination_slot": target_slot,
        "receipt": receipt,
    }
