def run_policy(sdk, params):
    order_id = params.get("order_id", "A042")
    target_slot = params.get("target_slot", "$SLOT_PICKUP_1")
    prefix = params.get("command_prefix", "cap")
    order = sdk.get_order(order_id)
    scene = sdk.observe_scene()

    if not scene.get("control_ready"):
        return {"status": "rejected_observation_not_ready"}

    threshold = scene.get("confidence_threshold", 1.0)
    confirmed_id = scene.get("operator_confirmed_cup_id")
    candidates = []
    for obj in scene.get("objects", []):
        if obj.get("cup_type") == "flexible_bag":
            continue
        if obj.get("cup_type") != "dine_in_red":
            continue
        if not obj.get("id"):
            continue
        if obj.get("confidence", 0.0) >= threshold or obj.get("id") == confirmed_id:
            candidates.append(obj)

    confirmed = [
        obj for obj in candidates
        if obj.get("id") == confirmed_id
    ]
    if len(confirmed) == 1:
        candidates = confirmed

    if len(candidates) != 1:
        sdk.escalate_to_human_operator(
            order_id,
            "Cannot uniquely bind a suitable rigid dine-in cup from live observation.",
            command_id=prefix + "-cup-escalation"
        )
        return {"status": "rejected_observation_not_ready"}

    cup_id = candidates[0]["id"]
    receipt = sdk.place_cup_under_machine(
        cup_id, command_id=prefix + "-place"
    )
    if receipt.get("receipt") != "done":
        return {"status": "failed_place", "receipt": receipt}

    scene = sdk.observe_scene()
    if not scene.get("control_ready"):
        return {"status": "rejected_observation_not_ready"}

    if not scene.get("l2_pouring_safety", {}).get("safe_to_pour"):
        receipt = sdk.recenter_cup_under_spout(
            cup_id, command_id=prefix + "-recenter"
        )
        if receipt.get("receipt") != "done":
            return {"status": "failed_recenter", "receipt": receipt}
        scene = sdk.observe_scene()

    if not scene.get("control_ready"):
        return {"status": "rejected_observation_not_ready"}
    if not scene.get("l2_pouring_safety", {}).get("safe_to_pour"):
        return {"status": "failed_recenter"}

    brew_cmd_id = prefix + "-brew"
    receipt = sdk.start_coffee_machine(command_id=brew_cmd_id)
    if not receipt or receipt.get("receipt") not in ("running", "done", "rejected"):
        receipt = sdk.query_command_status(brew_cmd_id)
    if not receipt or receipt.get("receipt") not in ("running", "done"):
        return {"status": "failed_brew_start", "receipt": receipt}

    receipt = sdk.wait_for_coffee_ready(command_id=prefix + "-wait")
    if receipt.get("receipt") != "done":
        return {"status": "failed_wait", "receipt": receipt}

    scene = sdk.observe_scene()
    if not scene.get("control_ready"):
        return {"status": "rejected_observation_not_ready"}
    if not scene.get("coffee_machine", {}).get("cup_filled"):
        return {"status": "failed_coffee_not_filled"}

    receipt = sdk.retrieve_and_deliver_cup(
        cup_id, target_slot, command_id=prefix + "-deliver"
    )
    if receipt.get("receipt") != "done":
        return {"status": "failed_delivery", "receipt": receipt}

    receipt = sdk.complete_order_delivery(
        order_id, cup_id, target_slot, command_id=prefix + "-complete"
    )
    if receipt.get("receipt") != "done":
        return {"status": "failed_completion", "receipt": receipt}

    return {
        "status": "done",
        "order_id": order_id,
        "cup_id": cup_id,
        "destination_slot": target_slot
    }
