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
        if obj.get("cup_type") == "dine_in_red"
        and obj.get("id")
        and (
            obj.get("confidence", 0.0) >= threshold
            or obj.get("id") == confirmed
        )
    ]
    if confirmed:
        selected = [obj for obj in candidates if obj.get("id") == confirmed]
        if len(selected) == 1:
            candidates = selected

    if len(candidates) != 1:
        sdk.escalate_to_human_operator(
            order_id,
            "Cannot uniquely bind one suitable dine-in cup.",
            command_id=prefix + "-selection-escalation"
        )
        return {"status": "rejected_cup_selection"}

    cup_id = candidates[0]["id"]
    placed = sdk.place_cup_under_machine(
        cup_id, command_id=prefix + "-place"
    )
    if placed.get("receipt") != "done":
        return {"status": "failed_place", "receipt": placed}

    scene = sdk.observe_scene()
    for attempt in range(5):
        if not scene.get("control_ready"):
            return {"status": "rejected_observation_not_ready"}
        if scene.get("l2_pouring_safety", {}).get("safe_to_pour") is True:
            break
        centered = sdk.recenter_cup_under_spout(
            cup_id,
            command_id=prefix + "-recenter-" + str(attempt)
        )
        if centered.get("receipt") != "done":
            return {"status": "failed_recenter", "receipt": centered}
        scene = sdk.observe_scene()

    if (
        not scene.get("control_ready")
        or scene.get("l2_pouring_safety", {}).get("safe_to_pour") is not True
    ):
        return {"status": "failed_recenter"}

    brew_command = prefix + "-brew"
    brew = sdk.start_coffee_machine(command_id=brew_command)
    if brew.get("receipt") not in ("running", "done", "rejected"):
        brew = sdk.query_command_status(brew_command)
    if brew.get("receipt") not in ("running", "done"):
        return {"status": "failed_brew_start", "receipt": brew}

    ready = sdk.wait_for_coffee_ready(command_id=prefix + "-wait-ready")
    if ready.get("receipt") != "done":
        return {"status": "failed_wait", "receipt": ready}

    scene = sdk.observe_scene()
    if (
        not scene.get("control_ready")
        or scene.get("coffee_machine", {}).get("cup_filled") is not True
    ):
        return {"status": "failed_ready_verification"}

    delivered = sdk.retrieve_and_deliver_cup(
        cup_id, target_slot, command_id=prefix + "-deliver"
    )
    if delivered.get("receipt") != "done":
        return {"status": "failed_delivery", "receipt": delivered}

    completed = sdk.complete_order_delivery(
        order_id, cup_id, target_slot,
        command_id=prefix + "-complete"
    )
    if completed.get("receipt") != "done":
        return {"status": "failed_completion", "receipt": completed}

    return {
        "status": "done",
        "order_id": order_id,
        "cup_id": cup_id,
        "destination_slot": target_slot,
        "receipt": completed
    }
