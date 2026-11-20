def run_policy(sdk, params):
    order_id = params.get("order_id", "A042")
    target_slot = params.get("target_slot", "$SLOT_PICKUP_1")
    prefix = params.get("command_prefix", "cap")
    order = sdk.get_order(order_id)
    if not order or order.get("order_id") != order_id:
        return {"status": "rejected_order"}

    scene = sdk.observe_scene()
    if not scene or scene.get("control_ready") is not True:
        return {"status": "rejected_observation_not_ready"}

    threshold = scene.get("confidence_threshold")
    if threshold is None:
        return {"status": "rejected_observation_not_ready"}

    confirmed_id = scene.get("operator_confirmed_cup_id")
    suitable = []
    candidates = []
    for cup in scene.get("objects", []):
        if cup.get("cup_type") == "flexible_bag":
            continue
        if cup.get("id") and cup.get("cup_type") == "dine_in_red":
            suitable.append(cup)
            if (
                cup.get("id") == confirmed_id
                or cup.get("confidence", 0) >= threshold
            ):
                candidates.append(cup)

    confirmed = [
        cup for cup in candidates if cup.get("id") == confirmed_id
    ]
    if len(confirmed) == 1:
        candidates = confirmed
    if len(candidates) != 1:
        reason = "A single suitable dine-in cup could not be bound."
        sdk.request_operator_confirmation(reason, suitable)
        return {
            "status": "rejected_observation_not_ready",
            "reason": reason,
        }

    cup_id = candidates[0]["id"]
    command_base = str(prefix) + ":" + str(order_id)

    receipt = sdk.place_cup_under_machine(
        cup_id, command_id=command_base + ":place"
    )
    if not receipt or receipt.get("receipt") != "done":
        return {"status": "failed_place", "receipt": receipt}

    # Placement invalidates the initial observation.
    scene = sdk.observe_scene()
    if not scene or scene.get("control_ready") is not True:
        return {"status": "failed_place", "reason": "Control is not ready."}

    if scene.get("l2_pouring_safety", {}).get("safe_to_pour") is not True:
        receipt = sdk.recenter_cup_under_spout(
            cup_id, command_id=command_base + ":recenter"
        )
        if not receipt or receipt.get("receipt") != "done":
            return {"status": "failed_recenter", "receipt": receipt}

        # Re-observe only when recentering changed the cup's pose.
        scene = sdk.observe_scene()
        if (
            not scene
            or scene.get("control_ready") is not True
            or scene.get("l2_pouring_safety", {}).get("safe_to_pour") is not True
        ):
            return {"status": "failed_recenter"}

    brew_command_id = command_base + ":brew"
    receipt = sdk.start_coffee_machine(command_id=brew_command_id)
    brew_status = receipt.get("receipt") if receipt else None
    if brew_status not in ("running", "done", "rejected"):
        # Reconcile unknown or missing receipts once; never reissue brew.
        receipt = sdk.query_command_status(brew_command_id)
        brew_status = receipt.get("receipt") if receipt else None
    if brew_status not in ("running", "done"):
        return {
            "status": "failed_brew_start",
            "command_id": brew_command_id,
            "receipt": receipt,
        }

    receipt = sdk.wait_for_coffee_ready(
        command_id=command_base + ":wait"
    )
    if not receipt or receipt.get("receipt") != "done":
        return {"status": "failed_wait", "receipt": receipt}

    # A wait receipt does not replace control and fill verification.
    scene = sdk.observe_scene()
    if (
        not scene
        or scene.get("control_ready") is not True
        or scene.get("coffee_machine", {}).get("cup_filled") is not True
    ):
        return {"status": "failed_fill_verification"}

    receipt = sdk.retrieve_and_deliver_cup(
        cup_id, target_slot, command_id=command_base + ":deliver"
    )
    if not receipt or receipt.get("receipt") != "done":
        return {"status": "failed_delivery", "receipt": receipt}

    receipt = sdk.complete_order_delivery(
        order_id,
        cup_id,
        target_slot,
        command_id=command_base + ":complete",
    )
    if not receipt or receipt.get("receipt") != "done":
        return {"status": "failed_completion", "receipt": receipt}

    return {
        "status": "completed",
        "order_id": order_id,
        "cup_id": cup_id,
        "destination_slot": target_slot,
        "brew_command_id": brew_command_id,
    }
