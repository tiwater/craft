def run_policy(sdk, params):
    order_id = params.get("order_id", "A042")
    target_slot = params.get("target_slot", "$SLOT_PICKUP_1")
    prefix = params.get("command_prefix", "cap")
    command_base = str(prefix) + ":" + str(order_id)

    def resolve(result, command_id):
        for attempt in range(12):
            if isinstance(result, dict):
                status = str(result.get("status", "")).lower()
                if status in ("failed", "rejected", "cancelled", "error"):
                    return False
                if status in (
                    "completed", "succeeded", "success", "done",
                    "ready", "delivered"
                ):
                    return True
                if not status and (
                    result.get("ok") is True or result.get("success") is True
                ):
                    return True
            result = sdk.query_command_status(command_id)
        return False

    def stop(reason):
        retract_id = command_base + ":retract"
        retracted = resolve(
            sdk.retract_safe_pose(command_id=retract_id), retract_id
        )
        escalation_id = command_base + ":escalate"
        escalated = resolve(
            sdk.escalate_to_human_operator(
                order_id, reason, command_id=escalation_id
            ),
            escalation_id
        )
        return {
            "status": "escalated" if escalated else "needs_human",
            "order_id": order_id,
            "reason": reason,
            "retracted": retracted
        }

    order = sdk.get_order(order_id)
    if not isinstance(order, dict):
        return stop("Order could not be verified.")
    if str(order.get("status", "")).lower() in (
        "completed", "delivered", "cancelled", "canceled"
    ):
        return {
            "status": "not_started",
            "order_id": order_id,
            "reason": "Order is already finished or cancelled."
        }

    required_type = order.get("required_cup_type", order.get("cup_type"))
    scene = sdk.observe_scene()
    if scene.get("control_ready") is not True:
        return stop("Robot control is not ready.")
    if scene.get("coffee_machine", {}).get("cup_filled") is True:
        return stop("Machine already contains a filled cup.")

    def candidates_from(scene):
        candidates = []
        for obj in scene.get("objects", []):
            if (
                obj.get("id") is not None
                and obj.get("world") is not None
                and obj.get("cup_type") is not None
                and (
                    required_type is None
                    or obj.get("cup_type") == required_type
                )
            ):
                candidates.append(obj)
        return candidates

    candidates = candidates_from(scene)
    threshold = scene.get("confidence_threshold", 0.95)
    confirmed_id = scene.get("operator_confirmed_cup_id")
    selected = None
    for candidate in candidates:
        if candidate.get("id") == confirmed_id:
            selected = candidate

    confident = [
        candidate for candidate in candidates
        if candidate.get("confidence", 0) >= threshold
    ]
    if selected is None and required_type is not None and len(confident) == 1:
        selected = confident[0]

    if selected is None:
        confirmation = sdk.request_operator_confirmation(
            "Confirm the correct cup for order " + str(order_id) + ".",
            candidates
        )
        scene = sdk.observe_scene()
        candidates = candidates_from(scene)
        confirmed_id = scene.get("operator_confirmed_cup_id")
        if confirmed_id is None and isinstance(confirmation, dict):
            confirmed_id = confirmation.get("operator_confirmed_cup_id")
            if confirmation.get("confirmed") is True:
                confirmed_id = confirmation.get("cup_id", confirmed_id)
        for candidate in candidates:
            if candidate.get("id") == confirmed_id:
                selected = candidate
        if selected is None:
            return stop("No valid cup was confirmed.")

    cup_id = selected["id"]
    scene = sdk.observe_scene()
    if scene.get("control_ready") is not True:
        return stop("Robot control became unavailable.")
    current_cup = None
    for candidate in candidates_from(scene):
        if candidate["id"] == cup_id:
            current_cup = candidate
    if current_cup is None:
        return stop("Selected cup is no longer visible.")

    command_id = command_base + ":move"
    if not resolve(
        sdk.move_to_target(current_cup["world"], command_id=command_id),
        command_id
    ):
        return stop("Movement did not complete successfully.")

    scene = sdk.observe_scene()
    if scene.get("control_ready") is not True:
        return stop("Robot control is unavailable before cup placement.")
    command_id = command_base + ":place"
    if not resolve(
        sdk.place_cup_under_machine(cup_id, command_id=command_id),
        command_id
    ):
        return stop("Cup placement did not complete successfully.")

    scene = sdk.observe_scene()
    if scene.get("control_ready") is not True:
        return stop("Robot control is unavailable after cup placement.")
    if scene.get("l2_pouring_safety", {}).get("safe_to_pour") is not True:
        command_id = command_base + ":recenter"
        if not resolve(
            sdk.recenter_cup_under_spout(cup_id, command_id=command_id),
            command_id
        ):
            return stop("Cup recentering did not complete successfully.")
        scene = sdk.observe_scene()

    if scene.get("control_ready") is not True:
        return stop("Robot control is unavailable before brewing.")
    if scene.get("l2_pouring_safety", {}).get("safe_to_pour") is not True:
        return stop("Pouring safety has not been established.")
    machine = scene.get("coffee_machine", {})
    if machine.get("cup_filled") is True:
        return stop("Cup is already filled; additional brewing is unsafe.")
    if str(machine.get("status", "")).lower() not in ("ready", "idle"):
        return stop("Coffee machine is not ready.")

    command_id = command_base + ":start"
    if not resolve(
        sdk.start_coffee_machine(command_id=command_id), command_id
    ):
        return stop("Brewing command failed or its outcome is uncertain.")

    command_id = command_base + ":wait"
    if not resolve(
        sdk.wait_for_coffee_ready(command_id=command_id), command_id
    ):
        return stop("Coffee readiness was not confirmed.")

    scene = sdk.observe_scene()
    if scene.get("control_ready") is not True:
        return stop("Robot control is unavailable before delivery.")
    machine = scene.get("coffee_machine", {})
    if machine.get("cup_filled") is not True:
        return stop("The cup has not been verified as filled.")
    if str(machine.get("status", "")).lower() not in (
        "ready", "idle", "done", "completed", "finished"
    ):
        return stop("The machine has not safely finished brewing.")

    command_id = command_base + ":deliver"
    if not resolve(
        sdk.retrieve_and_deliver_cup(
            cup_id, target_slot, command_id=command_id
        ),
        command_id
    ):
        return stop("Cup delivery failed or its outcome is uncertain.")

    command_id = command_base + ":complete"
    if not resolve(
        sdk.complete_order_delivery(
            order_id, cup_id, target_slot, command_id=command_id
        ),
        command_id
    ):
        return stop("Delivery occurred, but order completion is unconfirmed.")

    return {
        "status": "completed",
        "order_id": order_id,
        "cup_id": cup_id,
        "target_slot": target_slot
    }
