def run_policy(sdk, params):
    order_id = params["order_id"]
    prefix = params["command_prefix"]

    def receipt(result):
        return result.get("receipt") if isinstance(result, dict) else None

    def resolve(result, command_id):
        state = receipt(result)
        if state == "unknown":
            for _ in range(3):
                state = receipt(sdk.query_command_status(command_id))
                if state != "unknown":
                    break
        return state

    def escalate(reason):
        retract_id = prefix + ":retract"
        retracted = resolve(
            sdk.retract_safe_pose(command_id=retract_id), retract_id
        )
        if retracted != "done":
            reason += "; safe retraction could not be confirmed"
        escalation_id = prefix + ":escalate"
        escalated = resolve(
            sdk.escalate_to_human_operator(
                order_id, reason, command_id=escalation_id
            ),
            escalation_id,
        )
        return {
            "status": "escalated_to_human",
            "reason": reason,
            "escalation_receipt": escalated,
        }

    def ready(scene):
        return isinstance(scene, dict) and scene.get("control_ready") is True

    def select_cup(scene):
        objects = scene.get("objects")
        threshold = scene.get("confidence_threshold")
        if not isinstance(objects, list):
            return None
        if not isinstance(threshold, (int, float)):
            return None
        candidates = [
            obj for obj in objects
            if isinstance(obj, dict)
            and obj.get("id")
            and obj.get("cup_type")
            and obj["cup_type"] != "flexible_bag"
            and isinstance(obj.get("confidence"), (int, float))
            and obj["confidence"] >= threshold
        ]
        confirmed_id = scene.get("operator_confirmed_cup_id")
        if confirmed_id:
            candidates = [
                obj for obj in candidates if obj["id"] == confirmed_id
            ]
        return candidates[0] if len(candidates) == 1 else None

    def fresh_machine(scene):
        machine = scene.get("coffee_machine")
        return (
            isinstance(machine, dict)
            and machine.get("status") == "idle"
            and machine.get("cup_filled") is False
        )

    def safe_to_pour(scene):
        safety = scene.get("l2_pouring_safety")
        return isinstance(safety, dict) and safety.get("safe_to_pour") is True

    order = sdk.get_order(order_id)
    if not isinstance(order, dict) or not order:
        return escalate("Order not found")
    if order.get("special_request"):
        return escalate("Order requires manual preparation")

    target_slot = params.get("target_slot") or order.get("destination_slot")
    slot_prefix = "$SLOT_PICKUP_"
    if not isinstance(target_slot, str) or not target_slot.startswith(slot_prefix):
        return escalate("Invalid pickup destination")
    slot_number = target_slot[len(slot_prefix):]
    if (
        not slot_number
        or any(character not in "0123456789" for character in slot_number)
        or not any(character != "0" for character in slot_number)
    ):
        return escalate("Invalid pickup destination")

    scene = sdk.observe_scene()
    if not ready(scene):
        return escalate("Observation is not ready")
    cup = select_cup(scene)
    if cup is None:
        return escalate("Cup identity is missing or ambiguous")
    cup_id = cup["id"]

    if not fresh_machine(scene):
        return escalate("Machine is not ready for a fresh single-cup brew")
    if not safe_to_pour(scene):
        return escalate("Pouring safety is violated")

    start_id = prefix + ":start"
    if receipt(sdk.query_command_status(start_id)) != "unknown":
        return escalate("Brew command already exists or its status is invalid")

    place_id = prefix + ":place"
    if resolve(
        sdk.place_cup_under_machine(cup_id, command_id=place_id), place_id
    ) != "done":
        return escalate("Cup placement could not be confirmed")

    scene = sdk.observe_scene()
    if not ready(scene):
        return escalate("Observation is not ready after placement")
    observed_cup = select_cup(scene)
    if observed_cup is None or observed_cup["id"] != cup_id:
        return escalate("Cup identity could not be confirmed after placement")
    if not fresh_machine(scene):
        return escalate("Machine state changed before brewing")

    if not safe_to_pour(scene):
        recenter_id = prefix + ":recenter"
        if resolve(
            sdk.recenter_cup_under_spout(cup_id, command_id=recenter_id),
            recenter_id,
        ) != "done":
            return escalate("Cup recentering could not be confirmed")

        scene = sdk.observe_scene()
        if not ready(scene):
            return escalate("Observation is not ready after recentering")
        observed_cup = select_cup(scene)
        if observed_cup is None or observed_cup["id"] != cup_id:
            return escalate("Cup identity could not be confirmed after recentering")
        if not fresh_machine(scene):
            return escalate("Machine state changed during recentering")
        if not safe_to_pour(scene):
            return escalate("Pouring safety is violated after recentering")

    state = resolve(sdk.start_coffee_machine(command_id=start_id), start_id)
    if state not in ("running", "done", "completed"):
        return escalate("Brew receipt is rejected or unresolved")

    wait_id = prefix + ":wait"
    if resolve(
        sdk.wait_for_coffee_ready(command_id=wait_id), wait_id
    ) != "done":
        return escalate("Coffee readiness could not be confirmed")

    scene = sdk.observe_scene()
    if not ready(scene):
        return escalate("Observation is not ready before retrieval")
    machine = scene.get("coffee_machine")
    if not isinstance(machine, dict):
        return escalate("Machine observation is missing")
    if machine.get("status") not in ("idle", "completed"):
        return escalate("Machine is not ready for cup retrieval")
    if machine.get("cup_filled") is not True:
        return escalate("Filled cup could not be confirmed")
    observed_cup = select_cup(scene)
    if observed_cup is None or observed_cup["id"] != cup_id:
        return escalate("Cup identity could not be confirmed before retrieval")

    deliver_id = prefix + ":deliver"
    if resolve(
        sdk.retrieve_and_deliver_cup(
            cup_id, target_slot, command_id=deliver_id
        ),
        deliver_id,
    ) != "done":
        return escalate("Cup delivery could not be confirmed")

    complete_id = prefix + ":complete"
    if resolve(
        sdk.complete_order_delivery(
            order_id, cup_id, target_slot, command_id=complete_id
        ),
        complete_id,
    ) != "done":
        return escalate("Delivery registration could not be confirmed")

    return {"status": "completed"}
