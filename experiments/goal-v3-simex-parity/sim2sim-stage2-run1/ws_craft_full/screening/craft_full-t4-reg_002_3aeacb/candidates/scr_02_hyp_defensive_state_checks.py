def run_policy(sdk, params):
    order_id = params["order_id"]
    prefix = params["command_prefix"]

    def resolve(result, command_id):
        receipt = result.get("receipt") if isinstance(result, dict) else None
        if receipt == "unknown":
            for _ in range(3):
                result = sdk.query_command_status(command_id)
                receipt = (
                    result.get("receipt") if isinstance(result, dict) else None
                )
                if receipt != "unknown":
                    break
        return receipt

    def escalate(reason):
        retract_id = prefix + ":retract"
        receipt = resolve(
            sdk.retract_safe_pose(command_id=retract_id), retract_id
        )
        if receipt != "done":
            reason += "; safe retraction could not be confirmed"
        escalation_id = prefix + ":escalate"
        receipt = resolve(
            sdk.escalate_to_human_operator(
                order_id, reason, command_id=escalation_id
            ),
            escalation_id,
        )
        result = {"status": "escalated_to_human", "reason": reason}
        if receipt not in ("done", "completed"):
            result["escalation_receipt_unconfirmed"] = True
        return result

    def ready(scene):
        return isinstance(scene, dict) and scene.get("control_ready") is True

    def machine_ready(scene):
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
    if not isinstance(target_slot, str) or not target_slot:
        return escalate("Missing pickup destination")
    if "NOGO" in target_slot.upper() or "STEAM" in target_slot.upper():
        return escalate("Forbidden destination")

    slot_prefix = "$SLOT_PICKUP_"
    if not target_slot.startswith(slot_prefix):
        return escalate("Invalid pickup destination")
    slot_number = target_slot[len(slot_prefix):]
    if (
        not slot_number
        or any(character not in "0123456789" for character in slot_number)
        or int(slot_number) < 1
    ):
        return escalate("Invalid pickup destination")

    scene = sdk.observe_scene()
    if not ready(scene):
        return escalate("Observation is not ready")

    objects = scene.get("objects")
    threshold = scene.get("confidence_threshold")
    if (
        not isinstance(objects, list)
        or not isinstance(threshold, (int, float))
        or not 0 <= threshold <= 1
    ):
        return escalate("Cup observation is invalid")

    candidates = []
    for obj in objects:
        if not isinstance(obj, dict):
            return escalate("Cup observation is invalid")
        confidence = obj.get("confidence")
        if (
            obj.get("id")
            and obj.get("cup_type")
            and obj["cup_type"] != "flexible_bag"
            and isinstance(confidence, (int, float))
            and threshold <= confidence <= 1
        ):
            candidates.append(obj)

    confirmed_id = scene.get("operator_confirmed_cup_id")
    if confirmed_id:
        matches = [obj for obj in candidates if obj["id"] == confirmed_id]
        if len(matches) != 1:
            return escalate("Confirmed cup is not reliably visible")
        cup = matches[0]
    elif len(candidates) == 1:
        cup = candidates[0]
    else:
        return escalate("Cup identity is missing or ambiguous")

    cup_id = cup["id"]
    if not machine_ready(scene):
        return escalate("Machine is not ready for a fresh single-cup brew")
    if not safe_to_pour(scene):
        return escalate("Pouring safety is violated")

    start_id = prefix + ":start"
    prior = sdk.query_command_status(start_id)
    if not isinstance(prior, dict) or prior.get("receipt") != "unknown":
        return escalate("Brew command already exists or status is invalid")

    place_id = prefix + ":place"
    receipt = resolve(
        sdk.place_cup_under_machine(cup_id, command_id=place_id), place_id
    )
    if receipt != "done":
        return escalate("Cup placement could not be confirmed")

    scene = sdk.observe_scene()
    if not ready(scene):
        return escalate("Observation is not ready after placement")
    if not machine_ready(scene):
        return escalate("Machine state changed before brewing")

    if not safe_to_pour(scene):
        recenter_id = prefix + ":recenter"
        receipt = resolve(
            sdk.recenter_cup_under_spout(cup_id, command_id=recenter_id),
            recenter_id,
        )
        if receipt != "done":
            return escalate("Cup recentering could not be confirmed")
        scene = sdk.observe_scene()
        if not ready(scene):
            return escalate("Observation is not ready after recentering")
        if not machine_ready(scene):
            return escalate("Machine state changed after recentering")
        if not safe_to_pour(scene):
            return escalate("Pouring safety remains violated after recentering")

    receipt = resolve(
        sdk.start_coffee_machine(command_id=start_id), start_id
    )
    if receipt not in ("running", "done", "completed"):
        return escalate("Brew receipt is rejected or unresolved")

    wait_id = prefix + ":wait"
    receipt = resolve(
        sdk.wait_for_coffee_ready(command_id=wait_id), wait_id
    )
    if receipt != "done":
        return escalate("Coffee readiness could not be confirmed")

    scene = sdk.observe_scene()
    if not ready(scene):
        return escalate("Observation is not ready for retrieval")
    machine = scene.get("coffee_machine")
    if not isinstance(machine, dict) or machine.get("status") not in (
        "idle", "completed"
    ):
        return escalate("Machine is not ready for cup retrieval")
    if machine.get("cup_filled") is not True:
        return escalate("Filled cup could not be confirmed")

    deliver_id = prefix + ":deliver"
    receipt = resolve(
        sdk.retrieve_and_deliver_cup(
            cup_id, target_slot, command_id=deliver_id
        ),
        deliver_id,
    )
    if receipt != "done":
        return escalate("Cup delivery could not be confirmed")

    complete_id = prefix + ":complete"
    receipt = resolve(
        sdk.complete_order_delivery(
            order_id, cup_id, target_slot, command_id=complete_id
        ),
        complete_id,
    )
    if receipt != "done":
        return escalate("Delivery registration could not be confirmed")

    return {"status": "completed"}
