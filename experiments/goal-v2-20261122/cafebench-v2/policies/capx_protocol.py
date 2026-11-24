def run_policy(sdk, params):
    def command_id(step):
        return prefix + ":" + step

    def escalate(reason):
        retraction = sdk.retract_safe_pose(command_id=command_id("retract"))
        escalation = sdk.escalate_to_human_operator(
            order_id, reason, command_id=command_id("escalate")
        )
        return {
            "status": "escalated_to_human",
            "reason": reason,
            "retraction": retraction,
            "escalation": escalation,
        }

    def done(result):
        return bool(result) and result.get("receipt") == "done"

    def resolve_receipt(result):
        receipt = result.get("receipt", "unknown") if result else "unknown"
        for attempt in range(4):
            if receipt in ("running", "done", "completed", "rejected"):
                return receipt
            result = sdk.query_command_status(command_id("brew"))
            receipt = result.get("receipt", "unknown") if result else "unknown"
        return receipt

    def select_cup(scene):
        candidates = [
            obj
            for obj in scene["objects"]
            if obj["cup_type"] != "flexible_bag"
            and obj["confidence"] >= scene["confidence_threshold"]
            and obj.get("id")
        ]
        confirmed_id = scene.get("operator_confirmed_cup_id")
        selected = candidates
        if confirmed_id:
            selected = [obj for obj in candidates if obj["id"] == confirmed_id]
        if len(selected) == 1:
            return {"cup": selected[0], "candidates": candidates}
        return {"cup": None, "candidates": candidates}

    def pause(selection):
        reason = "Cup identity is missing or ambiguous; confirm a visible rigid cup."
        confirmation = sdk.request_operator_confirmation(
            reason, selection["candidates"]
        )
        return {
            "status": "paused_for_operator",
            "reason": reason,
            "confirmation": confirmation,
        }

    order_id = params.get("order_id")
    prefix = params.get("command_prefix")
    if not order_id or not prefix:
        return {"status": "rejected_invalid_parameters"}

    order = sdk.get_order(order_id)
    if not order:
        return escalate("The requested order does not exist.")

    target_slot = params.get("target_slot") or order["destination_slot"]
    marker = "$SLOT_PICKUP_"
    if (
        not target_slot
        or "NOGO" in target_slot
        or "STEAM" in target_slot
        or not target_slot.startswith(marker)
    ):
        return escalate("The requested destination is not a safe pickup slot.")
    suffix = target_slot[len(marker):]
    if (
        not suffix
        or not all(character in "0123456789" for character in suffix)
        or not any(character != "0" for character in suffix)
    ):
        return escalate("The requested destination is not a safe pickup slot.")

    if order.get("special_request"):
        return escalate("This order requires manual preparation.")
    if order["status"] in ("completed", "delivered"):
        return {
            "status": "completed",
            "order_id": order_id,
            "already_completed": True,
        }
    if order["status"] in ("cancelled", "canceled", "rejected"):
        return escalate("The order is no longer eligible for fulfillment.")

    scene = sdk.observe_scene()
    for attempt in range(4):
        if not scene["control_ready"]:
            return {"status": "rejected_observation_not_ready"}
        if scene["coffee_machine"]["status"] not in ("busy", "cleaning"):
            break
        scene = sdk.observe_scene()

    if not scene["control_ready"]:
        return {"status": "rejected_observation_not_ready"}

    machine = scene["coffee_machine"]
    if machine["status"] == "fault":
        return escalate("The coffee machine reports a fault.")
    if machine["status"] not in ("idle", "running", "completed"):
        return escalate("The coffee machine is unavailable.")

    selection = select_cup(scene)
    if selection["cup"] is None:
        return pause(selection)
    cup_id = selection["cup"]["id"]

    resume_brew = (
        machine["status"] in ("running", "completed") or machine["cup_filled"]
    )

    if resume_brew:
        receipt = resolve_receipt(
            sdk.query_command_status(command_id("brew"))
        )
        if receipt not in ("running", "done", "completed"):
            return escalate("The existing brew cannot be confirmed for this command.")
        if (
            machine["status"] == "running"
            and not scene["l2_pouring_safety"]["safe_to_pour"]
        ):
            return escalate("Pouring safety is violated during the existing brew.")
    else:
        placement = sdk.place_cup_under_machine(
            cup_id, command_id=command_id("place")
        )
        if not done(placement):
            return escalate("Cup placement was not confirmed.")

        scene = sdk.observe_scene()
        if not scene["control_ready"]:
            return {"status": "rejected_observation_not_ready"}
        machine = scene["coffee_machine"]
        if machine["status"] != "idle" or machine["cup_filled"]:
            return escalate("Machine state changed unexpectedly after placement.")

        if not scene["l2_pouring_safety"]["safe_to_pour"]:
            recentering = sdk.recenter_cup_under_spout(
                cup_id, command_id=command_id("recenter")
            )
            if not done(recentering):
                return escalate("Cup recentering was not confirmed.")
            scene = sdk.observe_scene()

        if not scene["control_ready"]:
            return {"status": "rejected_observation_not_ready"}
        if not scene["l2_pouring_safety"]["safe_to_pour"]:
            return escalate("Pouring remains unsafe; brewing was not started.")
        machine = scene["coffee_machine"]
        if machine["status"] != "idle" or machine["cup_filled"]:
            return escalate("The machine is not ready for a new brew.")

        result = sdk.start_coffee_machine(command_id=command_id("brew"))
        receipt = resolve_receipt(result)
        if receipt not in ("running", "done", "completed"):
            return escalate("The brew command was rejected or its receipt is unresolved.")

    if not (
        scene["coffee_machine"]["status"] in ("idle", "completed")
        and scene["coffee_machine"]["cup_filled"]
    ):
        ready = sdk.wait_for_coffee_ready(command_id=command_id("wait"))
        if not done(ready):
            return escalate("Coffee readiness was not confirmed.")

    scene = sdk.observe_scene()
    if not scene["control_ready"]:
        return {"status": "rejected_observation_not_ready"}
    machine = scene["coffee_machine"]
    if machine["status"] not in ("idle", "completed") or not machine["cup_filled"]:
        return escalate("The machine did not confirm a finished, filled cup.")

    delivery = sdk.retrieve_and_deliver_cup(
        cup_id, target_slot, command_id=command_id("deliver")
    )
    if not done(delivery):
        return escalate("Physical delivery was not confirmed.")

    completion = sdk.complete_order_delivery(
        order_id, cup_id, target_slot, command_id=command_id("complete")
    )
    if not done(completion):
        return escalate("The cup was delivered, but order completion was not confirmed.")

    return {
        "status": "completed",
        "order_id": order_id,
        "cup_id": cup_id,
        "destination_slot": target_slot,
    }
