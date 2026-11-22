def run_policy(sdk, params):
    order_id = params.get("order_id", "A042")
    target_slot = params.get("target_slot", "$SLOT_PICKUP_1")
    prefix = str(params.get("command_prefix", "cap")) + ":" + str(order_id)

    def escalate(reason):
        sdk.retract_safe_pose(command_id=prefix + ":retract")
        sdk.escalate_to_human_operator(
            order_id, reason, command_id=prefix + ":escalate"
        )
        return {
            "status": "escalated_to_human",
            "order_id": order_id,
            "reason": reason
        }

    def eligible_cups(scene):
        return [
            obj for obj in scene["objects"]
            if obj["cup_type"] != "flexible_bag"
            and obj["confidence"] >= scene["confidence_threshold"]
        ]

    order = sdk.get_order(order_id)
    if order is None:
        return escalate("Order not found")

    stem = "$SLOT_PICKUP_"
    slot = str(target_slot)
    suffix = slot[len(stem):]
    valid_slot = (
        slot[:len(stem)] == stem
        and bool(suffix)
        and suffix[0] in "123456789"
        and all(character in "0123456789" for character in suffix)
    )
    if not valid_slot:
        return escalate("Delivery target is not a valid pickup slot")
    if order.get("special_request"):
        return escalate("Special requests require human handling")
    if order.get("drink") not in ("hot_americano", "takeout_americano"):
        return escalate("Unsupported automated recipe")

    scene = sdk.observe_scene()
    if scene.get("control_ready") != True:
        return {"status": "rejected_observation_not_ready"}

    candidates = eligible_cups(scene)
    confirmed = scene.get("operator_confirmed_cup_id")
    matches = candidates
    if confirmed is not None:
        matches = [obj for obj in candidates if obj["id"] == confirmed]

    if len(matches) != 1:
        confirmation = sdk.request_operator_confirmation(
            "Confirm one eligible rigid cup before movement or brewing",
            candidates
        )
        return {
            "status": "paused_for_operator",
            "order_id": order_id,
            "confirmation": confirmation
        }

    cup_id = matches[0]["id"]
    placement = sdk.place_cup_under_machine(
        cup_id, command_id=prefix + ":place"
    )
    if placement.get("receipt") != "done":
        return escalate("Cup placement failed")

    scene = sdk.observe_scene()
    if scene.get("control_ready") != True:
        return {"status": "rejected_observation_not_ready"}

    if scene.get("l2_pouring_safety", {}).get("safe_to_pour") != True:
        recenter = sdk.recenter_cup_under_spout(
            cup_id, command_id=prefix + ":recenter"
        )
        if recenter.get("receipt") != "done":
            return escalate("Cup recentering failed")
        scene = sdk.observe_scene()
        if scene.get("control_ready") != True:
            return {"status": "rejected_observation_not_ready"}
        if scene.get("l2_pouring_safety", {}).get("safe_to_pour") != True:
            return escalate("Pouring safety remains unsatisfied")

    brew_command = prefix + ":brew"
    brew = sdk.start_coffee_machine(command_id=brew_command)
    if brew.get("receipt") == "unknown":
        brew = sdk.query_command_status(brew_command)
    if brew.get("receipt") not in ("running", "done", "completed"):
        return {
            "status": "brew_rejected" if brew.get("receipt") == "rejected" else "brew_status_unresolved",
            "order_id": order_id,
            "command_id": brew_command
        }

    ready = sdk.wait_for_coffee_ready(command_id=prefix + ":wait")
    if ready.get("receipt") != "done":
        return {
            "status": "brew_completion_unconfirmed",
            "order_id": order_id
        }

    delivered = False
    for attempt in range(2):
        scene = sdk.observe_scene()
        if scene.get("control_ready") != True:
            return {"status": "rejected_observation_not_ready"}

        candidates = eligible_cups(scene)
        matches = [obj for obj in candidates if obj["id"] == cup_id]
        confirmed = scene.get("operator_confirmed_cup_id")
        if len(matches) != 1 or (
            confirmed is not None and confirmed != cup_id
        ):
            confirmation = sdk.request_operator_confirmation(
                "Reidentify the brewed cup before retrieval; do not substitute another cup",
                candidates
            )
            return {
                "status": "paused_for_operator",
                "order_id": order_id,
                "cup_id": cup_id,
                "confirmation": confirmation
            }

        delivery_command = prefix + ":deliver:" + str(attempt)
        delivery = sdk.retrieve_and_deliver_cup(
            cup_id, target_slot, command_id=delivery_command
        )
        receipt = delivery.get("receipt")
        if receipt != "done":
            delivery = sdk.query_command_status(delivery_command)
            receipt = delivery.get("receipt")

        if receipt in ("done", "completed"):
            delivered = True
            break

        if receipt != "rejected":
            return {
                "status": "delivery_status_unresolved",
                "order_id": order_id,
                "cup_id": cup_id,
                "command_id": delivery_command
            }

        if attempt == 0:
            recovery = sdk.retract_safe_pose(
                command_id=prefix + ":delivery_recovery"
            )
            if recovery.get("receipt") != "done":
                return escalate("Unable to retract after rejected retrieval")

    if not delivered:
        return escalate("Brewed cup retrieval failed after refreshed observation and retry")

    completion = sdk.complete_order_delivery(
        order_id, cup_id, target_slot, command_id=prefix + ":complete"
    )
    if completion.get("receipt") != "done":
        return {
            "status": "delivery_finalization_failed",
            "order_id": order_id,
            "cup_id": cup_id
        }

    return {
        "status": "completed",
        "order_id": order_id,
        "cup_id": cup_id,
        "destination_slot": target_slot
    }
