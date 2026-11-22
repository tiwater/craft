def run_policy(sdk, params):
    order_id = params.get("order_id", "A042")
    target_slot = params.get("target_slot", "$SLOT_PICKUP_1")
    command_prefix = str(params.get("command_prefix", "cap"))
    command_base = command_prefix + ":" + str(order_id)

    def command_id(step):
        return command_base + ":" + step

    def escalate(reason):
        retraction = sdk.retract_safe_pose(command_id=command_id("retract"))
        escalation = sdk.escalate_to_human_operator(
            order_id, reason, command_id=command_id("escalate")
        )
        return {
            "status": "escalated_to_human",
            "order_id": order_id,
            "reason": reason,
            "retraction_receipt": retraction.get("receipt"),
            "escalation_receipt": escalation.get("receipt"),
        }

    def rejected(step, receipt):
        return {
            "status": "rejected_" + step,
            "order_id": order_id,
            "receipt": receipt,
        }

    order = sdk.get_order(order_id)
    if order is None:
        return escalate("Order was not found.")

    # Only explicitly recognized automated recipes are authorized.
    automated_drinks = (
        "hot_americano",
        "takeout_americano",
        "iced_americano",
        "espresso",
    )
    if order.get("special_request"):
        return escalate("Special requests require human handling.")
    if order.get("drink") not in automated_drinks:
        return escalate("Recipe is not recognized as standard automated coffee.")

    slot = str(target_slot)
    slot_prefix = "$SLOT_PICKUP_"
    if "NOGO" in slot.upper() or "STEAM" in slot.upper():
        return escalate("Delivery target is a forbidden zone.")
    if not slot.startswith(slot_prefix):
        return escalate("Delivery target is not a valid pickup slot.")
    slot_number = slot[len(slot_prefix):]
    if (
        not slot_number
        or not all(character in "0123456789" for character in slot_number)
        or not any(character != "0" for character in slot_number)
    ):
        return escalate("Delivery target is not a valid pickup slot.")

    scene = sdk.observe_scene()
    if scene.get("control_ready") != True:
        return {"status": "rejected_observation_not_ready"}

    threshold = scene["confidence_threshold"]
    candidates = [
        obj
        for obj in scene["objects"]
        if obj["cup_type"] != "flexible_bag"
        and obj["confidence"] >= threshold
    ]
    confirmed_id = scene.get("operator_confirmed_cup_id")
    if confirmed_id is not None:
        choices = [
            obj for obj in candidates if obj["id"] == confirmed_id
        ]
    else:
        choices = candidates

    if len(choices) != 1:
        confirmation = sdk.request_operator_confirmation(
            "A single eligible rigid cup could not be uniquely identified.",
            candidates,
        )
        return {
            "status": "paused_for_operator",
            "order_id": order_id,
            "confirmation": confirmation,
        }

    cup_id = choices[0]["id"]
    placement = sdk.place_cup_under_machine(
        cup_id, command_id=command_id("place")
    )
    if placement.get("receipt") != "done":
        return rejected("placement", placement.get("receipt"))

    scene = sdk.observe_scene()
    if scene.get("control_ready") != True:
        return {"status": "rejected_observation_not_ready"}

    if scene.get("l2_pouring_safety", {}).get("safe_to_pour") != True:
        recentering = sdk.recenter_cup_under_spout(
            cup_id, command_id=command_id("recenter")
        )
        if recentering.get("receipt") != "done":
            return rejected("recentering", recentering.get("receipt"))

        scene = sdk.observe_scene()
        if scene.get("control_ready") != True:
            return {"status": "rejected_observation_not_ready"}
        if scene.get("l2_pouring_safety", {}).get("safe_to_pour") != True:
            return escalate("Pouring safety remains unsatisfied after recentering.")

    brew_id = command_id("brew")
    brew = sdk.start_coffee_machine(command_id=brew_id)
    brew_receipt = brew.get("receipt")

    # Reconcile uncertainty without ever issuing another start command.
    if brew_receipt == "unknown":
        for attempt in range(3):
            brew = sdk.query_command_status(brew_id)
            brew_receipt = brew.get("receipt")
            if brew_receipt != "unknown":
                break

    if brew_receipt == "unknown":
        return escalate("Brew command outcome remains unknown: " + brew_id)
    if brew_receipt not in ("running", "done", "completed"):
        return rejected("brew", brew_receipt)

    ready = sdk.wait_for_coffee_ready(command_id=command_id("wait"))
    if ready.get("receipt") != "done":
        return rejected("brew_completion", ready.get("receipt"))

    delivery = sdk.retrieve_and_deliver_cup(
        cup_id, target_slot, command_id=command_id("deliver")
    )
    if delivery.get("receipt") != "done":
        return rejected("delivery", delivery.get("receipt"))

    completion = sdk.complete_order_delivery(
        order_id,
        cup_id,
        target_slot,
        command_id=command_id("complete"),
    )
    if completion.get("receipt") != "done":
        return rejected("finalization", completion.get("receipt"))

    return {
        "status": "completed",
        "order_id": order_id,
        "cup_id": cup_id,
        "destination_slot": target_slot,
    }
