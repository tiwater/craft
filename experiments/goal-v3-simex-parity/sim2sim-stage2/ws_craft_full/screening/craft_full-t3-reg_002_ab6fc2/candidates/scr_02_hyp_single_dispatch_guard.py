def run_policy(sdk, params):
    order_id = params.get("order_id")
    prefix = params.get("command_prefix") or str(order_id)
    brew_id = prefix + ":brew"

    def escalate(reason):
        sdk.retract_safe_pose(command_id=prefix + ":retract")
        sdk.escalate_to_human_operator(
            order_id, reason, command_id=prefix + ":escalate"
        )
        return {"status": "escalated_to_human", "reason": reason}

    def observation_not_ready():
        sdk.retract_safe_pose(command_id=prefix + ":observation-retract")
        return {"status": "rejected_observation_not_ready"}

    def reconcile_brew():
        receipt = "unknown"
        for attempt in range(3):
            receipt = sdk.query_command_status(brew_id).get("receipt")
            if receipt in ("running", "done", "completed", "rejected"):
                return receipt
        return receipt

    order = sdk.get_order(order_id)
    if not order:
        return escalate("Order not found")
    if order.get("special_request"):
        return escalate("Order requires manual assistance")

    target_slot = params.get("target_slot") or order["destination_slot"]
    if not target_slot or "NOGO" in target_slot or "STEAM" in target_slot:
        return escalate("Forbidden or missing destination")
    slot_prefix = "$SLOT_PICKUP_"
    if not target_slot.startswith(slot_prefix):
        return escalate("Destination is not a pickup slot")
    slot_number = target_slot[len(slot_prefix):]
    if not slot_number or not slot_number.isdecimal():
        return escalate("Invalid pickup slot")
    if int(slot_number) < 1:
        return escalate("Invalid pickup slot")

    scene = sdk.observe_scene()
    machine = scene["coffee_machine"]
    resume_brew = (
        machine["status"] in ("running", "completed")
        or machine["cup_filled"]
    )
    if resume_brew:
        receipt = reconcile_brew()
        if receipt not in ("running", "done", "completed"):
            return escalate("Previous brew remains unconfirmed")

    if not scene["control_ready"]:
        return observation_not_ready()

    if resume_brew:
        if machine["status"] not in ("idle", "running", "completed"):
            return escalate("Machine is not ready to resume the confirmed brew")
    elif machine["status"] != "idle":
        return escalate("Machine is not ready for a new single-cup brew")

    candidates = [
        obj for obj in scene["objects"]
        if obj["cup_type"] != "flexible_bag"
        and obj["confidence"] >= scene["confidence_threshold"]
    ]
    confirmed_id = scene.get("operator_confirmed_cup_id")
    selected = []
    if confirmed_id:
        selected = [obj for obj in candidates if obj["id"] == confirmed_id]
    elif len(candidates) == 1:
        selected = candidates

    if len(selected) != 1:
        return {"status": "paused_for_operator"}

    cup_id = selected[0]["id"]

    if not resume_brew:
        placement = sdk.place_cup_under_machine(
            cup_id, command_id=prefix + ":place"
        )
        if placement.get("receipt") != "done":
            return escalate("Cup placement was not confirmed")

        scene = sdk.observe_scene()
        if not scene["control_ready"]:
            return observation_not_ready()

        machine = scene["coffee_machine"]
        if machine["status"] != "idle" or machine["cup_filled"]:
            reconcile_brew()
            return escalate("Machine state changed before brewing")

        if not scene["l2_pouring_safety"]["safe_to_pour"]:
            recenter = sdk.recenter_cup_under_spout(
                cup_id, command_id=prefix + ":recenter"
            )
            if recenter.get("receipt") != "done":
                return escalate("Cup recentering was not confirmed")

            scene = sdk.observe_scene()
            if not scene["control_ready"]:
                return observation_not_ready()

            machine = scene["coffee_machine"]
            if machine["status"] != "idle" or machine["cup_filled"]:
                reconcile_brew()
                return escalate("Machine state changed before brewing")

            if not scene["l2_pouring_safety"]["safe_to_pour"]:
                return escalate("Pouring safety check failed")

        brew = sdk.start_coffee_machine(command_id=brew_id)
        receipt = brew.get("receipt")
        if receipt not in ("running", "done", "completed", "rejected"):
            receipt = reconcile_brew()
        if receipt not in ("running", "done", "completed"):
            return escalate("Brew receipt was rejected or remains unresolved")

    ready = sdk.wait_for_coffee_ready(command_id=prefix + ":wait")
    if ready.get("receipt") != "done":
        return escalate("Coffee readiness was not confirmed")

    scene = sdk.observe_scene()
    if not scene["control_ready"]:
        return observation_not_ready()
    machine = scene["coffee_machine"]
    if machine["status"] not in ("idle", "completed") or not machine["cup_filled"]:
        return escalate("Finished coffee could not be verified")

    delivery = sdk.retrieve_and_deliver_cup(
        cup_id, target_slot, command_id=prefix + ":deliver"
    )
    if delivery.get("receipt") != "done":
        return escalate("Cup delivery was not confirmed")

    completion = sdk.complete_order_delivery(
        order_id, cup_id, target_slot, command_id=prefix + ":complete"
    )
    if completion.get("receipt") != "done":
        return escalate("Order completion was not confirmed")

    return {"status": "completed"}
