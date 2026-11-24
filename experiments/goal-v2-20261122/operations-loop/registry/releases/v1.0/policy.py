def run_policy(sdk, params):
    order_id = params.get("order_id")
    prefix = params.get("command_prefix")
    if not order_id or not prefix:
        return {"status": "rejected_invalid_parameters"}

    prefix = str(prefix)

    def command(step):
        return prefix + ":" + step

    def receipt(result):
        return result.get("receipt", "unknown")

    def escalate(reason):
        sdk.retract_safe_pose(command_id=command("retract"))
        sdk.escalate_to_human_operator(
            order_id, reason, command_id=command("escalate")
        )
        return {"status": "escalated_to_human", "reason": reason}

    def observation_ready(scene):
        if scene.get("control_ready") is not True:
            return False

        health = scene.get("pipeline_health")
        if health is None or health is False:
            return False

        positive_flags = (
            "ready", "healthy", "ok", "available", "connected",
            "control_ready", "perception_ready", "pipeline_ready",
        )
        negative_flags = (
            "fault", "failed", "failure", "error", "unhealthy",
            "degraded", "unavailable", "not_ready", "stale",
            "disconnected", "offline", "has_error", "has_fault",
        )
        bad_states = (
            "fault", "failed", "failure", "error", "unhealthy",
            "degraded", "unavailable", "not_ready", "not ready",
            "stale", "disconnected", "offline",
        )

        for key, value in health.items():
            name = str(key).lower()
            state = str(value).lower()
            if state in bad_states:
                return False

            positive = (
                name in positive_flags
                or name.endswith("_ready")
                or name.endswith("_healthy")
                or name.endswith("_ok")
                or name.endswith("_connected")
            )
            negative = (
                name in negative_flags
                or name.endswith("_stale")
                or name.endswith("_failed")
                or name.endswith("_fault")
                or name.endswith("_error")
            )
            if negative:
                if value is True or state in ("true", "1"):
                    return False
            elif positive:
                if value is False or value is None or state in ("false", "0"):
                    return False

        return True

    def observation_rejected(retract):
        if retract:
            sdk.retract_safe_pose(command_id=command("retract"))
        return {"status": "rejected_observation_not_ready"}

    def valid_slot(slot):
        if not slot or not slot.startswith("$SLOT_PICKUP_"):
            return False
        suffix = slot[len("$SLOT_PICKUP_"):]
        return (
            bool(suffix)
            and all(character in "0123456789" for character in suffix)
            and any(character != "0" for character in suffix)
        )

    def cups(scene):
        threshold = scene.get("confidence_threshold")
        if threshold is None:
            return []
        return [
            item
            for item in scene.get("objects", [])
            if item.get("id")
            and item.get("cup_type") != "flexible_bag"
            and item.get("confidence", -1.0) >= threshold
        ]

    order = sdk.get_order(order_id)
    if not order:
        return escalate("Order was not found.")

    if order.get("special_request"):
        return escalate("The order requires manual preparation.")

    target_slot = params.get("target_slot") or order.get("destination_slot")
    if (
        not target_slot
        or "NOGO" in target_slot.upper()
        or "STEAM" in target_slot.upper()
        or not valid_slot(target_slot)
    ):
        return escalate("The requested destination is not a valid pickup slot.")

    scene = sdk.observe_scene()
    if not observation_ready(scene):
        return observation_rejected(False)

    if scene.get("coffee_machine", {}).get("status") == "fault":
        return escalate("The coffee machine reports a fault.")

    if scene.get("confidence_threshold") is None:
        return observation_rejected(False)

    candidates = cups(scene)
    confirmed_id = scene.get("operator_confirmed_cup_id")
    confirmed = [item for item in candidates if item["id"] == confirmed_id]

    if confirmed_id and len(confirmed) == 1:
        cup_id = confirmed[0]["id"]
    elif not confirmed_id and len(candidates) == 1:
        cup_id = candidates[0]["id"]
    else:
        sdk.request_operator_confirmation(
            "Confirm a confidently observed rigid cup before preparation.",
            candidates,
        )
        return {"status": "paused_for_operator"}

    start_id = command("start")
    previous = receipt(sdk.query_command_status(start_id))

    if previous == "rejected":
        return escalate("The brew command was rejected.")

    brewing = previous in ("running", "done", "completed")

    if not brewing:
        if previous != "unknown":
            return escalate("The previous brew receipt is unresolved.")

        for attempt in range(8):
            machine = scene.get("coffee_machine", {})
            if machine.get("status") not in ("busy", "cleaning"):
                break
            scene = sdk.observe_scene()
            if not observation_ready(scene):
                return observation_rejected(False)

        machine = scene.get("coffee_machine", {})
        if machine.get("status") == "fault":
            return escalate("The coffee machine reports a fault.")

        if machine.get("status") != "idle" or machine.get("cup_filled") is True:
            return escalate(
                "The machine is not empty and idle; brew ownership is unresolved."
            )

        candidates = cups(scene)
        if len([item for item in candidates if item["id"] == cup_id]) != 1:
            sdk.request_operator_confirmation(
                "The selected cup is no longer confidently observed.",
                candidates,
            )
            return {"status": "paused_for_operator"}

        # An absent or uncentered cup can make pouring unsafe before placement.
        # Placement and recentering prepare the cup; only brewing requires
        # an affirmative pouring-safety observation.
        placed = sdk.place_cup_under_machine(
            cup_id, command_id=command("place")
        )
        if receipt(placed) != "done":
            return escalate("Cup placement was not confirmed.")

        scene = sdk.observe_scene()
        if not observation_ready(scene):
            return observation_rejected(True)

        machine = scene.get("coffee_machine", {})
        if machine.get("status") != "idle" or machine.get("cup_filled") is True:
            return escalate("The machine is not empty and idle after placement.")

        if scene.get("l2_pouring_safety", {}).get("safe_to_pour") is not True:
            recentered = sdk.recenter_cup_under_spout(
                cup_id, command_id=command("recenter")
            )
            if receipt(recentered) != "done":
                return escalate("Cup recentering was not confirmed.")

            scene = sdk.observe_scene()
            if not observation_ready(scene):
                return observation_rejected(True)

            machine = scene.get("coffee_machine", {})
            if (
                machine.get("status") != "idle"
                or machine.get("cup_filled") is True
            ):
                return escalate("The machine state changed during recentering.")

            if scene.get("l2_pouring_safety", {}).get("safe_to_pour") is not True:
                return escalate("Pouring remains unsafe after recentering.")

        brew_receipt = receipt(
            sdk.start_coffee_machine(command_id=start_id)
        )

        if brew_receipt == "rejected":
            return escalate("The brew command was rejected.")

        if brew_receipt not in ("running", "done", "completed", "unknown"):
            return escalate("The brew receipt is unresolved; do not retry brewing.")

        queried_receipt = "unknown"
        for attempt in range(4):
            queried_receipt = receipt(sdk.query_command_status(start_id))
            if queried_receipt != "unknown":
                break

        if queried_receipt not in ("running", "done", "completed"):
            return escalate(
                "The brew status is rejected or unresolved; do not retry brewing."
            )

        if (
            brew_receipt in ("done", "completed")
            and queried_receipt == "running"
        ):
            return escalate("The brew acknowledgement contradicts its status.")

    ready = sdk.wait_for_coffee_ready(command_id=command("wait"))
    if receipt(ready) != "done":
        return escalate("Coffee completion was not confirmed.")

    scene = sdk.observe_scene()
    if not observation_ready(scene):
        return observation_rejected(True)

    machine = scene.get("coffee_machine", {})
    if machine.get("status") == "fault":
        return escalate("The coffee machine reports a fault after brewing.")

    if (
        machine.get("status") not in ("idle", "completed")
        or machine.get("cup_filled") is not True
    ):
        return escalate("The machine has not confirmed a finished, filled cup.")

    delivered = sdk.retrieve_and_deliver_cup(
        cup_id, target_slot, command_id=command("deliver")
    )
    if receipt(delivered) != "done":
        return escalate("Cup delivery was not confirmed.")

    completed = sdk.complete_order_delivery(
        order_id, cup_id, target_slot, command_id=command("complete")
    )
    if receipt(completed) != "done":
        return escalate("Delivery occurred, but order completion was not confirmed.")

    return {
        "status": "completed",
        "order_id": order_id,
        "cup_id": cup_id,
        "destination_slot": target_slot,
    }
