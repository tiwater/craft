def run_policy(sdk, params):
    order_id = params.get("order_id") or "A042"
    prefix = params.get("command_prefix") or "cap"
    order = sdk.get_order(order_id)
    target_slot = (
        params.get("target_slot")
        or order.get("destination_slot")
        or "$SLOT_PICKUP_1"
    )

    if target_slot == "$ZONE_STEAM_NOGO" or order.get("special_request") in (
        "steam_wand_manual_purge_and_latte_art",
        "open_sealed_bean_bag",
    ):
        sdk.retract_safe_pose(command_id=prefix + "-retract")
        sdk.escalate_to_human_operator(
            order_id,
            "hazard_or_out_of_scope",
            command_id=prefix + "-escalate",
        )
        return {"status": "escalated_to_human", "order_id": order_id}

    obs = sdk.observe_scene()
    if not obs.get("control_ready", False):
        return {"status": "rejected_observation_not_ready"}

    vessels = [
        obj
        for obj in obs.get("objects", [])
        if obj.get("cup_type") != "flexible_bag"
    ]
    confirmed_id = obs.get("operator_confirmed_cup_id")
    confirmed = [
        vessel
        for vessel in vessels
        if confirmed_id is not None and vessel.get("id") == confirmed_id
    ]
    confident = [
        vessel
        for vessel in vessels
        if vessel.get("confidence", 0) >= obs.get("confidence_threshold", 0.75)
    ]

    if len(confirmed) == 1:
        cup = confirmed[0]
    elif len(confident) == 1:
        cup = confident[0]
    else:
        sdk.request_operator_confirmation(
            "ambiguous_or_low_confidence_cup", vessels
        )
        return {"status": "paused_for_operator", "order_id": order_id}

    cup_id = cup["id"]
    receipt = sdk.place_cup_under_machine(
        cup_id, command_id=prefix + "-place"
    )
    if receipt.get("receipt") != "done":
        return {"status": "failed_place"}

    obs_after = sdk.observe_scene()
    if not obs_after.get("l2_pouring_safety", {}).get("safe_to_pour", False):
        receipt = sdk.recenter_cup_under_spout(
            cup_id, command_id=prefix + "-recenter"
        )
        if receipt.get("receipt") != "done":
            return {"status": "failed_recenter"}
        obs_after = sdk.observe_scene()
        if not obs_after.get("l2_pouring_safety", {}).get("safe_to_pour", False):
            return {"status": "failed_l2_pouring_gate"}

    brew_cmd = prefix + "-brew"
    r = sdk.start_coffee_machine(command_id=brew_cmd)
    if r.get("receipt") == "unknown":
        r = sdk.query_command_status(brew_cmd)
    if r.get("receipt") not in ("running", "done"):
        return {"status": "failed_brew_start"}

    receipt = sdk.wait_for_coffee_ready(command_id=prefix + "-wait")
    if receipt.get("receipt") != "done":
        return {"status": "failed_wait"}

    receipt = sdk.retrieve_and_deliver_cup(
        cup_id, target_slot, command_id=prefix + "-deliver"
    )
    if receipt.get("receipt") != "done":
        return {"status": "failed_deliver"}

    receipt = sdk.complete_order_delivery(
        order_id, cup_id, target_slot, command_id=prefix + "-verify"
    )
    if receipt.get("receipt") != "done":
        return {"status": "failed_verify"}

    return {
        "status": "completed",
        "order_id": order_id,
        "cup_id": cup_id,
        "target_slot": target_slot,
    }
