def run_policy(sdk, params):
    operator_instruction = "制作一杯堂食热美式，送到1号取餐口。"
    instruction_text = str(params.get("instruction", ""))
    instructions = (operator_instruction + " " + instruction_text).lower()
    takeout = any(term in instructions for term in (
        "外卖", "takeout", "2号取餐位", "2号取餐口", "a043", "$slot_pickup_2"
    ))
    default_order_id = "A043" if takeout else "A042"
    default_slot = "$SLOT_PICKUP_2" if takeout else "$SLOT_PICKUP_1"
    order_id = params.get("order_id") or default_order_id
    prefix = params.get("command_prefix", "cap")
    order = sdk.get_order(order_id)
    obs = sdk.observe_scene()

    if not obs.get("control_ready", False):
        return {
            "status": "rejected_observation_not_ready",
            "order_id": order_id,
            "pipeline_health": obs.get("pipeline_health")
        }

    target_slot = params.get("target_slot") or order.get("destination_slot") or default_slot
    special_req = order.get("special_request")
    hazard = any(term in instructions for term in (
        "蒸汽", "奶泡", "拉花", "steam", "拆开", "拆包",
        "open_sealed_bean_bag", "$zone_steam_nogo",
        "milk frothing", "froth milk", "latte art"
    ))
    if (
        hazard
        or special_req in (
            "steam_wand_manual_purge_and_latte_art",
            "open_sealed_bean_bag"
        )
        or target_slot == "$ZONE_STEAM_NOGO"
    ):
        ret = sdk.retract_safe_pose(command_id=prefix + "-retract-safe")
        esc = sdk.escalate_to_human_operator(
            order_id,
            "out_of_scope_or_hazard: " + str(special_req or target_slot or instruction_text),
            command_id=prefix + "-escalate"
        )
        return {
            "status": "escalated_to_human",
            "order_id": order_id,
            "retract_receipt": ret["receipt"],
            "escalation": esc
        }

    objects = [o for o in obs["objects"] if o.get("cup_type") != "flexible_bag"]
    if len(objects) == 0:
        return {
            "status": "failed",
            "reason": "no_cup_detected",
            "order_id": order_id
        }

    confirmed_id = obs.get("operator_confirmed_cup_id")
    threshold = obs["confidence_threshold"]
    confirmed_objects = [o for o in objects if o["id"] == confirmed_id]
    if len(confirmed_objects) > 0:
        selected_cup = confirmed_objects[0]
    elif len(objects) == 1 and objects[0]["confidence"] >= threshold:
        selected_cup = objects[0]
    else:
        req = sdk.request_operator_confirmation(
            "ambiguous_or_low_confidence_cup", objects
        )
        return {
            "status": "paused_for_operator",
            "order_id": order_id,
            "target_slot": target_slot,
            "confirmation_request": req
        }

    cup_id = selected_cup["id"]
    cup_xyz = selected_cup["world"]
    already_at_spout = (
        abs(cup_xyz[0] - 0.45) < 0.09
        and abs(cup_xyz[1] - 0.0) < 0.09
    )
    if not already_at_spout:
        r_place = sdk.place_cup_under_machine(
            cup_id, command_id=prefix + "-place-cup"
        )
        if r_place["receipt"] != "done":
            return {
                "status": "failed_place",
                "order_id": order_id,
                "step_receipt": r_place
            }
        obs_after_place = sdk.observe_scene()
    else:
        r_place = {"receipt": "done"}
        obs_after_place = obs

    recenter_receipt = "skipped"
    if not obs_after_place["l2_pouring_safety"].get("safe_to_pour", False):
        r_recenter = sdk.recenter_cup_under_spout(
            cup_id, command_id=prefix + "-recenter-cup"
        )
        recenter_receipt = r_recenter["receipt"]
        if recenter_receipt != "done":
            return {
                "status": "failed_recenter",
                "order_id": order_id,
                "step_receipt": r_recenter
            }
        obs_after_place = sdk.observe_scene()

    if not obs_after_place["l2_pouring_safety"].get("safe_to_pour", False):
        return {
            "status": "failed_l2_pouring_gate",
            "order_id": order_id,
            "recenter_receipt": recenter_receipt,
            "l2_pouring_safety": obs_after_place["l2_pouring_safety"]
        }

    brew_cmd_id = prefix + "-start-brew"
    r_brew = sdk.start_coffee_machine(command_id=brew_cmd_id)
    if r_brew["receipt"] == "unknown":
        r_brew = sdk.query_command_status(brew_cmd_id)
    if r_brew["receipt"] not in ("running", "done"):
        return {
            "status": "failed_brew_start",
            "order_id": order_id,
            "step_receipt": r_brew
        }

    r_wait = sdk.wait_for_coffee_ready(command_id=prefix + "-wait-brew")
    if r_wait["receipt"] != "done":
        return {
            "status": "failed_brew_wait",
            "order_id": order_id,
            "step_receipt": r_wait
        }

    r_deliver = sdk.retrieve_and_deliver_cup(
        cup_id, target_slot, command_id=prefix + "-deliver-cup"
    )
    if r_deliver["receipt"] != "done":
        return {
            "status": "failed_deliver",
            "order_id": order_id,
            "step_receipt": r_deliver
        }

    r_verify = sdk.complete_order_delivery(
        order_id, cup_id, target_slot, command_id=prefix + "-verify-order"
    )
    if r_verify["receipt"] != "done":
        return {
            "status": "failed_order_verification",
            "order_id": order_id,
            "step_receipt": r_verify
        }

    return {
        "status": "completed",
        "order_id": order_id,
        "selected_cup_id": cup_id,
        "target_slot": target_slot,
        "place_receipt": r_place["receipt"],
        "recenter_receipt": recenter_receipt,
        "brew_receipt": r_brew["receipt"],
        "wait_receipt": r_wait["receipt"],
        "deliver_receipt": r_deliver["receipt"],
        "verify_receipt": r_verify["receipt"],
        "order_verification": r_verify
    }