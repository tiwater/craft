def policy_move(sdk, position, yaw, gripper, limit, tolerance=0.012):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        position,
        yaw=yaw,
        gripper=gripper,
        max_steps=min(limit, remaining),
        tol=tolerance,
    )
    return result["reached"]


def policy_gripper(sdk, action, steps):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    sdk.set_gripper(action, steps=min(steps, remaining))
    return True


def run_policy(sdk):
    state = sdk.get_state()
    objects = state["objects"]
    sites = state["sites"]
    cabinets = [
        name for name in state["fixtures"] if "wooden_cabinet" in name
    ]
    bowls = [
        name for name in objects if "black_bowl" in name
    ]
    plates = [
        name for name in objects if "plate" in name
    ]
    if not cabinets or not bowls or not plates:
        return {"status": "failed", "reason": "Required scene entities missing"}

    cabinet = cabinets[0]
    anchor = state["fixtures"][cabinet]["pos"]
    if cabinet + "_top_region" in sites:
        anchor = sites[cabinet + "_top_region"]

    bowl = bowls[0]
    best_score = float("inf")
    for name in bowls:
        position = objects[name]["pos"]
        score = (
            (position[0] - anchor[0]) ** 2
            + (position[1] - anchor[1]) ** 2
            + 0.5 * (position[2] - anchor[2]) ** 2
        )
        if score < best_score:
            best_score = score
            bowl = name

    plate = plates[0]
    initial_bowl = list(objects[bowl]["pos"])
    bottom_offset = 0.035
    if cabinet + "_top_region" in sites:
        estimate = initial_bowl[2] - sites[cabinet + "_top_region"][2]
        if 0.015 <= estimate <= 0.065:
            bottom_offset = estimate

    clearance = max(state["eef_pos"][2], initial_bowl[2] + 0.15)
    for name in objects:
        clearance = max(clearance, objects[name]["pos"][2] + 0.12)

    held = False
    yaw = state["eef_yaw"]
    for attempt in range(3):
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining < 155:
            break

        position = list(state["objects"][bowl]["pos"])
        yaw = state["objects"][bowl]["yaw"] + attempt * pi / 2.0
        policy_gripper(sdk, "open", 8)

        eef = sdk.get_state()["eef_pos"]
        if not policy_move(
            sdk, [eef[0], eef[1], clearance], yaw, "open", 35
        ):
            continue
        if not policy_move(
            sdk, [position[0], position[1], clearance], yaw, "open", 40
        ):
            continue

        position = list(sdk.get_state()["objects"][bowl]["pos"])
        heights = [0.015, 0.0, 0.03]
        grasp_position = [
            position[0],
            position[1],
            position[2] + heights[attempt],
        ]
        if not policy_move(sdk, grasp_position, yaw, "open", 35, 0.008):
            continue

        before = list(sdk.get_state()["objects"][bowl]["pos"])
        policy_gripper(sdk, "close", 14)
        closed = sdk.get_state()
        eef_before = list(closed["eef_pos"])

        policy_move(
            sdk,
            [eef_before[0], eef_before[1], eef_before[2] + 0.10],
            yaw,
            "close",
            30,
            0.01,
        )
        lifted = sdk.get_state()
        after = lifted["objects"][bowl]["pos"]
        eef_after = lifted["eef_pos"]
        bowl_rise = after[2] - before[2]
        hand_rise = eef_after[2] - eef_before[2]
        if (
            bowl_rise > 0.045
            and abs(bowl_rise - hand_rise) < 0.035
            and hypot(after[0] - eef_after[0], after[1] - eef_after[1]) < 0.085
        ):
            held = True
            break

    if not held:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    bowl_position = state["objects"][bowl]["pos"]
    eef = state["eef_pos"]
    offset = [eef[i] - bowl_position[i] for i in range(3)]
    travel_height = max(clearance, eef[2], bowl_position[2] + 0.10 + offset[2])

    if not policy_move(
        sdk, [eef[0], eef[1], travel_height], yaw, "close", 35
    ):
        return {"status": "failed", "reason": "Could not clear cabinet"}

    target = sdk.get_state()["objects"][plate]["pos"]
    if not policy_move(
        sdk,
        [target[0] + offset[0], target[1] + offset[1], travel_height],
        yaw,
        "close",
        55,
    ):
        return {"status": "failed", "reason": "Could not reach plate"}

    state = sdk.get_state()
    current_bowl = state["objects"][bowl]["pos"]
    current_eef = state["eef_pos"]
    if abs((current_eef[2] - current_bowl[2]) - offset[2]) > 0.045:
        return {"status": "failed", "reason": "Bowl slipped during transport"}

    target = state["objects"][plate]["pos"]
    placement = [
        target[0] + offset[0],
        target[1] + offset[1],
        target[2] + bottom_offset + 0.012 + offset[2],
    ]
    if not policy_move(sdk, placement, yaw, "close", 50, 0.008):
        return {"status": "failed", "reason": "Could not reach placement height"}

    if not policy_gripper(sdk, "open", 16):
        return {"status": "failed", "reason": "Step budget exhausted before release"}

    eef = sdk.get_state()["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.12], yaw, "open", 25
    )
    policy_gripper(sdk, "open", 10)

    state = sdk.get_state()
    final_bowl = state["objects"][bowl]["pos"]
    final_plate = state["objects"][plate]["pos"]
    horizontal_error = hypot(
        final_bowl[0] - final_plate[0],
        final_bowl[1] - final_plate[1],
    )
    height_difference = final_bowl[2] - final_plate[2]
    placed = (
        horizontal_error < 0.055
        and 0.012 < height_difference < bottom_offset + 0.035
    )
    return {
        "status": "done" if placed else "failed",
        "object": bowl,
        "target": plate,
        "placement_verified_by_pose": placed,
        "horizontal_error_m": horizontal_error,
        "steps_used": state["steps_used"],
    }
