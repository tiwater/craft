def policy_move(sdk, pos, yaw, gripper, limit=40, tol=0.008):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        pos,
        yaw=yaw,
        gripper=gripper,
        max_steps=min(limit, remaining),
        tol=tol,
    )
    return result["reached"]


def policy_gripper(sdk, action, steps=12):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    sdk.set_gripper(action, steps=min(steps, remaining))
    return True


def run_policy(sdk):
    state = sdk.get_state()
    objects = state["objects"]
    bowls = [name for name in objects if "black_bowl" in name]
    plates = [name for name in objects if "plate" in name]
    if not bowls or not plates:
        return {"status": "failed", "reason": "Required objects unavailable"}

    stove_points = []
    for name in state["sites"]:
        if "stove" in name and ("burner" in name or "cook_region" in name):
            stove_points.append(state["sites"][name])
    if not stove_points:
        for name in state["fixtures"]:
            if "stove" in name:
                stove_points.append(state["fixtures"][name]["pos"])
    if not stove_points:
        return {"status": "failed", "reason": "Stove pose unavailable"}

    bowl_name = bowls[0]
    best_score = 1000000.0
    for name in bowls:
        pos = objects[name]["pos"]
        score = min([
            (pos[0] - point[0]) ** 2
            + (pos[1] - point[1]) ** 2
            + (pos[2] - point[2]) ** 2
            for point in stove_points
        ])
        if score < best_score:
            best_score = score
            bowl_name = name

    plate_name = plates[0]
    held = False
    yaw = state["eef_yaw"]

    for attempt in range(3):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 170:
            break

        bowl = list(state["objects"][bowl_name]["pos"])
        yaw = state["eef_yaw"] + (pi / 2.0 if attempt == 1 else 0.0)
        grasp_height = bowl[2] + (0.012 if attempt < 2 else 0.025)
        approach_height = max(state["eef_pos"][2], bowl[2] + 0.15)

        policy_gripper(sdk, "open", 8)
        if not policy_move(
            sdk,
            [state["eef_pos"][0], state["eef_pos"][1], approach_height],
            yaw, "open", 25,
        ):
            continue
        if not policy_move(
            sdk, [bowl[0], bowl[1], approach_height], yaw, "open", 40,
        ):
            continue

        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        grasp_height = bowl[2] + (0.012 if attempt < 2 else 0.025)
        if not policy_move(
            sdk, [bowl[0], bowl[1], grasp_height], yaw, "open", 35, 0.005,
        ):
            continue

        policy_gripper(sdk, "close", 14)
        before = sdk.get_state()
        original_z = before["objects"][bowl_name]["pos"][2]
        eef = before["eef_pos"]
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.12], yaw, "close", 35,
        )
        after = sdk.get_state()
        lifted = after["objects"][bowl_name]["pos"]
        if lifted[2] - original_z > 0.055:
            held = True
            break

    if not held:
        return {"status": "failed", "reason": "Could not verify bowl pickup"}

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    travel_height = max(eef[2], plate[2] + 0.18 - offset[2])

    if not policy_move(
        sdk, [eef[0], eef[1], travel_height], yaw, "close", 30,
    ):
        return {"status": "failed", "reason": "Could not reach transport height"}

    if not policy_move(
        sdk,
        [plate[0] - offset[0], plate[1] - offset[1], travel_height],
        yaw, "close", 55,
    ):
        return {"status": "failed", "reason": "Could not reach plate"}

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    eef = state["eef_pos"]
    if abs((bowl[2] - eef[2]) - offset[2]) > 0.045:
        return {"status": "failed", "reason": "Bowl slipped during transport"}

    offset = [bowl[i] - eef[i] for i in range(3)]
    plate = state["objects"][plate_name]["pos"]
    target = [
        plate[0] - offset[0],
        plate[1] - offset[1],
        plate[2] + 0.028 - offset[2],
    ]
    if not policy_move(sdk, target, yaw, "close", 45, 0.006):
        return {"status": "failed", "reason": "Could not lower bowl onto plate"}

    policy_gripper(sdk, "open", 15)
    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.12], yaw, "open", 30,
    )
    policy_gripper(sdk, "open", 12)

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    centered = hypot(bowl[0] - plate[0], bowl[1] - plate[1]) < 0.045
    supported_height = -0.01 < bowl[2] - plate[2] < 0.065
    if centered and supported_height:
        return {"status": "done", "object": bowl_name}
    return {"status": "failed", "reason": "Final placement could not be verified"}
