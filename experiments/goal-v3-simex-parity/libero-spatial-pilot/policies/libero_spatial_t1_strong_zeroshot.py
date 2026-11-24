def policy_move(sdk, pos, yaw, gripper, steps=55, tol=0.012):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        pos,
        yaw=yaw,
        gripper=gripper,
        max_steps=min(steps, remaining),
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
    ramekins = [name for name in objects if "ramekin" in name]
    plates = [name for name in objects if "plate" in name]
    if not bowls or not ramekins or not plates:
        return {"status": "failed", "reason": "Required objects unavailable"}

    ramekin = objects[ramekins[0]]["pos"]
    region = state["sites"].get("main_table_next_to_ramekin_region")
    target = bowls[0]
    best = float("inf")
    for name in bowls:
        pos = objects[name]["pos"]
        distance = hypot(pos[0] - ramekin[0], pos[1] - ramekin[1])
        if region is not None:
            distance += hypot(pos[0] - region[0], pos[1] - region[1])
        if distance < best:
            best = distance
            target = name

    plate = plates[0]
    initial_bowl_z = objects[target]["pos"][2]
    initial_plate_z = objects[plate]["pos"][2]
    held = False
    grasp_yaw = state["eef_yaw"]

    for attempt in range(3):
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining < 185:
            break

        bowl_pos = list(state["objects"][target]["pos"])
        baseline_z = bowl_pos[2]
        grasp_yaw = state["eef_yaw"]
        if attempt == 1:
            grasp_yaw += pi / 2.0
        grasp_height = bowl_pos[2] + 0.015
        if attempt == 2:
            grasp_height = bowl_pos[2] - 0.008

        clearance = max(
            state["eef_pos"][2],
            bowl_pos[2] + 0.15,
            state["objects"][plate]["pos"][2] + 0.15,
        )
        if not policy_move(
            sdk,
            [state["eef_pos"][0], state["eef_pos"][1], clearance],
            grasp_yaw, "open", 35
        ):
            continue

        policy_gripper(sdk, "open", 8)
        if not policy_move(
            sdk, [bowl_pos[0], bowl_pos[1], clearance],
            grasp_yaw, "open", 45
        ):
            continue

        state = sdk.get_state()
        bowl_pos = list(state["objects"][target]["pos"])
        if not policy_move(
            sdk, [bowl_pos[0], bowl_pos[1], grasp_height],
            grasp_yaw, "open", 40, 0.007
        ):
            continue

        policy_gripper(sdk, "close", 14)
        state = sdk.get_state()
        eef = state["eef_pos"]
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.12],
            grasp_yaw, "close", 40, 0.01
        )
        state = sdk.get_state()
        lifted = state["objects"][target]["pos"]
        if (
            lifted[2] > baseline_z + 0.05
            and hypot(
                lifted[0] - state["eef_pos"][0],
                lifted[1] - state["eef_pos"][1],
            ) < 0.09
        ):
            held = True
            break
        policy_gripper(sdk, "open", 8)

    if not held:
        return {"status": "failed", "reason": "Bowl did not lift"}

    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    offset = [bowl_pos[i] - eef[i] for i in range(3)]
    plate_pos = state["objects"][plate]["pos"]
    travel_z = max(eef[2], plate_pos[2] + 0.16 - offset[2])

    if not policy_move(
        sdk,
        [plate_pos[0] - offset[0], plate_pos[1] - offset[1], travel_z],
        grasp_yaw, "close", 60, 0.01
    ):
        return {"status": "failed", "reason": "Could not reach plate"}

    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    if bowl_pos[2] < initial_bowl_z + 0.045:
        return {"status": "failed", "reason": "Bowl slipped during transfer"}

    eef = state["eef_pos"]
    offset = [bowl_pos[i] - eef[i] for i in range(3)]
    plate_pos = state["objects"][plate]["pos"]
    resting_z = initial_bowl_z + plate_pos[2] - initial_plate_z + 0.025
    release_pos = [
        plate_pos[0] - offset[0],
        plate_pos[1] - offset[1],
        resting_z - offset[2],
    ]
    if not policy_move(
        sdk, release_pos, grasp_yaw, "close", 45, 0.008
    ):
        return {"status": "failed", "reason": "Placement approach incomplete"}

    if not policy_gripper(sdk, "open", 15):
        return {"status": "failed", "reason": "No time remaining to release"}

    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.12],
        grasp_yaw, "open", 30, 0.015
    )
    policy_gripper(sdk, "open", 10)

    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    expected_z = initial_bowl_z + plate_pos[2] - initial_plate_z
    placed = (
        hypot(bowl_pos[0] - plate_pos[0], bowl_pos[1] - plate_pos[1]) < 0.045
        and expected_z - 0.015 <= bowl_pos[2] <= expected_z + 0.065
        and state["gripper_width"] > 0.04
    )
    return {
        "status": "done" if placed else "failed",
        "object": target,
        "reason": "Placement verified geometrically" if placed else "Placement not verified",
    }
