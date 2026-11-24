def policy_move(sdk, pos, yaw, gripper, steps=40, tol=0.01):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        pos, yaw=yaw, gripper=gripper,
        max_steps=min(steps, remaining), tol=tol
    )
    return result["reached"]


def policy_gripper(sdk, action, steps=12):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    sdk.set_gripper(action, steps=min(steps, remaining))
    return True


def policy_contact(sdk, target, offset, yaw, steps, reserve):
    state = sdk.get_state()
    bowl = state["objects"][target]
    pos = bowl["pos"]
    goal = [
        pos[0] + offset * cos(bowl["yaw"]),
        pos[1] + offset * sin(bowl["yaw"]),
        pos[2] + 0.015
    ]
    available = state["max_steps"] - state["steps_used"] - reserve
    if available <= 0:
        return False
    policy_move(sdk, goal, yaw, "open", min(steps, available), 0.006)

    state = sdk.get_state()
    eef = state["eef_pos"]
    error = sqrt(sum((eef[i] - goal[i]) ** 2 for i in range(3)))
    if error <= 0.012:
        return True

    # Continue an incomplete descent before actuating the fingers.
    available = state["max_steps"] - state["steps_used"] - reserve
    if error < 0.04 and available >= 8:
        bowl = state["objects"][target]
        pos = bowl["pos"]
        goal = [
            pos[0] + offset * cos(bowl["yaw"]),
            pos[1] + offset * sin(bowl["yaw"]),
            pos[2] + 0.015
        ]
        policy_move(sdk, goal, yaw, "open", min(16, available), 0.006)
        state = sdk.get_state()
        eef = state["eef_pos"]
        error = sqrt(sum((eef[i] - goal[i]) ** 2 for i in range(3)))
    return error <= 0.012


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
    best = 1000000.0
    for name in bowls:
        pos = objects[name]["pos"]
        score = hypot(pos[0] - ramekin[0], pos[1] - ramekin[1])
        if region is not None:
            score += hypot(pos[0] - region[0], pos[1] - region[1])
        if score < best:
            best = score
            target = name

    plate = plates[0]
    initial_bowl_z = objects[target]["pos"][2]
    initial_plate_z = objects[plate]["pos"][2]
    grasp_yaw = state["eef_yaw"]
    held = False
    positive_contact_reached = True

    for rim_offset in [0.045, 0.060, -0.050]:
        # The farther positive rim does not help an unreachable positive approach.
        if rim_offset == 0.060 and not positive_contact_reached:
            continue
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining < 235:
            break

        bowl = state["objects"][target]
        pos = bowl["pos"]
        eef = state["eef_pos"]
        clearance = max(
            eef[2], pos[2] + 0.14,
            state["objects"][plate]["pos"][2] + 0.14
        )
        lifted = policy_move(
            sdk, [eef[0], eef[1], clearance],
            grasp_yaw, "open", 25
        )
        policy_gripper(sdk, "open", 6)
        if not lifted:
            if rim_offset > 0:
                positive_contact_reached = False
            continue

        state = sdk.get_state()
        bowl = state["objects"][target]
        pos = bowl["pos"]
        contact_x = pos[0] + rim_offset * cos(bowl["yaw"])
        contact_y = pos[1] + rim_offset * sin(bowl["yaw"])
        aligned = policy_move(
            sdk, [contact_x, contact_y, clearance],
            grasp_yaw, "open", 35, 0.008
        )
        if not aligned:
            if rim_offset > 0:
                positive_contact_reached = False
            continue

        state = sdk.get_state()
        bowl = state["objects"][target]
        pos = bowl["pos"]
        contact_x = pos[0] + rim_offset * cos(bowl["yaw"])
        contact_y = pos[1] + rim_offset * sin(bowl["yaw"])
        policy_move(
            sdk, [contact_x, contact_y, pos[2] + 0.060],
            grasp_yaw, "open", 22, 0.008
        )
        contacted = policy_contact(
            sdk, target, rim_offset, grasp_yaw, 25, 202
        )
        if rim_offset > 0:
            positive_contact_reached = contacted
        if not contacted:
            continue

        state = sdk.get_state()
        baseline_z = state["objects"][target]["pos"][2]
        policy_gripper(sdk, "close", 14)
        state = sdk.get_state()
        eef = state["eef_pos"]
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.085],
            grasp_yaw, "close", 28, 0.008
        )
        state = sdk.get_state()
        pos = state["objects"][target]["pos"]
        width = state["gripper_width"]
        held = (
            pos[2] - baseline_z >= 0.03
            and 0.004 <= width <= 0.04
            and hypot(
                pos[0] - state["eef_pos"][0],
                pos[1] - state["eef_pos"][1]
            ) < 0.105
        )
        if held:
            break
        policy_gripper(sdk, "open", 6)

    if not held:
        return {
            "status": "failed",
            "reason": "Rim grasp did not pass lift and width verification"
        }

    state = sdk.get_state()
    pos = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    offset = [pos[i] - eef[i] for i in range(3)]
    plate_pos = state["objects"][plate]["pos"]
    travel_z = max(eef[2], plate_pos[2] + 0.13 - offset[2])
    if not policy_move(
        sdk, [eef[0], eef[1], travel_z],
        grasp_yaw, "close", 25, 0.01
    ):
        return {"status": "failed", "reason": "Could not establish transport clearance"}

    state = sdk.get_state()
    pos = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    if pos[2] < initial_bowl_z + 0.04 or state["gripper_width"] < 0.004:
        return {"status": "failed", "reason": "Grasp lost during clearance lift"}

    offset = [pos[i] - eef[i] for i in range(3)]
    plate_pos = state["objects"][plate]["pos"]
    if not policy_move(
        sdk,
        [plate_pos[0] - offset[0], plate_pos[1] - offset[1], travel_z],
        grasp_yaw, "close", 50, 0.008
    ):
        return {"status": "failed", "reason": "Could not reach plate"}

    state = sdk.get_state()
    pos = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    if pos[2] < initial_bowl_z + 0.04 or state["gripper_width"] < 0.004:
        return {"status": "failed", "reason": "Bowl slipped during transport"}

    offset = [pos[i] - eef[i] for i in range(3)]
    plate_pos = state["objects"][plate]["pos"]
    resting_z = initial_bowl_z + plate_pos[2] - initial_plate_z + 0.025
    release_pos = [
        plate_pos[0] - offset[0],
        plate_pos[1] - offset[1],
        resting_z - offset[2]
    ]
    if not policy_move(
        sdk, release_pos, grasp_yaw, "close", 38, 0.008
    ):
        return {"status": "failed", "reason": "Placement approach incomplete"}

    if not policy_gripper(sdk, "open", 14):
        return {"status": "failed", "reason": "No remaining steps for release"}

    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.12],
        grasp_yaw, "open", 25, 0.012
    )
    policy_gripper(sdk, "open", 8)

    state = sdk.get_state()
    pos = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    expected_z = initial_bowl_z + plate_pos[2] - initial_plate_z
    placed = (
        hypot(pos[0] - plate_pos[0], pos[1] - plate_pos[1]) < 0.045
        and expected_z - 0.015 <= pos[2] <= expected_z + 0.065
        and state["gripper_width"] > 0.04
        and state["eef_pos"][2] - pos[2] > 0.075
    )
    return {
        "status": "done" if placed else "failed",
        "object": target,
        "reason": "Placement verified geometrically" if placed else "Placement not verified"
    }
