def policy_move(sdk, pos, yaw, gripper, steps=45, tol=0.008):
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
    plates = [name for name in objects if "plate" in name]
    ramekins = [name for name in objects if "ramekin" in name]
    if not bowls or not plates or not ramekins:
        return {"status": "failed", "reason": "Required objects are missing"}

    plate = plates[0]
    ramekin = ramekins[0]
    plate_pos = objects[plate]["pos"]
    ramekin_pos = objects[ramekin]["pos"]
    vx = ramekin_pos[0] - plate_pos[0]
    vy = ramekin_pos[1] - plate_pos[1]
    length_squared = max(vx * vx + vy * vy, 0.000001)
    region = None
    for name in state["sites"]:
        if "between_plate_ramekin" in name:
            region = state["sites"][name]
            break

    target = None
    best_score = 1000000.0
    for name in bowls:
        pos = objects[name]["pos"]
        t = ((pos[0] - plate_pos[0]) * vx +
             (pos[1] - plate_pos[1]) * vy) / length_squared
        bounded = max(0.0, min(1.0, t))
        px = plate_pos[0] + bounded * vx
        py = plate_pos[1] + bounded * vy
        score = (pos[0] - px) ** 2 + (pos[1] - py) ** 2
        score += 0.12 * (t - 0.5) ** 2 * length_squared
        if region is not None:
            score += 0.5 * (
                (pos[0] - region[0]) ** 2 +
                (pos[1] - region[1]) ** 2
            )
        if score < best_score:
            best_score = score
            target = name

    initial_bowl_z = objects[target]["pos"][2]
    initial_plate_z = plate_pos[2]
    yaw = state["eef_yaw"]
    held = False

    for attempt in range(3):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 165:
            break
        pos = state["objects"][target]["pos"]
        baseline_z = pos[2]
        clearance = max(
            state["eef_pos"][2],
            pos[2] + 0.15,
            state["objects"][plate]["pos"][2] + 0.15,
        )
        eef = state["eef_pos"]
        if not policy_move(
            sdk, [eef[0], eef[1], clearance], yaw, "open", 35
        ):
            continue
        policy_gripper(sdk, "open", 8)

        yaw = state["eef_yaw"] + attempt * pi / 2.0
        if not policy_move(
            sdk, [pos[0], pos[1], clearance], yaw, "open", 45
        ):
            continue

        state = sdk.get_state()
        pos = state["objects"][target]["pos"]
        grasp_offset = [0.015, 0.0, 0.025][attempt]
        if not policy_move(
            sdk,
            [pos[0], pos[1], pos[2] + grasp_offset],
            yaw,
            "open",
            45,
            0.006,
        ):
            continue

        policy_gripper(sdk, "close", 15)
        state = sdk.get_state()
        eef = state["eef_pos"]
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.12], yaw, "close", 40
        )
        state = sdk.get_state()
        bowl_pos = state["objects"][target]["pos"]
        if bowl_pos[2] - baseline_z > 0.055:
            held = True
            break

    if not held:
        return {"status": "failed", "reason": "No verified bowl lift"}

    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    offset = [bowl_pos[i] - eef[i] for i in range(3)]
    plate_pos = state["objects"][plate]["pos"]
    transit_z = max(eef[2], plate_pos[2] + 0.16 - offset[2])
    destination = [
        plate_pos[0] - offset[0],
        plate_pos[1] - offset[1],
        transit_z,
    ]
    if not policy_move(sdk, destination, yaw, "close", 60):
        return {"status": "failed", "reason": "Could not reach plate"}

    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    if bowl_pos[2] < initial_bowl_z + 0.04:
        return {"status": "failed", "reason": "Bowl slipped during transport"}

    offset = [bowl_pos[i] - eef[i] for i in range(3)]
    plate_pos = state["objects"][plate]["pos"]
    resting_z = plate_pos[2] + initial_bowl_z - initial_plate_z + 0.018
    destination = [
        plate_pos[0] - offset[0],
        plate_pos[1] - offset[1],
        resting_z - offset[2],
    ]
    if not policy_move(sdk, destination, yaw, "close", 45, 0.006):
        state = sdk.get_state()
        bowl_pos = state["objects"][target]["pos"]
        if (hypot(bowl_pos[0] - plate_pos[0],
                  bowl_pos[1] - plate_pos[1]) > 0.035 or
                abs(bowl_pos[2] - resting_z) > 0.025):
            return {"status": "failed", "reason": "Placement pose not reached"}

    if not policy_gripper(sdk, "open", 15):
        return {"status": "failed", "reason": "No time remaining to release"}

    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.12], yaw, "open", 30
    )
    policy_gripper(sdk, "open", 10)
    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    centered = hypot(
        bowl_pos[0] - plate_pos[0], bowl_pos[1] - plate_pos[1]
    ) < 0.04
    settled = abs(
        bowl_pos[2] -
        (plate_pos[2] + initial_bowl_z - initial_plate_z + 0.018)
    ) < 0.045
    return {
        "status": "done" if centered and settled else "failed",
        "target": target,
        "steps_used": state["steps_used"],
    }
