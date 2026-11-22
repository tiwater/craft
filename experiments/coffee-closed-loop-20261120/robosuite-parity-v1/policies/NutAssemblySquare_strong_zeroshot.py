def policy_angle(angle):
    return atan2(sin(angle), cos(angle))


def policy_move(sdk, pos, yaw, gripper, limit, tol=0.004):
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


def policy_gripper(sdk, action, limit):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining > 0:
        sdk.set_gripper(action, steps=min(limit, remaining))


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg position"}

    resting_z = state["objects"]["nut"]["pos"][2]
    grasped = False

    for attempt in range(2):
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining < 180:
            break

        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        handle = [
            nut["pos"][0] + 0.055 * cos(yaw),
            nut["pos"][1] + 0.055 * sin(yaw),
            nut["pos"][2],
        ]
        grasp_yaw = state["eef_yaw"] + (
            policy_angle(2.0 * (yaw - state["eef_yaw"])) / 2.0
        )
        safe_z = max(state["eef_pos"][2], handle[2] + 0.10)

        policy_gripper(sdk, "open", 8)
        policy_move(
            sdk,
            [state["eef_pos"][0], state["eef_pos"][1], safe_z],
            grasp_yaw, "open", 18,
        )
        if not policy_move(
            sdk, [handle[0], handle[1], safe_z],
            grasp_yaw, "open", 30,
        ):
            continue

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        handle = [
            nut["pos"][0] + 0.055 * cos(nut["yaw"]),
            nut["pos"][1] + 0.055 * sin(nut["yaw"]),
            nut["pos"][2],
        ]
        if not policy_move(sdk, handle, grasp_yaw, "open", 25, 0.003):
            continue

        before_z = sdk.get_state()["objects"]["nut"]["pos"][2]
        policy_gripper(sdk, "close", 12)
        state = sdk.get_state()
        width = state["gripper_width"]
        if width < 0.012 or width > 0.070:
            policy_gripper(sdk, "open", 6)
            continue

        eef = state["eef_pos"]
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.10],
            grasp_yaw, "close", 28,
        )
        state = sdk.get_state()
        grasped = (
            state["objects"]["nut"]["pos"][2] > before_z + 0.045
            and 0.012 < state["gripper_width"] < 0.070
        )
        if grasped:
            break

        policy_gripper(sdk, "open", 6)

    if not grasped:
        return {"status": "failed", "reason": "Handle grasp or lift verification failed"}

    state = sdk.get_state()
    goal = state["goal_pos"]
    peg_yaw = 0.0
    for name in ("peg", "square_peg", "peg1"):
        if name in state["objects"]:
            peg_yaw = state["objects"][name]["yaw"]
            break

    clearance_z = max(goal[2] + 0.10, resting_z + 0.14)
    nut = state["objects"]["nut"]
    eef = state["eef_pos"]
    policy_move(
        sdk,
        [eef[0], eef[1], eef[2] + max(0.0, clearance_z - nut["pos"][2])],
        state["eef_yaw"], "close", 20,
    )

    # A square hole admits four equivalent peg orientations.
    state = sdk.get_state()
    rotation = policy_angle(
        4.0 * (peg_yaw - state["objects"]["nut"]["yaw"])
    ) / 4.0
    target_yaw = state["eef_yaw"] + rotation
    policy_move(sdk, state["eef_pos"], target_yaw, "close", 22)

    # Measure the actual held hole-to-hand offset after rotation.
    aligned = False
    for correction in range(3):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        eef = state["eef_pos"]
        yaw_error = policy_angle(4.0 * (peg_yaw - nut["yaw"])) / 4.0
        target_yaw = state["eef_yaw"] + yaw_error
        policy_move(
            sdk,
            [
                eef[0] + goal[0] - nut["pos"][0],
                eef[1] + goal[1] - nut["pos"][1],
                eef[2] + clearance_z - nut["pos"][2],
            ],
            target_yaw, "close", 27 if correction == 0 else 16, 0.002,
        )
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        aligned = (
            hypot(
                nut["pos"][0] - state["goal_pos"][0],
                nut["pos"][1] - state["goal_pos"][1],
            ) < 0.0035
            and abs(policy_angle(4.0 * (peg_yaw - nut["yaw"])) / 4.0) < 0.045
        )
        if aligned:
            break

    if not aligned:
        return {"status": "failed", "reason": "Hole alignment did not converge"}

    # Preserve hand XY and yaw throughout insertion to avoid sweeping the peg.
    state = sdk.get_state()
    eef = state["eef_pos"]
    nut_z = state["objects"]["nut"]["pos"][2]
    insertion_x = eef[0]
    insertion_y = eef[1]
    insertion_yaw = state["eef_yaw"]
    hand_height_offset = eef[2] - nut_z
    final_nut_z = resting_z + 0.002

    for fraction in (0.45, 0.75, 1.0):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 25:
            return {"status": "failed", "reason": "Insufficient insertion budget"}
        target_z = nut_z + fraction * (final_nut_z - nut_z)
        policy_move(
            sdk,
            [insertion_x, insertion_y, target_z + hand_height_offset],
            insertion_yaw, "close", 18, 0.002,
        )
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        if hypot(
            nut["pos"][0] - state["goal_pos"][0],
            nut["pos"][1] - state["goal_pos"][1],
        ) > 0.008:
            return {"status": "failed", "reason": "Nut shifted during insertion"}

    state = sdk.get_state()
    if state["objects"]["nut"]["pos"][2] > resting_z + 0.018:
        return {"status": "failed", "reason": "Nut remained above seating height"}

    policy_gripper(sdk, "open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.10],
        insertion_yaw, "open", 20,
    )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    error = hypot(
        nut["pos"][0] - state["goal_pos"][0],
        nut["pos"][1] - state["goal_pos"][1],
    )
    seated = (
        error < 0.010
        and resting_z - 0.015 < nut["pos"][2] < resting_z + 0.018
        and state["gripper_width"] > 0.050
    )
    return {
        "status": "done" if seated else "failed",
        "reason": "Geometric seating verified" if seated else "Seating verification failed",
        "hole_xy_error_m": error,
        "steps_used": state["steps_used"],
    }
