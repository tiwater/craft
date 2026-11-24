def policy_move(sdk, pos, yaw, gripper, limit, tol=0.012):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return {"reached": False}
    return sdk.move_to(
        pos,
        yaw=yaw,
        gripper=gripper,
        max_steps=min(limit, remaining),
        tol=tol,
    )


def policy_grip(sdk, action, ticks):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining > 0:
        return sdk.set_gripper(action, steps=min(ticks, remaining))
    return {"grasped_hint": False}


def run_policy(sdk):
    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or target"}

    yaw = state["eef_yaw"]
    held = False

    # The previous descent stalled near the table, just outside its strict
    # tolerance. Grasp higher and judge alignment from live geometry.
    for attempt in range(2):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 195:
            break

        can = list(state["objects"]["can"]["pos"])
        hover = [
            can[0],
            can[1],
            max(can[2] + 0.15, state["eef_pos"][2]),
        ]
        policy_move(sdk, hover, yaw, "open", 35)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.025:
            continue

        grasp_height = 0.025 if attempt == 0 else 0.018
        policy_move(
            sdk,
            [can[0], can[1], can[2] + grasp_height],
            yaw,
            "open",
            30,
            0.01,
        )

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        aligned = (
            hypot(eef[0] - can[0], eef[1] - can[1]) < 0.018
            and 0.005 < eef[2] - can[2] < 0.055
        )
        if not aligned:
            continue

        initial_height = can[2]
        policy_grip(sdk, "close", 18)

        state = sdk.get_state()
        eef = list(state["eef_pos"])
        policy_move(
            sdk,
            [eef[0], eef[1], eef[2] + 0.12],
            yaw,
            "close",
            28,
        )

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        separation = sqrt(sum((can[i] - eef[i]) ** 2 for i in range(3)))
        held = (
            can[2] > initial_height + 0.06
            and separation < 0.12
            and 0.012 < state["gripper_width"] < 0.075
        )
        if held:
            break
        policy_grip(sdk, "open", 8)

    if not held:
        return {"status": "failed", "reason": "Could not verify lifted grasp"}

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    eef = list(state["eef_pos"])
    goal = list(state["goal_pos"])
    offset = [can[i] - eef[i] for i in range(3)]

    # Establish wall clearance before moving horizontally across the bins.
    clearance = max(can[2], initial_height + 0.22, goal[2] + 0.20)
    carry_height = clearance - offset[2]
    policy_move(
        sdk,
        [eef[0], eef[1], carry_height],
        yaw,
        "close",
        25,
    )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if can[2] < clearance - 0.025:
        return {"status": "failed", "reason": "Insufficient transport clearance"}

    policy_move(
        sdk,
        [goal[0] - offset[0], goal[1] - offset[1], carry_height],
        yaw,
        "close",
        65,
    )

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    eef = list(state["eef_pos"])
    current_offset = [can[i] - eef[i] for i in range(3)]
    slip = sqrt(sum((current_offset[i] - offset[i]) ** 2 for i in range(3)))
    if slip > 0.05 or state["gripper_width"] < 0.012:
        return {"status": "failed", "reason": "Grasp lost during transport"}

    if hypot(can[0] - goal[0], can[1] - goal[1]) > 0.025:
        policy_move(
            sdk,
            [
                goal[0] - current_offset[0],
                goal[1] - current_offset[1],
                eef[2],
            ],
            yaw,
            "close",
            20,
            0.01,
        )

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    eef = list(state["eef_pos"])
    if hypot(can[0] - goal[0], can[1] - goal[1]) > 0.035:
        return {"status": "failed", "reason": "Could not align above target"}

    if state["max_steps"] - state["steps_used"] < 70:
        return {"status": "failed", "reason": "Insufficient placement budget"}

    current_offset = [can[i] - eef[i] for i in range(3)]
    policy_move(
        sdk,
        [
            goal[0] - current_offset[0],
            goal[1] - current_offset[1],
            goal[2] + 0.065 - current_offset[2],
        ],
        yaw,
        "close",
        30,
        0.012,
    )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.04
        or can[2] > goal[2] + 0.11
        or can[2] < goal[2] - 0.025
    ):
        return {"status": "failed", "reason": "Unsafe release position"}

    policy_grip(sdk, "open", 15)
    state = sdk.get_state()
    eef = list(state["eef_pos"])
    policy_move(
        sdk,
        [eef[0], eef[1], max(eef[2] + 0.16, carry_height)],
        yaw,
        "open",
        25,
    )
    policy_grip(sdk, "open", 12)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    horizontal_error = hypot(can[0] - goal[0], can[1] - goal[1])
    placed = (
        horizontal_error < 0.06
        and goal[2] - 0.03 < can[2] < goal[2] + 0.10
        and eef[2] - can[2] > 0.10
        and state["gripper_width"] > 0.055
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Released in target bin" if placed else "Placement not verified",
        "horizontal_error_m": horizontal_error,
    }
