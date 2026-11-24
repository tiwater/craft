def lift_move(sdk, target, yaw, gripper, limit, tolerance=0.01):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    sdk.move_to(
        target,
        yaw=yaw,
        gripper=gripper,
        max_steps=min(limit, remaining),
        tol=tolerance,
    )
    state = sdk.get_state()
    eef = state["eef_pos"]
    return (
        hypot(eef[0] - target[0], eef[1] - target[1]) <= 0.012
        and abs(eef[2] - target[2]) <= 0.015
    )


def lift_gripper(sdk, action, ticks):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining > 0:
        sdk.set_gripper(action, steps=min(ticks, remaining))


def run_policy(sdk):
    state = sdk.get_state()
    resting_z = state["objects"]["cube"]["pos"][2]

    for attempt in range(3):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 85:
            break

        cube = state["objects"]["cube"]
        yaw = cube["yaw"]
        yaw += round((state["eef_yaw"] - yaw) / (pi / 2)) * (pi / 2)
        eef = state["eef_pos"]
        clearance = max(resting_z + 0.13, cube["pos"][2] + 0.12)

        lift_move(
            sdk, [eef[0], eef[1], clearance], yaw, None, 30
        )
        state = sdk.get_state()
        if state["eef_pos"][2] < state["objects"]["cube"]["pos"][2] + 0.07:
            continue

        lift_gripper(sdk, "open", 12)
        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        if not lift_move(
            sdk, [pos[0], pos[1], clearance], yaw, "open", 35
        ):
            continue

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        grasp_offset = 0.005 if attempt == 0 else 0.0

        # The rollouts stalled about 6 mm above the cube center.
        # A 5 mm convergence requirement prevented closure entirely.
        lift_move(
            sdk,
            [pos[0], pos[1], pos[2] + grasp_offset],
            yaw,
            "open",
            35,
            0.009,
        )

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if hypot(pos[0] - eef[0], pos[1] - eef[1]) > 0.008:
            lift_move(
                sdk,
                [pos[0], pos[1], pos[2] + grasp_offset],
                yaw,
                "open",
                15,
                0.008,
            )

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if (
            hypot(pos[0] - eef[0], pos[1] - eef[1]) > 0.012
            or eef[2] - pos[2] > 0.022
            or eef[2] - pos[2] < -0.015
        ):
            continue

        lift_gripper(sdk, "close", 20)
        state = sdk.get_state()
        eef = state["eef_pos"]
        cube_before = list(state["objects"]["cube"]["pos"])

        # Test actual object motion rather than relying on finger width.
        lift_move(
            sdk,
            [eef[0], eef[1], eef[2] + 0.055],
            yaw,
            "close",
            22,
        )
        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        if pos[2] - cube_before[2] < 0.018:
            continue

        eef = state["eef_pos"]
        required_height = resting_z + 0.16
        goal = state["goal_pos"]
        if goal is not None:
            required_height = max(required_height, goal[2] + 0.035)
        target_z = eef[2] + max(0.04, required_height - pos[2])

        lift_move(
            sdk,
            [eef[0], eef[1], target_z],
            yaw,
            "close",
            40,
        )
        state = sdk.get_state()
        if state["objects"]["cube"]["pos"][2] < resting_z + 0.12:
            continue

        if state["max_steps"] - state["steps_used"] < 10:
            break
        lift_gripper(sdk, "close", 10)
        state = sdk.get_state()
        height = state["objects"]["cube"]["pos"][2] - resting_z
        if height >= 0.12:
            return {
                "status": "done",
                "lift_height_m": height,
                "steps_used": state["steps_used"],
            }

    state = sdk.get_state()
    return {
        "status": "failed",
        "reason": "Could not confirm a sustained cube lift within the step budget.",
        "lift_height_m": state["objects"]["cube"]["pos"][2] - resting_z,
        "steps_used": state["steps_used"],
    }
