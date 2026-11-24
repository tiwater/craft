def run_policy(sdk):
    state = sdk.get_state()
    if "cube" not in state["objects"]:
        return {"status": "failed", "reason": "Cube observation unavailable"}

    initial_cube_z = state["objects"]["cube"]["pos"][2]
    lift_height = 0.14
    attempts = 0

    def remaining():
        observation = sdk.get_state()
        return max(0, min(400, observation["max_steps"]) - observation["steps_used"])

    def move(position, yaw, gripper, budget, tolerance):
        available = remaining()
        if available <= 0:
            return False
        result = sdk.move_to(
            position,
            yaw=yaw,
            gripper=gripper,
            max_steps=min(budget, available),
            tol=tolerance,
        )
        return result["reached"]

    for attempt in range(3):
        if remaining() < 65:
            break
        attempts += 1
        state = sdk.get_state()
        cube = state["objects"]["cube"]
        position = cube["pos"]
        yaw = cube["yaw"]

        # A square cube admits equivalent grasp orientations every quarter turn.
        while yaw - state["eef_yaw"] > pi / 4:
            yaw -= pi / 2
        while yaw - state["eef_yaw"] < -pi / 4:
            yaw += pi / 2

        # Retreat vertically before opening or translating across the table.
        clearance = max(state["eef_pos"][2], position[2] + 0.10)
        move(
            [state["eef_pos"][0], state["eef_pos"][1], clearance],
            yaw,
            None,
            25,
            0.012,
        )
        if remaining() < 45:
            break
        sdk.set_gripper("open", steps=min(12, remaining()))

        state = sdk.get_state()
        position = state["objects"]["cube"]["pos"]
        reached = move(
            [position[0], position[1], position[2] + 0.10],
            yaw,
            "open",
            45,
            0.008,
        )
        if not reached:
            continue

        # Reobserve after approaching; contact can shift the cube.
        state = sdk.get_state()
        position = state["objects"]["cube"]["pos"]
        grasp_z = position[2] + (0.004 if attempt != 1 else 0.0)
        reached = move(
            [position[0], position[1], grasp_z],
            yaw,
            "open",
            40,
            0.005,
        )
        if not reached or remaining() < 30:
            continue

        sdk.set_gripper("close", steps=min(20, remaining()))
        state = sdk.get_state()
        cube_before = list(state["objects"]["cube"]["pos"])
        eef_before = list(state["eef_pos"])

        # Test attachment with a short vertical lift using object motion.
        move(
            [eef_before[0], eef_before[1], eef_before[2] + 0.055],
            yaw,
            "close",
            30,
            0.008,
        )
        state = sdk.get_state()
        cube_now = state["objects"]["cube"]["pos"]
        if cube_now[2] - cube_before[2] < 0.025:
            continue

        eef = state["eef_pos"]
        target_z = eef[2] + max(
            0.0, initial_cube_z + lift_height - cube_now[2]
        )
        move(
            [eef[0], eef[1], target_z],
            yaw,
            "close",
            45,
            0.008,
        )

        if remaining() < 8:
            break
        sdk.set_gripper("close", steps=min(12, remaining()))
        state = sdk.get_state()
        cube_now = state["objects"]["cube"]["pos"]
        rise = cube_now[2] - initial_cube_z
        separation = sqrt(sum(
            (cube_now[axis] - state["eef_pos"][axis]) ** 2
            for axis in range(3)
        ))
        if rise >= 0.10 and separation <= 0.08:
            return {
                "status": "done",
                "cube_lift_m": rise,
                "attempts": attempts,
                "steps_used": state["steps_used"],
            }

    state = sdk.get_state()
    return {
        "status": "failed",
        "reason": "Could not verify a sustained cube lift within the step budget",
        "cube_lift_m": state["objects"]["cube"]["pos"][2] - initial_cube_z,
        "attempts": attempts,
        "steps_used": state["steps_used"],
    }
