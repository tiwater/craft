def run_policy(sdk):
    state = sdk.get_state()
    cube = state["objects"]["cube"]
    initial_cube_z = cube["pos"][2]
    lift_threshold = 0.09
    attempts = 0

    for attempt in range(3):
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining < 95:
            break
        attempts += 1

        sdk.set_gripper("open", steps=10)
        state = sdk.get_state()
        cube = state["objects"]["cube"]
        pos = cube["pos"]

        yaw = cube["yaw"]
        while yaw - state["eef_yaw"] > pi / 4:
            yaw -= pi / 2
        while yaw - state["eef_yaw"] < -pi / 4:
            yaw += pi / 2

        # Clear the cube before moving sideways.
        if state["eef_pos"][2] < pos[2] + 0.07:
            sdk.move_to(
                [state["eef_pos"][0], state["eef_pos"][1], pos[2] + 0.10],
                gripper="open",
                max_steps=20,
                tol=0.008,
            )

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        sdk.move_to(
            [pos[0], pos[1], pos[2] + 0.10],
            yaw=yaw,
            gripper="open",
            max_steps=35,
            tol=0.006,
        )

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.018:
            continue
        if state["max_steps"] - state["steps_used"] < 80:
            break

        # The end-effector site is the gripping center.
        sdk.move_to(
            [pos[0], pos[1], pos[2] + 0.002],
            yaw=yaw,
            gripper="open",
            max_steps=35,
            tol=0.004,
        )

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.012
            or abs(eef[2] - pos[2]) > 0.016
        ):
            continue

        sdk.set_gripper("close", steps=18)
        state = sdk.get_state()
        eef = state["eef_pos"]
        remaining = state["max_steps"] - state["steps_used"]
        if remaining < 12:
            break

        sdk.move_to(
            [eef[0], eef[1], max(eef[2] + 0.15, initial_cube_z + 0.16)],
            yaw=yaw,
            gripper="close",
            max_steps=min(45, remaining - 8),
            tol=0.008,
        )

        state = sdk.get_state()
        if state["objects"]["cube"]["pos"][2] >= initial_cube_z + lift_threshold:
            remaining = state["max_steps"] - state["steps_used"]
            if remaining > 0:
                sdk.set_gripper("close", steps=min(8, remaining))
            state = sdk.get_state()
            height_gain = state["objects"]["cube"]["pos"][2] - initial_cube_z
            if height_gain >= lift_threshold:
                return {
                    "status": "done",
                    "cube_height_gain": height_gain,
                    "attempts": attempts,
                    "steps_used": state["steps_used"],
                }

    state = sdk.get_state()
    height_gain = state["objects"]["cube"]["pos"][2] - initial_cube_z
    return {
        "status": "done" if height_gain >= lift_threshold else "failed",
        "cube_height_gain": height_gain,
        "attempts": attempts,
        "steps_used": state["steps_used"],
    }
