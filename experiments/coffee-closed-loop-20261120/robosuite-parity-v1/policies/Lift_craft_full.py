def run_policy(sdk):
    state = sdk.get_state()
    initial_cube_z = state["objects"]["cube"]["pos"][2]
    target_cube_z = initial_cube_z + 0.08
    limit = min(400, state["max_steps"])

    for attempt in range(3):
        state = sdk.get_state()
        remaining = limit - state["steps_used"]
        if remaining < 90:
            break

        cube = state["objects"]["cube"]
        pos = cube["pos"]
        yaw = cube["yaw"]
        current_yaw = state["eef_yaw"]
        while yaw - current_yaw > pi / 4:
            yaw -= pi / 2
        while yaw - current_yaw < -pi / 4:
            yaw += pi / 2

        # Clear the cube before moving sideways.
        eef = state["eef_pos"]
        hover_z = max(pos[2] + 0.12, eef[2])
        sdk.move_to(
            [eef[0], eef[1], hover_z],
            gripper="open",
            max_steps=min(25, remaining - 65),
            tol=0.01,
        )

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        remaining = limit - state["steps_used"]
        sdk.move_to(
            [pos[0], pos[1], pos[2] + 0.12],
            yaw=yaw,
            gripper="open",
            max_steps=min(45, remaining - 60),
            tol=0.006,
        )

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.015:
            continue

        remaining = limit - state["steps_used"]
        if remaining < 65:
            break

        sdk.move_to(
            [pos[0], pos[1], pos[2] + 0.002],
            yaw=yaw,
            gripper="open",
            max_steps=min(45, remaining - 45),
            tol=0.004,
        )

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.012
            or abs(eef[2] - (pos[2] + 0.002)) > 0.015
        ):
            continue

        sdk.set_gripper("close", steps=20)

        state = sdk.get_state()
        remaining = limit - state["steps_used"]
        if remaining < 15:
            break

        eef = state["eef_pos"]
        cube_z = state["objects"]["cube"]["pos"][2]
        lift_distance = max(0.12, target_cube_z - cube_z + 0.035)
        sdk.move_to(
            [eef[0], eef[1], eef[2] + lift_distance],
            yaw=yaw,
            gripper="close",
            max_steps=min(55, remaining - 10),
            tol=0.008,
        )
        sdk.set_gripper("close", steps=10)

        state = sdk.get_state()
        cube_z = state["objects"]["cube"]["pos"][2]
        if cube_z >= target_cube_z:
            return {
                "status": "done",
                "cube_z": cube_z,
                "lift_height": cube_z - initial_cube_z,
                "steps_used": state["steps_used"],
            }

    state = sdk.get_state()
    cube_z = state["objects"]["cube"]["pos"][2]
    return {
        "status": "done" if cube_z >= target_cube_z else "failed",
        "cube_z": cube_z,
        "lift_height": cube_z - initial_cube_z,
        "steps_used": state["steps_used"],
    }
