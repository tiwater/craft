def run_policy(sdk):
    state = sdk.get_state()
    initial_cube_z = state["objects"]["cube"]["pos"][2]
    success_z = initial_cube_z + 0.08
    limit = min(400, state["max_steps"])

    for attempt in range(3):
        state = sdk.get_state()
        remaining = limit - state["steps_used"]
        if remaining < 45:
            break

        cube = state["objects"]["cube"]
        pos = cube["pos"]
        yaw = cube["yaw"]
        current_yaw = state["eef_yaw"]
        while yaw - current_yaw > pi / 4:
            yaw -= pi / 2
        while yaw - current_yaw < -pi / 4:
            yaw += pi / 2

        sdk.set_gripper("open", steps=min(10, remaining))

        state = sdk.get_state()
        remaining = limit - state["steps_used"]
        if remaining < 35:
            break

        # Clear the cube before moving sideways.
        clearance = max(state["eef_pos"][2], pos[2] + 0.10)
        sdk.move_to(
            [state["eef_pos"][0], state["eef_pos"][1], clearance],
            gripper="open",
            max_steps=min(25, remaining - 25),
            tol=0.008,
        )

        state = sdk.get_state()
        remaining = limit - state["steps_used"]
        if remaining < 30:
            break
        pos = state["objects"]["cube"]["pos"]
        sdk.move_to(
            [pos[0], pos[1], pos[2] + 0.10],
            yaw=yaw,
            gripper="open",
            max_steps=min(40, remaining - 20),
            tol=0.006,
        )

        state = sdk.get_state()
        remaining = limit - state["steps_used"]
        if remaining < 65:
            break
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.015:
            continue

        sdk.move_to(
            [pos[0], pos[1], pos[2] + 0.002],
            yaw=yaw,
            gripper="open",
            max_steps=min(40, remaining - 45),
            tol=0.004,
        )

        state = sdk.get_state()
        remaining = limit - state["steps_used"]
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if remaining < 40:
            break
        if (
            hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.015
            or abs(eef[2] - pos[2]) > 0.025
        ):
            continue

        sdk.set_gripper("close", steps=18)

        state = sdk.get_state()
        remaining = limit - state["steps_used"]
        if remaining < 2:
            break

        # Keep the hand's horizontal position fixed during the lift.
        eef = state["eef_pos"]
        cube_z = state["objects"]["cube"]["pos"][2]
        target_z = eef[2] + max(0.12, success_z + 0.04 - cube_z)
        sdk.move_to(
            [eef[0], eef[1], target_z],
            yaw=yaw,
            gripper="close",
            max_steps=min(55, remaining),
            tol=0.006,
        )

        state = sdk.get_state()
        if state["objects"]["cube"]["pos"][2] >= success_z:
            remaining = limit - state["steps_used"]
            if remaining > 0:
                sdk.set_gripper("close", steps=min(10, remaining))
            state = sdk.get_state()
            if state["objects"]["cube"]["pos"][2] >= success_z:
                return {
                    "status": "done",
                    "lift_height": state["objects"]["cube"]["pos"][2] - initial_cube_z,
                    "steps_used": state["steps_used"],
                }

    state = sdk.get_state()
    lifted = state["objects"]["cube"]["pos"][2] - initial_cube_z
    return {
        "status": "done" if lifted >= 0.08 else "failed",
        "lift_height": lifted,
        "steps_used": state["steps_used"],
    }
