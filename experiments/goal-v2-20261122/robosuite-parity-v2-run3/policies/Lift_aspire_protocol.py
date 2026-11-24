def run_policy(sdk):
    state = sdk.get_state()
    if "cube" not in state["objects"]:
        return {"status": "failed", "reason": "Cube observation unavailable"}

    initial_cube_z = state["objects"]["cube"]["pos"][2]
    attempts = 0

    for attempt in range(3):
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining < 130:
            break

        attempts = attempt + 1
        cube = state["objects"]["cube"]
        pos = cube["pos"]
        yaw = cube["yaw"]
        current_yaw = state["eef_yaw"]

        # A square cube permits equivalent grasps every quarter turn.
        while yaw - current_yaw > pi / 4:
            yaw -= pi / 2
        while yaw - current_yaw < -pi / 4:
            yaw += pi / 2

        # Clear the cube before translating or opening on a retry.
        clearance = max(state["eef_pos"][2], pos[2] + 0.12)
        sdk.move_to(
            [state["eef_pos"][0], state["eef_pos"][1], clearance],
            gripper=None,
            max_steps=20,
            tol=0.012,
        )

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        sdk.move_to(
            [pos[0], pos[1], pos[2] + 0.12],
            yaw=yaw,
            gripper="open",
            max_steps=35,
            tol=0.008,
        )

        # Refresh the target after the approach in case the cube moved.
        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        approach_error = hypot(
            state["eef_pos"][0] - pos[0],
            state["eef_pos"][1] - pos[1],
        )
        if approach_error > 0.025:
            continue

        sdk.move_to(
            [pos[0], pos[1], pos[2] + 0.003],
            yaw=yaw,
            gripper="open",
            max_steps=30,
            tol=0.006,
        )

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.018
            or abs(eef[2] - pos[2]) > 0.025
        ):
            continue

        sdk.set_gripper("close", steps=18)

        state = sdk.get_state()
        eef = state["eef_pos"]
        remaining = state["max_steps"] - state["steps_used"]
        if remaining < 15:
            break

        sdk.move_to(
            [eef[0], eef[1], eef[2] + 0.18],
            yaw=yaw,
            gripper="close",
            max_steps=min(45, remaining - 8),
            tol=0.01,
        )
        sdk.set_gripper("close", steps=8)

        state = sdk.get_state()
        cube_pos = state["objects"]["cube"]["pos"]
        lift_height = cube_pos[2] - initial_cube_z

        # Verify actual object motion, rather than gripper motion alone.
        if lift_height >= 0.10:
            return {
                "status": "done",
                "lift_height": lift_height,
                "attempts": attempts,
                "steps_used": state["steps_used"],
            }

    state = sdk.get_state()
    lift_height = state["objects"]["cube"]["pos"][2] - initial_cube_z
    return {
        "status": "done" if lift_height >= 0.10 else "failed",
        "lift_height": lift_height,
        "attempts": attempts,
        "steps_used": state["steps_used"],
    }
