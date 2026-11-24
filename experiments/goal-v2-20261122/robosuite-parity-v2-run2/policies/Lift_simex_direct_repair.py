def run_policy(sdk):
    state = sdk.get_state()
    if "cube" not in state["objects"]:
        return {"status": "failed", "reason": "Cube observation unavailable"}

    initial_z = state["objects"]["cube"]["pos"][2]
    limit = min(400, state["max_steps"])

    for attempt in range(3):
        state = sdk.get_state()
        remaining = limit - state["steps_used"]
        if remaining < 85:
            break

        cube = state["objects"]["cube"]
        pos = cube["pos"]
        yaw = cube["yaw"]
        while yaw - state["eef_yaw"] > pi / 4:
            yaw -= pi / 2
        while yaw - state["eef_yaw"] < -pi / 4:
            yaw += pi / 2

        # Clear the object before opening or moving laterally.
        eef = state["eef_pos"]
        clearance = max(initial_z + 0.12, pos[2] + 0.10, eef[2])
        sdk.move_to(
            [eef[0], eef[1], clearance],
            gripper=None,
            max_steps=min(25, remaining - 60),
            tol=0.012,
        )
        sdk.set_gripper("open", steps=10)

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        remaining = limit - state["steps_used"]
        sdk.move_to(
            [pos[0], pos[1], max(initial_z + 0.12, pos[2] + 0.10)],
            yaw=yaw,
            gripper="open",
            max_steps=min(40, remaining - 45),
            tol=0.008,
        )

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.025:
            continue

        remaining = limit - state["steps_used"]
        if remaining < 65:
            break

        sdk.move_to(
            [pos[0], pos[1], pos[2] + 0.003],
            yaw=yaw,
            gripper="open",
            max_steps=min(40, remaining - 40),
            tol=0.005,
        )

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.018
            or abs(eef[2] - pos[2]) > 0.025
        ):
            continue

        sdk.set_gripper("close", steps=16)

        state = sdk.get_state()
        eef = state["eef_pos"]
        remaining = limit - state["steps_used"]
        if remaining < 12:
            break

        sdk.move_to(
            [eef[0], eef[1], max(initial_z + 0.20, eef[2] + 0.17)],
            yaw=yaw,
            gripper="close",
            max_steps=min(55, remaining - 6),
            tol=0.010,
        )

        state = sdk.get_state()
        if state["objects"]["cube"]["pos"][2] >= initial_z + 0.10:
            remaining = limit - state["steps_used"]
            sdk.set_gripper("close", steps=min(6, remaining))
            state = sdk.get_state()
            height_gain = state["objects"]["cube"]["pos"][2] - initial_z
            if height_gain >= 0.10:
                return {
                    "status": "done",
                    "height_gain_m": height_gain,
                    "steps_used": state["steps_used"],
                }

    state = sdk.get_state()
    return {
        "status": "failed",
        "reason": "Could not verify a sustained cube lift within the step budget",
        "height_gain_m": state["objects"]["cube"]["pos"][2] - initial_z,
        "steps_used": state["steps_used"],
    }
