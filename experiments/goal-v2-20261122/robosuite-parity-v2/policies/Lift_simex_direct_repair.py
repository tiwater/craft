def run_policy(sdk):
    state = sdk.get_state()
    initial_cube_z = state["objects"]["cube"]["pos"][2]
    limit = min(400, state["max_steps"])
    attempts = 0

    def remaining():
        return max(0, limit - sdk.get_state()["steps_used"])

    def move(pos, yaw, gripper, steps, tol):
        budget = min(steps, remaining())
        if budget > 0:
            return sdk.move_to(
                pos, yaw=yaw, gripper=gripper,
                max_steps=budget, tol=tol
            )
        return {"reached": False}

    def actuate(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    for attempt in range(3):
        if remaining() < 65:
            break

        attempts += 1
        state = sdk.get_state()
        cube = state["objects"]["cube"]
        pos = cube["pos"]
        yaw = cube["yaw"]
        current_yaw = state["eef_yaw"]

        # A square cube permits equivalent grasps every quarter turn.
        while yaw - current_yaw > pi / 4:
            yaw -= pi / 2
        while yaw - current_yaw < -pi / 4:
            yaw += pi / 2

        # Retreat vertically before opening or moving laterally.
        eef = state["eef_pos"]
        clearance = max(eef[2], pos[2] + 0.11)
        move([eef[0], eef[1], clearance], None, None, 25, 0.012)
        actuate("open", 12)

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        move([pos[0], pos[1], pos[2] + 0.11],
             yaw, "open", 40, 0.008)

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        lateral_error = hypot(eef[0] - pos[0], eef[1] - pos[1])
        if lateral_error > 0.018:
            continue

        # Reobserve after approaching to accommodate any cube motion.
        grasp_offset = 0.002 if attempt != 1 else -0.003
        move([pos[0], pos[1], pos[2] + grasp_offset],
             yaw, "open", 40, 0.005)

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if (hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.014
                or abs(eef[2] - pos[2] - grasp_offset) > 0.018):
            continue
        if remaining() < 40:
            break

        actuate("close", 20)

        state = sdk.get_state()
        eef = state["eef_pos"]
        lift_z = max(eef[2] + 0.14, initial_cube_z + 0.15)
        move([eef[0], eef[1], lift_z],
             yaw, "close", 55, 0.008)

        state = sdk.get_state()
        cube_z = state["objects"]["cube"]["pos"][2]
        if cube_z - initial_cube_z >= 0.08 and remaining() >= 10:
            # Verify the cube stays elevated while the gripper holds still.
            actuate("close", 10)
            state = sdk.get_state()
            cube_z = state["objects"]["cube"]["pos"][2]
            if cube_z - initial_cube_z >= 0.08:
                return {
                    "status": "done",
                    "lift_height_m": cube_z - initial_cube_z,
                    "attempts": attempts,
                    "steps_used": state["steps_used"],
                }

    state = sdk.get_state()
    return {
        "status": "failed",
        "reason": "Could not verify a sustained cube lift within the step budget.",
        "lift_height_m": state["objects"]["cube"]["pos"][2] - initial_cube_z,
        "attempts": attempts,
        "steps_used": state["steps_used"],
    }
