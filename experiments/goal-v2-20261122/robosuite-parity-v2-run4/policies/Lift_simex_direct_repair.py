def run_policy(sdk):
    def move(pos, yaw, gripper, limit, tol=0.012):
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining > 0:
            sdk.move_to(
                pos,
                yaw=yaw,
                gripper=gripper,
                max_steps=min(limit, remaining),
                tol=tol,
            )

    def grip(action, limit):
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining > 0:
            sdk.set_gripper(action, steps=min(limit, remaining))

    state = sdk.get_state()
    resting_z = state["objects"]["cube"]["pos"][2]

    for attempt in range(3):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 80:
            break

        cube = state["objects"]["cube"]
        pos = cube["pos"]
        yaw = cube["yaw"]
        yaw += round((state["eef_yaw"] - yaw) / (pi / 2)) * (pi / 2)
        eef = state["eef_pos"]
        clearance = max(resting_z + 0.12, pos[2] + 0.10)

        move([eef[0], eef[1], clearance], yaw, None, 25)
        grip("open", 10)

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        move([pos[0], pos[1], clearance], yaw, "open", 30)

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if hypot(pos[0] - eef[0], pos[1] - eef[1]) > 0.025:
            continue

        # Stop slightly above the center to avoid table contact.
        grasp_offset = 0.012 if attempt == 0 else 0.006
        move(
            [pos[0], pos[1], pos[2] + grasp_offset],
            yaw,
            "open",
            35,
            0.010,
        )

        # Judge the actual grasp geometry, not the move's reached flag.
        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if (
            hypot(pos[0] - eef[0], pos[1] - eef[1]) > 0.012
            or eef[2] - pos[2] > 0.035
        ):
            move(
                [pos[0], pos[1], pos[2] + grasp_offset],
                yaw,
                "open",
                15,
                0.010,
            )

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if (
            hypot(pos[0] - eef[0], pos[1] - eef[1]) > 0.020
            or eef[2] - pos[2] > 0.040
            or eef[2] < pos[2] - 0.025
        ):
            continue

        grip("close", 20)
        state = sdk.get_state()
        eef = state["eef_pos"]

        move(
            [eef[0], eef[1], eef[2] + 0.065],
            yaw,
            "close",
            22,
        )

        state = sdk.get_state()
        height = state["objects"]["cube"]["pos"][2] - resting_z
        if height < 0.025:
            continue

        eef = state["eef_pos"]
        move(
            [eef[0], eef[1], max(resting_z + 0.20, eef[2] + 0.12)],
            yaw,
            "close",
            35,
        )

        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 10:
            break

        grip("close", 10)
        state = sdk.get_state()
        height = state["objects"]["cube"]["pos"][2] - resting_z
        if height >= 0.10:
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
