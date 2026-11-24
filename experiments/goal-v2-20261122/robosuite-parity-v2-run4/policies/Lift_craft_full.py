def lift_move(sdk, pos, yaw, gripper, limit, tol=0.008):
    state = sdk.get_state()
    remaining = max(0, state["max_steps"] - state["steps_used"])
    if remaining == 0:
        return False
    result = sdk.move_to(
        pos,
        yaw=yaw,
        gripper=gripper,
        max_steps=min(limit, remaining),
        tol=tol,
    )
    return result["reached"]


def lift_gripper(sdk, action, limit):
    state = sdk.get_state()
    remaining = max(0, state["max_steps"] - state["steps_used"])
    if remaining > 0:
        sdk.set_gripper(action, steps=min(limit, remaining))


def run_policy(sdk):
    state = sdk.get_state()
    resting_z = state["objects"]["cube"]["pos"][2]

    for attempt in range(3):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 100:
            break

        cube = state["objects"]["cube"]
        pos = cube["pos"]
        yaw = cube["yaw"]
        yaw += round((state["eef_yaw"] - yaw) / (pi / 2)) * (pi / 2)

        # Preserve the vertical retreat before opening or moving sideways.
        clearance = max(resting_z + 0.14, pos[2] + 0.12)
        eef = state["eef_pos"]
        if not lift_move(
            sdk, [eef[0], eef[1], clearance], yaw, None, 40, 0.012
        ):
            continue

        lift_gripper(sdk, "open", 12)
        state = sdk.get_state()
        if state["gripper_width"] < 0.075:
            lift_gripper(sdk, "open", 8)

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        if not lift_move(
            sdk, [pos[0], pos[1], clearance], yaw, "open", 40
        ):
            continue

        state = sdk.get_state()
        if state["gripper_width"] < 0.075:
            lift_gripper(sdk, "open", 8)

        # Refresh both position and orientation immediately before descent.
        state = sdk.get_state()
        if state["gripper_width"] < 0.075:
            continue
        cube = state["objects"]["cube"]
        pos = cube["pos"]
        yaw = cube["yaw"]
        yaw += round((state["eef_yaw"] - yaw) / (pi / 2)) * (pi / 2)

        # The traces show a 6-7 mm residual, so a 5 mm reached gate
        # incorrectly skipped every closure. Target the live center and
        # evaluate the physical alignment independently of that flag.
        lift_move(
            sdk, [pos[0], pos[1], pos[2]], yaw, "open", 40, 0.008
        )

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if (
            hypot(pos[0] - eef[0], pos[1] - eef[1]) > 0.006
            or abs(pos[2] - eef[2]) > 0.010
        ):
            lift_move(
                sdk, [pos[0], pos[1], pos[2]], yaw, "open", 15, 0.008
            )

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if (
            hypot(pos[0] - eef[0], pos[1] - eef[1]) > 0.008
            or abs(pos[2] - eef[2]) > 0.012
            or state["gripper_width"] < 0.075
        ):
            continue
        if state["max_steps"] - state["steps_used"] < 85:
            break

        grasp_cube_z = pos[2]
        grasp_eef_z = eef[2]
        lift_gripper(sdk, "close", 20)

        state = sdk.get_state()
        if not 0.015 <= state["gripper_width"] <= 0.065:
            continue

        # Verify that the cube follows a short vertical lift before
        # committing to the full lift.
        eef = state["eef_pos"]
        lift_move(
            sdk,
            [eef[0], eef[1], eef[2] + 0.06],
            yaw,
            "close",
            25,
        )

        state = sdk.get_state()
        cube_z = state["objects"]["cube"]["pos"][2]
        eef_rise = state["eef_pos"][2] - grasp_eef_z
        cube_rise = cube_z - grasp_cube_z
        if (
            not 0.015 <= state["gripper_width"] <= 0.065
            or cube_rise < 0.025
            or eef_rise < 0.025
            or abs(cube_rise - eef_rise) > 0.025
        ):
            continue

        eef = state["eef_pos"]
        lift_move(
            sdk,
            [eef[0], eef[1], max(resting_z + 0.20, grasp_eef_z + 0.16)],
            yaw,
            "close",
            30,
        )

        state = sdk.get_state()
        if (
            state["objects"]["cube"]["pos"][2] >= resting_z + 0.10
            and 0.015 <= state["gripper_width"] <= 0.065
        ):
            if state["max_steps"] - state["steps_used"] < 10:
                break
            lift_gripper(sdk, "close", 10)
            state = sdk.get_state()
            height = state["objects"]["cube"]["pos"][2] - resting_z
            if height >= 0.10 and 0.015 <= state["gripper_width"] <= 0.065:
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
