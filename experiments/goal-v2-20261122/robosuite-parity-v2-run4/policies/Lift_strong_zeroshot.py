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
        if state["max_steps"] - state["steps_used"] < 65:
            break

        cube = state["objects"]["cube"]
        pos = cube["pos"]
        yaw = cube["yaw"]
        yaw += round((state["eef_yaw"] - yaw) / (pi / 2)) * (pi / 2)

        # Retreat vertically before opening or moving sideways.
        clearance = max(resting_z + 0.14, pos[2] + 0.12)
        eef = state["eef_pos"]
        if not lift_move(
            sdk, [eef[0], eef[1], clearance], yaw, None, 35
        ):
            continue

        lift_gripper(sdk, "open", 12)
        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        if not lift_move(
            sdk, [pos[0], pos[1], clearance], yaw, "open", 40
        ):
            continue

        # Refresh the target after approach in case contact moved the cube.
        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        grasp_z = pos[2] + (0.005 if attempt == 0 else 0.0)
        if not lift_move(
            sdk, [pos[0], pos[1], grasp_z], yaw, "open", 40, 0.005
        ):
            continue

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if hypot(pos[0] - eef[0], pos[1] - eef[1]) > 0.008:
            if not lift_move(
                sdk, [pos[0], pos[1], grasp_z], yaw, "open", 15, 0.005
            ):
                continue

        lift_gripper(sdk, "close", 20)
        state = sdk.get_state()
        eef = state["eef_pos"]

        # Lift vertically, maintaining closure throughout the motion.
        lift_move(
            sdk,
            [eef[0], eef[1], max(resting_z + 0.20, eef[2] + 0.16)],
            yaw,
            "close",
            45,
        )

        state = sdk.get_state()
        if state["objects"]["cube"]["pos"][2] >= resting_z + 0.10:
            # Confirm sustained elevation rather than a transient bounce.
            if state["max_steps"] - state["steps_used"] < 10:
                break
            lift_gripper(sdk, "close", 10)
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
