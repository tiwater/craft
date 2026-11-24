def run_policy(sdk):
    state = sdk.get_state()
    resting_z = state["objects"]["cube"]["pos"][2]

    for attempt in range(3):
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining < 75:
            break

        cube = state["objects"]["cube"]
        pos = cube["pos"]
        yaw = cube["yaw"]
        yaw += round((state["eef_yaw"] - yaw) / (pi / 2)) * (pi / 2)
        eef = state["eef_pos"]
        clearance = max(resting_z + 0.13, pos[2] + 0.12)

        if eef[2] < clearance - 0.015:
            sdk.move_to(
                [eef[0], eef[1], clearance],
                yaw=yaw,
                max_steps=min(40, remaining),
                tol=0.012,
            )

        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining < 65:
            break
        sdk.set_gripper("open", steps=min(12, remaining))

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        remaining = state["max_steps"] - state["steps_used"]
        sdk.move_to(
            [pos[0], pos[1], clearance],
            yaw=yaw,
            gripper="open",
            max_steps=min(45, remaining),
            tol=0.012,
        )

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.018:
            continue

        remaining = state["max_steps"] - state["steps_used"]
        if remaining < 65:
            break
        sdk.move_to(
            [pos[0], pos[1], pos[2]],
            yaw=yaw,
            gripper="open",
            max_steps=min(50, remaining - 45),
            tol=0.009,
        )

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        remaining = state["max_steps"] - state["steps_used"]

        # Judge grasp readiness from live geometry, not the reached flag.
        if (
            hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.012
            or abs(eef[2] - pos[2]) > 0.018
        ):
            if remaining < 65:
                break
            sdk.move_to(
                [pos[0], pos[1], pos[2]],
                yaw=yaw,
                gripper="open",
                max_steps=20,
                tol=0.010,
            )
            state = sdk.get_state()
            pos = state["objects"]["cube"]["pos"]
            eef = state["eef_pos"]
            if (
                hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.014
                or abs(eef[2] - pos[2]) > 0.020
            ):
                continue

        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining < 45:
            break
        sdk.set_gripper("close", steps=20)

        state = sdk.get_state()
        eef = state["eef_pos"]
        remaining = state["max_steps"] - state["steps_used"]
        sdk.move_to(
            [eef[0], eef[1], max(resting_z + 0.20, eef[2] + 0.18)],
            yaw=yaw,
            gripper="close",
            max_steps=min(60, remaining - 8),
            tol=0.012,
        )

        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if state["objects"]["cube"]["pos"][2] >= resting_z + 0.10:
            if remaining >= 8:
                sdk.set_gripper("close", steps=8)
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
