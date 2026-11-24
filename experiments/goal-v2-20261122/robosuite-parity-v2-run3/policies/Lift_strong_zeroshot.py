def run_policy(sdk):
    state = sdk.get_state()
    cube = state["objects"]["cube"]
    initial_height = cube["pos"][2]
    ceiling = min(state["max_steps"], 400)

    for attempt in range(3):
        state = sdk.get_state()
        remaining = ceiling - state["steps_used"]
        if remaining < 65:
            break

        cube = state["objects"]["cube"]
        pos = cube["pos"]
        yaw = cube["yaw"]
        yaw += round((state["eef_yaw"] - yaw) / (pi / 2)) * (pi / 2)

        # Release only after raising the hand clear of the table.
        if attempt > 0:
            eef = state["eef_pos"]
            sdk.move_to(
                [eef[0], eef[1], max(eef[2], pos[2] + 0.10)],
                gripper="close",
                max_steps=min(25, remaining - 40),
            )

        state = sdk.get_state()
        remaining = ceiling - state["steps_used"]
        sdk.set_gripper("open", steps=min(12, remaining))

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        remaining = ceiling - state["steps_used"]
        if remaining < 55:
            break

        sdk.move_to(
            [pos[0], pos[1], pos[2] + 0.10],
            yaw=yaw,
            gripper="open",
            max_steps=min(45, remaining - 45),
            tol=0.008,
        )

        # Refresh the target after approaching.
        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        remaining = ceiling - state["steps_used"]
        if remaining < 45:
            break
        if hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.025:
            continue

        sdk.move_to(
            [pos[0], pos[1], pos[2] + (0.003 if attempt == 0 else 0.0)],
            yaw=yaw,
            gripper="open",
            max_steps=min(40, remaining - 35),
            tol=0.005,
        )

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        remaining = ceiling - state["steps_used"]
        if remaining < 30:
            break
        if (
            hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.018
            or abs(eef[2] - pos[2]) > 0.025
        ):
            continue

        sdk.set_gripper("close", steps=18)

        state = sdk.get_state()
        eef = state["eef_pos"]
        remaining = ceiling - state["steps_used"]
        if remaining < 10:
            break

        sdk.move_to(
            [eef[0], eef[1], max(eef[2] + 0.14, initial_height + 0.15)],
            yaw=yaw,
            gripper="close",
            max_steps=min(50, remaining - 5),
            tol=0.008,
        )

        state = sdk.get_state()
        remaining = ceiling - state["steps_used"]
        if remaining > 0:
            sdk.set_gripper("close", steps=min(10, remaining))

        state = sdk.get_state()
        lift = state["objects"]["cube"]["pos"][2] - initial_height
        if lift >= 0.08:
            return {
                "status": "done",
                "lift_height_m": lift,
                "steps_used": state["steps_used"],
            }

    state = sdk.get_state()
    lift = state["objects"]["cube"]["pos"][2] - initial_height
    return {
        "status": "done" if lift >= 0.08 else "failed",
        "lift_height_m": lift,
        "steps_used": state["steps_used"],
    }
