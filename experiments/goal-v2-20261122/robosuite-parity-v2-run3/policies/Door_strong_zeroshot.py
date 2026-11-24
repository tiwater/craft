def run_policy(sdk):
    def clamp(value, low, high):
        return max(low, min(high, value))

    def remaining(state):
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def geometry(state):
        handle = state["objects"]["handle"]["pos"]
        door = state["objects"]["door"]["pos"]
        rx = handle[0] - door[0]
        ry = handle[1] - door[1]
        radius = hypot(rx, ry)
        if radius < 0.025:
            quat = state["objects"]["door"]["quat"]
            x, y, z, w = quat
            rx = 1.0 - 2.0 * (y * y + z * z)
            ry = 2.0 * (x * y + w * z)
            radius = hypot(rx, ry)
        radius = max(radius, 0.001)
        return [rx / radius, ry / radius]

    state = sdk.get_state()
    initial_eef = list(state["eef_pos"])

    for attempt in range(3):
        state = sdk.get_state()
        if state["hinge_qpos"] > 0.30:
            return {"status": "done", "hinge_qpos": state["hinge_qpos"]}
        if remaining(state) < 35:
            break

        radial = geometry(state)
        yaw = atan2(radial[1], radial[0])
        if attempt == 1:
            yaw += pi / 2.0

        handle = state["objects"]["handle"]["pos"]
        sdk.move_to(
            [handle[0], handle[1], handle[2] + 0.09],
            yaw=yaw,
            gripper="open",
            max_steps=min(35, remaining(state)),
            tol=0.015,
        )
        state = sdk.get_state()
        if remaining(state) < 15:
            break

        handle = state["objects"]["handle"]["pos"]
        sdk.move_to(
            [handle[0], handle[1], handle[2] + 0.002],
            yaw=yaw,
            gripper="open",
            max_steps=min(28, remaining(state)),
            tol=0.006,
        )
        state = sdk.get_state()
        if remaining(state) < 10:
            break
        sdk.set_gripper("close", steps=min(12, remaining(state)))

        state = sdk.get_state()
        handle = state["objects"]["handle"]["pos"]
        offset = [
            clamp(state["eef_pos"][axis] - handle[axis], -0.018, 0.018)
            for axis in range(3)
        ]
        previous_handle = state["handle_qpos"]
        best_hinge = state["hinge_qpos"]
        stagnant = 0
        turn_sign = 1.0
        pull_sign = 1.0

        for tick in range(90):
            state = sdk.get_state()
            if state["hinge_qpos"] > 0.30:
                return {
                    "status": "done",
                    "hinge_qpos": state["hinge_qpos"],
                    "steps_used": state["steps_used"],
                }
            if remaining(state) < 1:
                break

            handle = state["objects"]["handle"]["pos"]
            radial = geometry(state)
            tangent = [-radial[1], radial[0], 0.0]

            if tick == 8:
                if state["handle_qpos"] < previous_handle - 0.015:
                    turn_sign = -1.0
                previous_handle = state["handle_qpos"]
            if tick == 24:
                if state["handle_qpos"] < 0.12:
                    turn_sign = -turn_sign

            if state["hinge_qpos"] > best_hinge + 0.008:
                best_hinge = state["hinge_qpos"]
                stagnant = 0
            else:
                stagnant += 1

            unlocked = state["handle_qpos"] > 0.24
            opening = state["hinge_qpos"] > 0.06
            pull = 0.032 if unlocked or opening else 0.012

            if stagnant == 38 and state["hinge_qpos"] < 0.025:
                pull_sign = -pull_sign

            direction = [
                pull_sign * tangent[0],
                pull_sign * tangent[1],
                0.0,
            ]
            if attempt == 2:
                dx = initial_eef[0] - handle[0]
                dy = initial_eef[1] - handle[1]
                distance = max(hypot(dx, dy), 0.001)
                direction = [dx / distance, dy / distance, 0.0]

            downward = 0.004 if opening else 0.018
            target = [
                handle[0] + offset[0] + pull * direction[0],
                handle[1] + offset[1] + pull * direction[1],
                handle[2] + offset[2] - downward,
            ]
            translation = [
                clamp(
                    (target[axis] - state["eef_pos"][axis]) / 0.05,
                    -0.65,
                    0.65,
                )
                for axis in range(3)
            ]

            torque = 0.14 * turn_sign
            if state["handle_qpos"] > 0.38:
                torque = 0.0
            elif unlocked:
                torque *= 0.25
            if opening:
                torque = 0.0

            sdk.step_raw(
                translation
                + [torque * tangent[0], torque * tangent[1], 0.0, 1.0],
                steps=1,
            )

            if stagnant > 60:
                break
            if tick > 20 and state["gripper_width"] < 0.009 and not opening:
                break

    state = sdk.get_state()
    return {
        "status": "done" if state["hinge_qpos"] > 0.30 else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "steps_used": state["steps_used"],
    }
