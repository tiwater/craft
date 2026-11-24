def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def opened(state):
        return state["hinge_qpos"] is not None and state["hinge_qpos"] > 0.30

    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def rotate(quat, vector):
        x, y, z, w = quat
        tx = 2.0 * (y * vector[2] - z * vector[1])
        ty = 2.0 * (z * vector[0] - x * vector[2])
        tz = 2.0 * (x * vector[1] - y * vector[0])
        return [
            vector[0] + w * tx + y * tz - z * ty,
            vector[1] + w * ty + z * tx - x * tz,
            vector[2] + w * tz + x * ty - y * tx,
        ]

    def finish():
        state = sdk.get_state()
        return {
            "status": "done" if opened(state) else "failed",
            "hinge_qpos": state["hinge_qpos"],
            "handle_qpos": state["handle_qpos"],
            "steps_used": state["steps_used"],
        }

    attempt = 0
    while remaining() > 0:
        state = sdk.get_state()
        if opened(state):
            return finish()
        if remaining() < 28:
            break

        handle = state["objects"]["handle"]
        yaw = handle["yaw"] + (attempt % 2) * pi / 2.0
        pos = handle["pos"]
        sdk.move_to(
            [pos[0], pos[1], pos[2] + 0.07],
            yaw=yaw,
            gripper="open",
            max_steps=min(22, remaining()),
            tol=0.012,
        )
        if remaining() == 0:
            break
        pos = sdk.get_state()["objects"]["handle"]["pos"]
        sdk.move_to(
            list(pos),
            yaw=yaw,
            gripper="open",
            max_steps=min(28, remaining()),
            tol=0.004,
        )
        if remaining() == 0:
            break
        sdk.set_gripper("close", steps=min(16, remaining()))

        state = sdk.get_state()
        if opened(state):
            return finish()

        handle = state["objects"]["handle"]
        offset = [state["eef_pos"][i] - handle["pos"][i] for i in range(3)]
        attempt += 1
        if sqrt(sum(value * value for value in offset)) > 0.065:
            continue

        # Store the contact offset in the moving handle frame. A fixed
        # world-space offset resists the very latch rotation being requested.
        quat = handle["quat"]
        local_offset = rotate([-quat[0], -quat[1], -quat[2], quat[3]], offset)
        direction = 1.0
        reference_latch = state["handle_qpos"]
        reference_hinge = state["hinge_qpos"]
        peak_latch = reference_latch
        peak_hinge = reference_hinge
        start_steps = state["steps_used"]
        progress_steps = start_steps
        empty_ticks = 0
        reversed_direction = False

        while remaining() > 0:
            state = sdk.get_state()
            if opened(state):
                return finish()

            elapsed = state["steps_used"] - start_steps
            handle = state["objects"]["handle"]
            door = state["objects"]["door"]
            latch = state["handle_qpos"]
            hinge = state["hinge_qpos"]

            moving_offset = rotate(handle["quat"], local_offset)
            error = [
                handle["pos"][i] + moving_offset[i] - state["eef_pos"][i]
                for i in range(3)
            ]
            separation = sqrt(sum(value * value for value in error))

            if state["gripper_width"] < 0.009:
                empty_ticks += 2
            else:
                empty_ticks = 0
            if separation > 0.11 or empty_ticks >= 18:
                break

            # Positive joint rotation is about the door's local Y axis.
            axis = rotate(door["quat"], [0.0, 1.0, 0.0])
            if (
                not reversed_direction
                and elapsed >= 12
                and latch < reference_latch - 0.025
            ):
                direction = -direction
                reversed_direction = True

            rx = handle["pos"][0] - door["pos"][0]
            ry = handle["pos"][1] - door["pos"][1]
            radius = hypot(rx, ry)
            if radius < 0.04:
                radial = rotate(door["quat"], [1.0, 0.0, 0.0])
                rx, ry = radial[0], radial[1]
                radius = max(hypot(rx, ry), 0.000001)
            tangent = [-ry / radius, rx / radius, 0.0]

            # Keep opening preload small until the latch clears, so contact
            # pressure against the latch stop does not prevent rotation.
            lead = 0.004
            if latch > 0.30:
                lead = 0.018
            if latch > 0.48 or hinge > 0.035:
                lead = 0.055

            torque = 0.85
            if latch > 0.65:
                torque = 0.22
            if latch > 0.95:
                torque = -0.10

            # A compliant position servo lets the grasp follow the lever arc.
            translation = [
                clamp(5.0 * error[i] + 8.0 * lead * tangent[i], 0.65)
                for i in range(3)
            ]
            rotation = [direction * torque * value for value in axis]
            if hinge > 0.035:
                rotation[2] += 0.16

            sdk.step_raw(
                translation + rotation + [1.0],
                steps=min(2, remaining()),
            )
            after = sdk.get_state()
            if opened(after):
                return finish()

            if (
                after["handle_qpos"] > peak_latch + 0.012
                or after["hinge_qpos"] > peak_hinge + 0.003
            ):
                peak_latch = max(peak_latch, after["handle_qpos"])
                peak_hinge = max(peak_hinge, after["hinge_qpos"])
                progress_steps = after["steps_used"]

            stalled_steps = after["steps_used"] - progress_steps
            # Reserve time for another grasp only when there is enough
            # budget to complete it; otherwise keep working the current grip.
            if stalled_steps >= 64 and remaining() >= 85:
                break
            if elapsed >= 150 and remaining() >= 100:
                break

    return finish()
