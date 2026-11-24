def clamp(value, low, high):
    return max(low, min(high, value))


def remaining_steps(state):
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def orientation_control(target, current):
    tx, ty, tz, tw = target
    cx, cy, cz, cw = current
    error = [
        -tw * cx + tx * cw - ty * cz + tz * cy,
        -tw * cy + tx * cz + ty * cw - tz * cx,
        -tw * cz - tx * cy + ty * cx + tz * cw,
        tw * cw + tx * cx + ty * cy + tz * cz,
    ]
    sign = 1.0 if error[3] >= 0.0 else -1.0
    return [clamp(3.0 * sign * error[i], -0.45, 0.45) for i in range(3)]


def run_policy(sdk):
    state = sdk.get_state()

    for attempt in range(2):
        if state["hinge_qpos"] > 0.30:
            break
        if remaining_steps(state) < 55:
            break

        handle = state["objects"]["handle"]
        position = list(handle["pos"])
        yaw = handle["yaw"] + attempt * pi / 2.0

        sdk.move_to(
            [position[0], position[1] + 0.085, position[2] + 0.035],
            yaw=yaw,
            gripper="open",
            max_steps=min(30, remaining_steps(state)),
            tol=0.012,
        )
        state = sdk.get_state()
        if remaining_steps(state) < 15:
            break

        position = state["objects"]["handle"]["pos"]
        sdk.move_to(
            [position[0], position[1] + 0.006, position[2] + 0.006],
            yaw=yaw,
            gripper="open",
            max_steps=min(28, remaining_steps(state)),
            tol=0.007,
        )
        state = sdk.get_state()
        if remaining_steps(state) < 10:
            break

        sdk.set_gripper("close", steps=min(12, remaining_steps(state)))
        state = sdk.get_state()
        held_orientation = list(state["eef_quat"])
        handle = state["objects"]["handle"]
        contact_x = clamp(state["eef_pos"][0] - handle["pos"][0], -0.018, 0.018)
        contact_y = clamp(state["eef_pos"][1] - handle["pos"][1], -0.012, 0.018)

        best_angle = state["handle_qpos"]
        stalled = 0
        released = False
        pull_ticks = 0
        previous_hinge = state["hinge_qpos"]
        previous_position = list(handle["pos"])
        direction = [0.55, 0.835]
        lead = 0.0

        for tick in range(245):
            if remaining_steps(state) <= 0 or state["hinge_qpos"] > 0.30:
                break

            angle = state["handle_qpos"]
            hinge = state["hinge_qpos"]
            handle = state["objects"]["handle"]
            position = handle["pos"]

            if angle > best_angle + 0.008:
                best_angle = angle
                stalled = 0
            else:
                stalled += 1

            if angle >= 1.50:
                released = True

            # Pause horizontal loading if the latch starts to spring back.
            pulling = released and (angle >= 1.45 or hinge > 0.08)
            if pulling:
                lead = min(0.055, lead + 0.0015)
                pull_ticks += 1
            else:
                lead = max(0.0, lead - 0.004)

            # Follow the live bar while loading it deeply downward.
            depth = 0.035 + min(0.035, 0.00055 * stalled)
            if angle >= 1.20:
                depth = max(depth, 0.055)
            if pulling:
                depth = max(depth, 0.045)

            target = [
                position[0] + contact_x + lead * direction[0],
                position[1] + contact_y + lead * direction[1],
                position[2] - depth,
            ]
            translation = [
                clamp(24.0 * (target[0] - state["eef_pos"][0]), -0.65, 0.65),
                clamp(24.0 * (target[1] - state["eef_pos"][1]), -0.65, 0.65),
                clamp(26.0 * (target[2] - state["eef_pos"][2]), -0.95, 0.35),
            ]
            rotation = orientation_control(held_orientation, state["eef_quat"])
            sdk.step_raw(translation + rotation + [1.0], steps=1)
            state = sdk.get_state()

            # Blend toward the observed opening arc once motion is measurable.
            if tick % 8 == 7:
                current_position = state["objects"]["handle"]["pos"]
                dx = current_position[0] - previous_position[0]
                dy = current_position[1] - previous_position[1]
                distance = hypot(dx, dy)
                if (
                    pulling
                    and state["hinge_qpos"] - previous_hinge > 0.004
                    and distance > 0.0015
                    and dx > 0.0
                    and dy > 0.0
                ):
                    nx = 0.7 * direction[0] + 0.3 * dx / distance
                    ny = 0.7 * direction[1] + 0.3 * dy / distance
                    norm = max(hypot(nx, ny), 0.000001)
                    direction = [nx / norm, ny / norm]
                previous_position = list(current_position)
                previous_hinge = state["hinge_qpos"]

            separation = sqrt(
                sum(
                    (state["eef_pos"][i] - state["objects"]["handle"]["pos"][i]) ** 2
                    for i in range(3)
                )
            )
            if separation > 0.115:
                break
            if not released and stalled > 65 and tick > 90:
                break
            if pull_ticks > 100 and state["hinge_qpos"] < 0.035:
                break

        if state["hinge_qpos"] > 0.30:
            break

    state = sdk.get_state()
    return {
        "status": "done" if state["hinge_qpos"] > 0.30 else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "steps_used": state["steps_used"],
    }
