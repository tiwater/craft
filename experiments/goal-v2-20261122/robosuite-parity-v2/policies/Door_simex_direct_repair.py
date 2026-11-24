def clamp(value, low, high):
    return max(low, min(high, value))


def quat_multiply(a, b):
    return [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]


def quat_inverse(q):
    scale = max(sum(v * v for v in q), 0.000001)
    return [-q[0] / scale, -q[1] / scale, -q[2] / scale, q[3] / scale]


def quat_rotate(q, vector):
    return quat_multiply(
        quat_multiply(q, list(vector) + [0.0]), quat_inverse(q)
    )[:3]


def remaining_steps(state):
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def orientation_action(target, current):
    error = quat_multiply(target, quat_inverse(current))
    sign = 1.0 if error[3] >= 0.0 else -1.0
    return [clamp(8.0 * sign * error[i], -0.8, 0.8) for i in range(3)]


def run_policy(sdk):
    state = sdk.get_state()
    attempt = 0

    while remaining_steps(state) > 32 and state["hinge_qpos"] <= 0.30:
        handle = state["objects"]["handle"]
        yaw = handle["yaw"] + (pi / 2.0 if attempt % 2 else 0.0)
        position = list(handle["pos"])
        sdk.move_to(
            [position[0], position[1], position[2] + 0.075],
            yaw=yaw,
            gripper="open",
            max_steps=min(22, remaining_steps(state)),
            tol=0.012,
        )
        state = sdk.get_state()
        if remaining_steps(state) <= 0:
            break
        sdk.move_to(
            list(state["objects"]["handle"]["pos"]),
            yaw=yaw,
            gripper="open",
            max_steps=min(26, remaining_steps(state)),
            tol=0.005,
        )
        state = sdk.get_state()
        if remaining_steps(state) <= 0:
            break
        sdk.set_gripper("close", steps=min(12, remaining_steps(state)))
        state = sdk.get_state()
        attempt += 1
        if state["gripper_width"] < 0.008:
            continue

        handle = state["objects"]["handle"]
        local_offset = quat_rotate(
            quat_inverse(handle["quat"]),
            [state["eef_pos"][i] - handle["pos"][i] for i in range(3)],
        )
        axes = [
            quat_rotate(handle["quat"], [1.0, 0.0, 0.0]),
            quat_rotate(handle["quat"], [0.0, 1.0, 0.0]),
            quat_rotate(handle["quat"], [0.0, 0.0, 1.0]),
        ]
        best_axis = None
        best_gain = 0.001

        for axis in axes:
            if state["handle_qpos"] >= 0.95 or remaining_steps(state) < 90:
                break
            for sign in [1.0, -1.0]:
                before = state["handle_qpos"]
                direction = [sign * component for component in axis]
                for tick in range(6):
                    if remaining_steps(state) <= 0:
                        break
                    handle = state["objects"]["handle"]
                    offset = quat_rotate(handle["quat"], local_offset)
                    translation = [
                        clamp(
                            35.0 * (handle["pos"][i] + offset[i] - state["eef_pos"][i]),
                            -0.7,
                            0.7,
                        )
                        for i in range(3)
                    ]
                    sdk.step_raw(translation + [0.65 * v for v in direction] + [1.0])
                    state = sdk.get_state()
                gain = state["handle_qpos"] - before
                if gain > best_gain:
                    best_gain = gain
                    best_axis = list(direction)
                if gain > 0.015:
                    break
                for tick in range(6):
                    if remaining_steps(state) <= 0:
                        break
                    handle = state["objects"]["handle"]
                    offset = quat_rotate(handle["quat"], local_offset)
                    translation = [
                        clamp(
                            35.0 * (handle["pos"][i] + offset[i] - state["eef_pos"][i]),
                            -0.7,
                            0.7,
                        )
                        for i in range(3)
                    ]
                    sdk.step_raw(translation + [-0.65 * v for v in direction] + [1.0])
                    state = sdk.get_state()
            if best_gain > 0.015:
                break

        if best_axis is not None:
            for tick in range(48):
                if state["handle_qpos"] >= 1.05 or remaining_steps(state) < 75:
                    break
                handle = state["objects"]["handle"]
                offset = quat_rotate(handle["quat"], local_offset)
                translation = [
                    clamp(
                        35.0 * (handle["pos"][i] + offset[i] - state["eef_pos"][i]),
                        -0.7,
                        0.7,
                    )
                    for i in range(3)
                ]
                sdk.step_raw(translation + [0.65 * v for v in best_axis] + [1.0])
                state = sdk.get_state()

        reference_hinge = state["hinge_qpos"]
        reference_latch = state["handle_qpos"]
        reference_orientation = list(state["eef_quat"])
        handle = state["objects"]["handle"]
        local_offset = quat_rotate(
            quat_inverse(handle["quat"]),
            [state["eef_pos"][i] - handle["pos"][i] for i in range(3)],
        )
        door = state["objects"]["door"]
        radial = [
            handle["pos"][0] - door["pos"][0],
            handle["pos"][1] - door["pos"][1],
        ]
        base_angle = atan2(radial[1], radial[0]) + pi / 2.0
        candidates = [
            base_angle,
            base_angle + pi,
            base_angle + pi / 2.0,
            base_angle - pi / 2.0,
            base_angle + pi / 4.0,
            base_angle - pi / 4.0,
            base_angle + 3.0 * pi / 4.0,
            base_angle - 3.0 * pi / 4.0,
        ]
        candidate = 0
        direction_angle = candidates[0]
        direction_hinge = state["hinge_qpos"]
        window_start = state["hinge_qpos"]
        window_ticks = 0
        best_open_gain = -1.0
        best_open_angle = direction_angle
        best_open_hinge = direction_hinge
        lost_ticks = 0
        sweep = 0

        while remaining_steps(state) > 0 and state["hinge_qpos"] <= 0.30:
            if window_ticks >= 18:
                gain = state["hinge_qpos"] - window_start
                if gain > best_open_gain:
                    best_open_gain = gain
                    best_open_angle = direction_angle
                    best_open_hinge = direction_hinge
                if gain < 0.008:
                    candidate += 1
                    if candidate >= len(candidates):
                        sweep += 1
                        candidate = 0
                        direction_angle = best_open_angle
                        direction_hinge = best_open_hinge
                        if sweep > 1 and best_open_gain < 0.002:
                            break
                    else:
                        direction_angle = candidates[candidate]
                        direction_hinge = reference_hinge
                window_start = state["hinge_qpos"]
                window_ticks = 0

            hinge_change = state["hinge_qpos"] - reference_hinge
            hinge_rotation = [
                0.0, 0.0, sin(hinge_change / 2.0), cos(hinge_change / 2.0)
            ]
            target_orientation = quat_multiply(hinge_rotation, reference_orientation)

            if best_axis is not None:
                correction = clamp(1.10 - reference_latch, -0.15, 0.50)
                axis = quat_rotate(hinge_rotation, best_axis)
                latch_rotation = [
                    axis[0] * sin(correction / 2.0),
                    axis[1] * sin(correction / 2.0),
                    axis[2] * sin(correction / 2.0),
                    cos(correction / 2.0),
                ]
                target_orientation = quat_multiply(latch_rotation, target_orientation)

            handle = state["objects"]["handle"]
            offset = quat_rotate(handle["quat"], local_offset)
            angle = direction_angle + state["hinge_qpos"] - direction_hinge
            lead = (0.070 if sweep == 0 else 0.090)
            lead *= min(1.0, (window_ticks + 1) / 5.0)
            if state["handle_qpos"] < 0.65 and state["hinge_qpos"] < 0.04:
                lead *= 0.4
            direction = [cos(angle), sin(angle), 0.0]
            target = [
                handle["pos"][i] + offset[i] + lead * direction[i]
                for i in range(3)
            ]
            translation = [
                clamp(40.0 * (target[i] - state["eef_pos"][i]), -1.0, 1.0)
                for i in range(3)
            ]
            sdk.step_raw(
                translation
                + orientation_action(target_orientation, state["eef_quat"])
                + [1.0]
            )
            state = sdk.get_state()
            window_ticks += 1

            handle = state["objects"]["handle"]
            offset = quat_rotate(handle["quat"], local_offset)
            tracking_error = sqrt(sum(
                (state["eef_pos"][i] - handle["pos"][i] - offset[i]) ** 2
                for i in range(3)
            ))
            if state["gripper_width"] < 0.007 or tracking_error > 0.105:
                lost_ticks += 1
            else:
                lost_ticks = 0
            if lost_ticks >= 5:
                break

    state = sdk.get_state()
    return {
        "status": "done" if state["hinge_qpos"] > 0.30 else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "steps_used": state["steps_used"],
    }
