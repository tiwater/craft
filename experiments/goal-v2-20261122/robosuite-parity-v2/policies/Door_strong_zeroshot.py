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
    result = quat_multiply(
        quat_multiply(q, [vector[0], vector[1], vector[2], 0.0]),
        quat_inverse(q),
    )
    return result[:3]


def unit_horizontal(vector):
    length = hypot(vector[0], vector[1])
    if length < 0.00001:
        return [1.0, 0.0, 0.0]
    return [vector[0] / length, vector[1] / length, 0.0]


def remaining_steps(state):
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def run_policy(sdk):
    state = sdk.get_state()
    if state["hinge_qpos"] > 0.30:
        return {"status": "done", "hinge_qpos": state["hinge_qpos"]}

    start_position = list(state["eef_pos"])
    best_hinge = state["hinge_qpos"]

    for attempt in range(2):
        state = sdk.get_state()
        if remaining_steps(state) < 65:
            break

        handle = state["objects"]["handle"]
        position = handle["pos"]
        yaw = handle["yaw"] + attempt * pi / 2.0

        sdk.move_to(
            [position[0], position[1], position[2] + 0.10],
            yaw=yaw,
            gripper="open",
            max_steps=min(35, remaining_steps(state)),
            tol=0.015,
        )
        state = sdk.get_state()
        position = state["objects"]["handle"]["pos"]
        if remaining_steps(state) < 25:
            break
        sdk.move_to(
            [position[0], position[1], position[2]],
            yaw=yaw,
            gripper="open",
            max_steps=min(28, remaining_steps(state)),
            tol=0.008,
        )
        state = sdk.get_state()
        if remaining_steps(state) < 10:
            break
        sdk.set_gripper("close", steps=10)
        state = sdk.get_state()

        handle = state["objects"]["handle"]
        local_offset = quat_rotate(
            quat_inverse(handle["quat"]),
            [state["eef_pos"][i] - handle["pos"][i] for i in range(3)],
        )

        # Identify the wrist torque direction that increases the latch angle.
        # Each unsuccessful probe is reversed before trying another direction.
        axes = [
            quat_rotate(handle["quat"], [1.0, 0.0, 0.0]),
            quat_rotate(handle["quat"], [0.0, 1.0, 0.0]),
            quat_rotate(handle["quat"], [0.0, 0.0, 1.0]),
        ]
        best_axis = None
        best_gain = 0.001

        for axis in axes:
            if state["handle_qpos"] >= 0.40 or state["hinge_qpos"] > 0.06:
                break
            if remaining_steps(state) < 90:
                break
            for sign in [1.0, -1.0]:
                before = state["handle_qpos"]
                direction = [sign * component for component in axis]
                for tick in range(5):
                    handle = state["objects"]["handle"]
                    offset = quat_rotate(handle["quat"], local_offset)
                    translation = [
                        clamp(
                            18.0 * (handle["pos"][i] + offset[i] - state["eef_pos"][i]),
                            -0.5,
                            0.5,
                        )
                        for i in range(3)
                    ]
                    sdk.step_raw(
                        translation + [0.55 * v for v in direction] + [1.0],
                        steps=1,
                    )
                    state = sdk.get_state()
                gain = state["handle_qpos"] - before
                if gain > best_gain:
                    best_gain = gain
                    best_axis = direction
                if gain > 0.015:
                    break
                for tick in range(5):
                    if remaining_steps(state) <= 0:
                        break
                    sdk.step_raw(
                        [0.0, 0.0, 0.0]
                        + [-0.55 * v for v in direction]
                        + [1.0],
                        steps=1,
                    )
                    state = sdk.get_state()
            if best_gain > 0.015:
                break

        if best_axis is not None:
            for tick in range(32):
                if remaining_steps(state) < 45:
                    break
                if state["handle_qpos"] >= 0.45 or state["hinge_qpos"] > 0.06:
                    break
                handle = state["objects"]["handle"]
                offset = quat_rotate(handle["quat"], local_offset)
                translation = [
                    clamp(
                        18.0 * (handle["pos"][i] + offset[i] - state["eef_pos"][i]),
                        -0.55,
                        0.55,
                    )
                    for i in range(3)
                ]
                sdk.step_raw(
                    translation + [0.65 * v for v in best_axis] + [1.0],
                    steps=1,
                )
                state = sdk.get_state()

        handle = state["objects"]["handle"]
        relative_orientation = quat_multiply(
            quat_inverse(handle["quat"]), state["eef_quat"]
        )
        local_offset = quat_rotate(
            quat_inverse(handle["quat"]),
            [state["eef_pos"][i] - handle["pos"][i] for i in range(3)],
        )
        toward_robot = unit_horizontal(
            [start_position[i] - handle["pos"][i] for i in range(3)]
        )
        previous_handle = list(handle["pos"])
        previous_hinge = state["hinge_qpos"]
        motion_direction = None
        stagnant = 0

        for tick in range(190):
            if state["hinge_qpos"] > 0.30:
                return {
                    "status": "done",
                    "hinge_qpos": state["hinge_qpos"],
                    "steps_used": state["steps_used"],
                }
            if remaining_steps(state) <= 0:
                break

            handle = state["objects"]["handle"]
            door = state["objects"]["door"]
            radial = [
                handle["pos"][0] - door["pos"][0],
                handle["pos"][1] - door["pos"][1],
                0.0,
            ]
            tangent = unit_horizontal([-radial[1], radial[0], 0.0])

            # Prefer the measured opening trajectory once the hinge moves.
            if motion_direction is not None:
                direction = motion_direction
            elif stagnant < 24:
                direction = tangent
            elif stagnant < 48:
                direction = toward_robot
            else:
                direction = [-tangent[0], -tangent[1], 0.0]

            offset = quat_rotate(handle["quat"], local_offset)
            lead = 0.025 if state["hinge_qpos"] < 0.06 else 0.045
            target = [
                handle["pos"][i] + offset[i] + lead * direction[i]
                for i in range(3)
            ]
            translation = [
                clamp(18.0 * (target[i] - state["eef_pos"][i]), -0.75, 0.75)
                for i in range(3)
            ]

            target_orientation = quat_multiply(
                handle["quat"], relative_orientation
            )
            error = quat_multiply(
                target_orientation, quat_inverse(state["eef_quat"])
            )
            sign = 1.0 if error[3] >= 0.0 else -1.0
            rotation = [clamp(3.0 * sign * error[i], -0.6, 0.6) for i in range(3)]
            if best_axis is not None and state["hinge_qpos"] < 0.08:
                for i in range(3):
                    rotation[i] = clamp(
                        rotation[i] + 0.35 * best_axis[i], -0.8, 0.8
                    )

            sdk.step_raw(translation + rotation + [1.0], steps=1)
            state = sdk.get_state()

            if state["hinge_qpos"] > best_hinge + 0.002:
                best_hinge = state["hinge_qpos"]
                stagnant = 0
            else:
                stagnant += 1

            if tick % 6 == 5:
                current_handle = state["objects"]["handle"]["pos"]
                displacement = [
                    current_handle[i] - previous_handle[i] for i in range(3)
                ]
                if (
                    state["hinge_qpos"] - previous_hinge > 0.004
                    and hypot(displacement[0], displacement[1]) > 0.001
                ):
                    measured = unit_horizontal(displacement)
                    if motion_direction is None:
                        motion_direction = measured
                    else:
                        motion_direction = unit_horizontal(
                            [
                                0.4 * motion_direction[i] + 0.6 * measured[i]
                                for i in range(3)
                            ]
                        )
                previous_handle = list(current_handle)
                previous_hinge = state["hinge_qpos"]

            separation = sqrt(
                sum(
                    (state["eef_pos"][i] - state["objects"]["handle"]["pos"][i]) ** 2
                    for i in range(3)
                )
            )
            if separation > 0.10 or stagnant > 72:
                break

    state = sdk.get_state()
    return {
        "status": "done" if state["hinge_qpos"] > 0.30 else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "steps_used": state["steps_used"],
    }
