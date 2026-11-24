def door_clip(value, limit):
    return max(-limit, min(limit, value))


def door_quat_mul(a, b):
    return [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]


def door_axis_quat(axis, angle):
    scale = sin(angle * 0.5)
    return [axis[0] * scale, axis[1] * scale,
            axis[2] * scale, cos(angle * 0.5)]


def door_servo(sdk, state, target, orientation, grip):
    current = state["eef_quat"]
    inverse = [-current[0], -current[1], -current[2], current[3]]
    error = door_quat_mul(orientation, inverse)
    sign = 1.0 if error[3] >= 0.0 else -1.0
    vector_length = sqrt(sum(error[i] * error[i] for i in range(3)))
    rotation = [0.0, 0.0, 0.0]
    if vector_length > 0.000001:
        angle = 2.0 * atan2(vector_length, abs(error[3]))
        rotation = [
            sign * error[i] * angle / vector_length for i in range(3)
        ]
    action = [
        door_clip(0.65 * (target[i] - state["eef_pos"][i]) / 0.05, 0.7)
        for i in range(3)
    ]
    action += [door_clip(rotation[i] / 0.5, 0.55) for i in range(3)]
    action += [grip]
    sdk.step_raw(action, steps=1)


def run_policy(sdk):
    state = sdk.get_state()
    limit = min(400, state["max_steps"])
    if state["hinge_qpos"] > 0.30:
        return {"status": "done", "hinge_qpos": state["hinge_qpos"]}

    handle = state["objects"]["handle"]["pos"]
    door = state["objects"]["door"]["pos"]
    start = list(state["eef_pos"])

    radial = [handle[0] - door[0], handle[1] - door[1]]
    length = hypot(radial[0], radial[1])
    if length > 0.025:
        radial = [radial[0] / length, radial[1] / length]
        normal = [-radial[1], radial[0], 0.0]
    else:
        length = hypot(start[0] - handle[0], start[1] - handle[1])
        if length < 0.001:
            return {"status": "failed", "reason": "Ambiguous approach geometry"}
        normal = [
            (start[0] - handle[0]) / length,
            (start[1] - handle[1]) / length,
            0.0,
        ]
        radial = [normal[1], -normal[0]]

    if sum(normal[i] * (start[i] - handle[i]) for i in range(2)) < 0.0:
        normal = [-normal[0], -normal[1], 0.0]

    yaw = atan2(radial[1], radial[0])
    while yaw - state["eef_yaw"] > pi * 0.5:
        yaw -= pi
    while yaw - state["eef_yaw"] < -pi * 0.5:
        yaw += pi

    approach = [
        handle[0] + 0.075 * normal[0],
        handle[1] + 0.075 * normal[1],
        handle[2] + 0.065,
    ]
    remaining = limit - state["steps_used"]
    if remaining < 35:
        return {"status": "failed", "reason": "Insufficient remaining steps"}
    sdk.move_to(approach, yaw=yaw, gripper="open",
                max_steps=min(40, remaining - 20), tol=0.015)

    state = sdk.get_state()
    handle = state["objects"]["handle"]["pos"]
    remaining = limit - state["steps_used"]
    if remaining <= 15:
        return {"status": "failed", "reason": "Approach exhausted step budget"}
    sdk.move_to(list(handle), yaw=yaw, gripper="open",
                max_steps=min(35, remaining - 12), tol=0.007)

    state = sdk.get_state()
    remaining = limit - state["steps_used"]
    if remaining <= 0:
        return {"status": "failed", "reason": "Step budget exhausted"}
    sdk.set_gripper("close", steps=min(12, remaining))

    state = sdk.get_state()
    reference = list(state["eef_quat"])
    handle = state["objects"]["handle"]["pos"]
    offset = [
        door_clip(state["eef_pos"][i] - handle[i], 0.018)
        for i in range(3)
    ]
    initial_hinge = state["hinge_qpos"]
    previous_hinge = initial_hinge
    previous_handle = list(handle)
    previous_latch = state["handle_qpos"]
    twist = 0.0
    twist_sign = 1.0
    best_latch = abs(previous_latch)
    stagnant = 0
    pull_sign = 1.0
    phase = "unlatch"
    phase_ticks = 0
    direction = list(normal)

    while state["steps_used"] < limit:
        hinge = state["hinge_qpos"]
        latch = state["handle_qpos"]
        handle = state["objects"]["handle"]["pos"]

        if hinge > 0.30:
            return {
                "status": "done",
                "hinge_qpos": hinge,
                "handle_qpos": latch,
                "steps_used": state["steps_used"],
            }

        delta_hinge = hinge - previous_hinge
        displacement = [
            handle[i] - previous_handle[i] for i in range(3)
        ]
        travel = hypot(displacement[0], displacement[1])
        if delta_hinge > 0.001 and travel > 0.0005:
            tangent = [
                displacement[0] / travel,
                displacement[1] / travel,
                0.0,
            ]
            direction = [
                0.8 * direction[i] + 0.2 * tangent[i] for i in range(3)
            ]
            norm = hypot(direction[0], direction[1])
            if norm > 0.001:
                direction = [direction[0] / norm, direction[1] / norm, 0.0]
            pull_sign = 1.0

        angle = hinge - initial_hinge
        axis = [
            cos(angle) * normal[0] - sin(angle) * normal[1],
            sin(angle) * normal[0] + cos(angle) * normal[1],
            0.0,
        ]
        rotated_offset = [
            cos(angle) * offset[0] - sin(angle) * offset[1],
            sin(angle) * offset[0] + cos(angle) * offset[1],
            offset[2],
        ]

        phase_ticks += 1
        if abs(latch) > best_latch + 0.008:
            best_latch = abs(latch)
            stagnant = 0
        else:
            stagnant += 1

        if phase == "unlatch":
            twist += twist_sign * 0.023
            twist = door_clip(twist, 1.05)
            pressure = 0.008
            downward = min(0.028, 0.0006 * phase_ticks)

            if phase_ticks == 30 and best_latch < 0.08:
                twist_sign = -1.0
            if abs(latch) > 0.28 or phase_ticks >= 85 or hinge > 0.025:
                phase = "pull"
                phase_ticks = 0
                stagnant = 0
        else:
            pressure = min(0.055, 0.018 + 0.0008 * phase_ticks)
            downward = 0.018 if hinge < 0.08 else 0.005

            if hinge < 0.08 and abs(latch) < 0.22:
                twist = door_clip(twist + twist_sign * 0.012, 1.15)

            if delta_hinge > 0.0006:
                stagnant = 0

            if hinge < 0.025 and stagnant >= 40:
                pull_sign = -pull_sign
                stagnant = 0
                phase_ticks = 0

        if phase == "unlatch":
            pull_direction = axis
        else:
            pull_direction = direction

        target = [
            handle[i] + rotated_offset[i]
            + pressure * pull_sign * pull_direction[i]
            for i in range(3)
        ]
        target[2] -= downward

        hinge_rotation = door_axis_quat([0.0, 0.0, 1.0], angle)
        latch_rotation = door_axis_quat(axis, twist)
        orientation = door_quat_mul(
            latch_rotation, door_quat_mul(hinge_rotation, reference)
        )

        previous_hinge = hinge
        previous_handle = list(handle)
        previous_latch = latch
        door_servo(sdk, state, target, orientation, 1.0)
        state = sdk.get_state()

    return {
        "status": "done" if state["hinge_qpos"] > 0.30 else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "steps_used": state["steps_used"],
    }
