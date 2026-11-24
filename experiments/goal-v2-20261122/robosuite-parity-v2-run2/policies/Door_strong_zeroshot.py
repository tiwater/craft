def quat_multiply(a, b):
    return [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]


def axis_quat(axis, angle):
    scale = sin(angle * 0.5)
    return [
        axis[0] * scale,
        axis[1] * scale,
        axis[2] * scale,
        cos(angle * 0.5),
    ]


def bounded(value, limit):
    return max(-limit, min(limit, value))


def control_tick(sdk, position, quaternion, grip):
    state = sdk.get_state()
    if state["steps_used"] >= state["max_steps"]:
        return False
    current = state["eef_quat"]
    conjugate = [-current[0], -current[1], -current[2], current[3]]
    error = quat_multiply(quaternion, conjugate)
    if error[3] < 0.0:
        error = [-value for value in error]
    magnitude = sqrt(sum(error[i] * error[i] for i in range(3)))
    scale = 0.0
    if magnitude > 0.000001:
        scale = 2.0 * atan2(magnitude, max(0.0, error[3])) / magnitude
    action = [
        bounded(0.65 * (position[i] - state["eef_pos"][i]) / 0.05, 0.8)
        for i in range(3)
    ]
    action += [
        bounded(0.55 * error[i] * scale / 0.5, 0.65)
        for i in range(3)
    ]
    action.append(grip)
    sdk.step_raw(action, steps=1)
    return True


def reach_pose(sdk, position, quaternion, grip, ticks, tolerance):
    for index in range(ticks):
        state = sdk.get_state()
        if state["steps_used"] >= state["max_steps"]:
            return False
        distance = sqrt(sum(
            (position[i] - state["eef_pos"][i]) ** 2 for i in range(3)
        ))
        alignment = abs(sum(
            quaternion[i] * state["eef_quat"][i] for i in range(4)
        ))
        if index >= 5 and distance < tolerance and alignment > 0.985:
            return True
        control_tick(sdk, position, quaternion, grip)
    return False


def door_basis(state):
    handle = state["objects"]["handle"]["pos"]
    door = state["objects"]["door"]["pos"]
    dx = handle[0] - door[0]
    dy = handle[1] - door[1]
    length = hypot(dx, dy)
    if length < 0.025:
        yaw = state["objects"]["door"]["yaw"]
        dx = -sin(yaw)
        dy = cos(yaw)
        length = 1.0
    radial = [dx / length, dy / length, 0.0]
    tangent = [-radial[1], radial[0], 0.0]
    return radial, tangent


def run_policy(sdk):
    state = sdk.get_state()
    limit = min(state["max_steps"], state["steps_used"] + 400)
    best_hinge = state["hinge_qpos"] or 0.0

    for attempt in range(3):
        state = sdk.get_state()
        if (state["hinge_qpos"] or 0.0) > 0.30:
            return {"status": "done", "hinge_qpos": state["hinge_qpos"]}
        if limit - state["steps_used"] < 65:
            break

        radial, normal = door_basis(state)
        yaw = atan2(radial[1], radial[0])
        if attempt == 1:
            yaw += pi * 0.5
        base_quat = [cos(yaw * 0.5), sin(yaw * 0.5), 0.0, 0.0]
        handle = list(state["objects"]["handle"]["pos"])
        approach = [handle[0], handle[1], handle[2] + 0.10]

        reach_pose(sdk, approach, base_quat, -1.0, 40, 0.015)
        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        reach_pose(sdk, handle, base_quat, -1.0, 32, 0.007)

        for index in range(12):
            state = sdk.get_state()
            if state["steps_used"] >= limit:
                break
            control_tick(sdk, handle, base_quat, 1.0)

        state = sdk.get_state()
        anchor = list(state["objects"]["handle"]["pos"])
        hinge_start = state["hinge_qpos"] or 0.0
        initial_handle = state["handle_qpos"] or 0.0
        turn_sign = 1.0
        turn_angle = 0.0
        wrist_quat = base_quat
        unlocked = False

        for index in range(52):
            state = sdk.get_state()
            if state["steps_used"] >= limit:
                break
            hinge = state["hinge_qpos"] or 0.0
            handle_angle = state["handle_qpos"] or 0.0
            if hinge > 0.30:
                return {"status": "done", "hinge_qpos": hinge}

            if index == 12 and handle_angle < initial_handle - 0.025:
                turn_sign = -1.0

            turn_angle = turn_sign * min(0.95, 0.025 * (index + 1))
            wrist_quat = quat_multiply(
                axis_quat(normal, turn_angle), base_quat
            )
            depth = min(0.045, 0.0012 * (index + 1))
            target = [anchor[0], anchor[1], anchor[2] - depth]
            control_tick(sdk, target, wrist_quat, 1.0)

            if handle_angle > 0.32 or hinge > hinge_start + 0.025:
                unlocked = True
                break

        state = sdk.get_state()
        held_quat = list(state["eef_quat"])
        held_hinge = state["hinge_qpos"] or 0.0
        handle = state["objects"]["handle"]["pos"]
        offset = [
            bounded(state["eef_pos"][i] - handle[i], 0.025)
            for i in range(3)
        ]
        direction = 1.0
        checkpoint_hinge = held_hinge
        best_attempt = held_hinge
        stalled = 0
        opening_ticks = 125 if unlocked else 75

        for index in range(opening_ticks):
            state = sdk.get_state()
            if state["steps_used"] >= limit:
                break
            hinge = state["hinge_qpos"] or 0.0
            best_hinge = max(best_hinge, hinge)
            if hinge > 0.30:
                return {
                    "status": "done",
                    "hinge_qpos": hinge,
                    "steps_used": state["steps_used"],
                }

            if hinge > best_attempt + 0.003:
                best_attempt = hinge
                stalled = 0
            else:
                stalled += 1

            if index > 0 and index % 18 == 0:
                if hinge - checkpoint_hinge < 0.005:
                    direction = -direction
                checkpoint_hinge = hinge

            radial, tangent = door_basis(state)
            handle = state["objects"]["handle"]["pos"]
            angle = hinge - held_hinge
            rotated_offset = [
                cos(angle) * offset[0] - sin(angle) * offset[1],
                sin(angle) * offset[0] + cos(angle) * offset[1],
                offset[2],
            ]
            lead = 0.045 if unlocked else 0.03
            target = [
                handle[i] + rotated_offset[i] + direction * lead * tangent[i]
                for i in range(3)
            ]
            if not unlocked:
                target[2] -= 0.012

            wrist_quat = quat_multiply(
                axis_quat([0.0, 0.0, 1.0], angle), held_quat
            )
            control_tick(sdk, target, wrist_quat, 1.0)

            separation = sqrt(sum(
                (state["eef_pos"][i] - handle[i]) ** 2 for i in range(3)
            ))
            if separation > 0.105 or stalled > 48:
                break

        state = sdk.get_state()
        if state["steps_used"] + 12 < limit:
            position = list(state["eef_pos"])
            for index in range(6):
                control_tick(sdk, position, state["eef_quat"], -1.0)
            position[2] += 0.085
            reach_pose(sdk, position, base_quat, -1.0, 16, 0.02)

    state = sdk.get_state()
    hinge = state["hinge_qpos"] or 0.0
    return {
        "status": "done" if hinge > 0.30 else "failed",
        "hinge_qpos": hinge,
        "handle_qpos": state["handle_qpos"],
        "best_hinge_qpos": max(best_hinge, hinge),
        "steps_used": state["steps_used"],
    }
