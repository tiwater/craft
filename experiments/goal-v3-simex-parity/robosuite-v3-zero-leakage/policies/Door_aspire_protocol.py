def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def hinge(state):
        value = state["hinge_qpos"]
        return 0.0 if value is None else value

    def latch(state):
        value = state["handle_qpos"]
        return 0.0 if value is None else value

    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def multiply(a, b):
        return [
            a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
            a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
            a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
            a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
        ]

    def rotate(quaternion, axis, angle):
        scale = sin(0.5 * angle)
        return multiply(
            [axis[0] * scale, axis[1] * scale,
             axis[2] * scale, cos(0.5 * angle)],
            quaternion,
        )

    def distance(state):
        handle = state["objects"]["handle"]["pos"]
        return sqrt(sum(
            (state["eef_pos"][i] - handle[i]) ** 2
            for i in range(3)
        ))

    def unit(x, y):
        length = hypot(x, y)
        if length < 0.000001:
            return [0.0, -1.0]
        return [x / length, y / length]

    def tick(target, orientation, grip):
        state = sdk.get_state()
        if remaining() <= 0 or hinge(state) > 0.30:
            return
        current = state["eef_quat"]
        error = multiply(
            orientation,
            [-current[0], -current[1], -current[2], current[3]],
        )
        sign = 1.0 if error[3] >= 0.0 else -1.0
        length = sqrt(sum(error[i] ** 2 for i in range(3)))
        scale = (
            2.0 * atan2(length, abs(error[3])) / length
            if length > 0.000001 else 2.0
        )
        action = [
            clamp(22.0 * (target[i] - state["eef_pos"][i]), 0.8)
            for i in range(3)
        ]
        action += [
            clamp(2.5 * sign * scale * error[i], 0.65)
            for i in range(3)
        ]
        sdk.step_raw(action + [grip], steps=1)

    initial = sdk.get_state()
    approach = list(initial["eef_quat"])
    best_hinge = hinge(initial)
    best_latch = abs(latch(initial))
    attempts = 0

    while remaining() > 0 and attempts < 4:
        state = sdk.get_state()
        if hinge(state) > 0.30:
            break

        orientation = rotate(
            approach, [0.0, 0.0, 1.0],
            (attempts % 2) * pi * 0.5,
        )
        attempts += 1

        # Release before retreating, then approach the current handle site.
        for step in range(min(7, remaining())):
            state = sdk.get_state()
            tick(list(state["eef_pos"]), orientation, -1.0)

        state = sdk.get_state()
        handle = state["objects"]["handle"]["pos"]
        retreat = list(state["eef_pos"])
        retreat[2] = max(retreat[2], handle[2] + 0.07)
        for step in range(min(12, remaining())):
            tick(retreat, orientation, -1.0)

        for step in range(min(22, remaining())):
            state = sdk.get_state()
            handle = state["objects"]["handle"]["pos"]
            target = [handle[0], handle[1], handle[2] + 0.055]
            tick(target, orientation, -1.0)
            if sqrt(sum(
                (sdk.get_state()["eef_pos"][i] - target[i]) ** 2
                for i in range(3)
            )) < 0.012:
                break

        for step in range(min(27, remaining())):
            state = sdk.get_state()
            tick(list(state["objects"]["handle"]["pos"]), orientation, -1.0)
            if distance(sdk.get_state()) < 0.009:
                break

        for step in range(min(12, remaining())):
            state = sdk.get_state()
            tick(list(state["objects"]["handle"]["pos"]), orientation, 1.0)

        state = sdk.get_state()
        if hinge(state) > 0.30:
            break
        if distance(state) > 0.045 or state["gripper_width"] < 0.011:
            continue

        handle = list(state["objects"]["handle"]["pos"])
        offset = [
            clamp(state["eef_pos"][i] - handle[i], 0.018)
            for i in range(3)
        ]
        grasp_orientation = list(state["eef_quat"])
        grasp_latch = latch(state)
        grasp_hinge = hinge(state)
        door_yaw = state["objects"]["door"]["yaw"]
        axis = [sin(door_yaw), -cos(door_yaw), 0.0]

        # Follow the moving lever instead of pinning the wrist to its
        # pre-turn position. Positive lever travel moves down and outward.
        lost = 0
        turn_start = latch(state)
        for step in range(min(62, remaining())):
            state = sdk.get_state()
            best_hinge = max(best_hinge, hinge(state))
            best_latch = max(best_latch, abs(latch(state)))
            if hinge(state) > 0.30:
                break
            if latch(state) >= 0.55 or hinge(state) > 0.045:
                break

            lost = lost + 1 if (
                distance(state) > 0.055
                or state["gripper_width"] < 0.010
            ) else 0
            if lost >= 5:
                break

            q = latch(state)
            lead = clamp(0.65 - q, 0.22)
            orientation = rotate(
                grasp_orientation, axis, q - grasp_latch + lead
            )
            orientation = rotate(
                orientation, [0.0, 0.0, 1.0],
                hinge(state) - grasp_hinge,
            )
            yaw = door_yaw + hinge(state) - grasp_hinge
            handle = state["objects"]["handle"]["pos"]
            drive = clamp(0.055 * (0.65 - q), 0.018)
            target = [
                handle[0] + offset[0] + drive * sin(q) * cos(yaw),
                handle[1] + offset[1] + drive * sin(q) * sin(yaw),
                handle[2] + offset[2] - drive * cos(q),
            ]
            tick(target, orientation, 1.0)
            if step >= 35 and latch(state) - turn_start < 0.035:
                break

        state = sdk.get_state()
        if hinge(state) > 0.30:
            break
        if lost >= 5 or distance(state) > 0.060:
            continue

        # Preserve the achieved latch angle while testing opening forces.
        held_latch = max(0.58, min(1.20, latch(state)))
        handle = state["objects"]["handle"]["pos"]
        goal = state["goal_pos"]
        primary = (
            unit(goal[0] - handle[0], goal[1] - handle[1])
            if goal is not None
            else [sin(door_yaw), -cos(door_yaw)]
        )
        directions = [
            primary,
            [sin(door_yaw), -cos(door_yaw)],
            [-sin(door_yaw), cos(door_yaw)],
            [cos(door_yaw), sin(door_yaw)],
            [-cos(door_yaw), -sin(door_yaw)],
        ]
        direction = list(primary)
        probe = 0
        stalled = 0
        lost = 0
        pull_ticks = 0
        block_hinge = hinge(state)
        block_handle = list(handle)
        best_direction = list(primary)
        best_gain = 0.0

        while remaining() > 0 and pull_ticks < 135:
            state = sdk.get_state()
            if hinge(state) > 0.30:
                break

            lost = lost + 1 if (
                distance(state) > 0.060
                or state["gripper_width"] < 0.010
            ) else 0
            if lost >= 5:
                break

            q = latch(state)
            angle = clamp(held_latch - q, 0.18)
            orientation = rotate(
                grasp_orientation, axis, q - grasp_latch + angle
            )
            orientation = rotate(
                orientation, [0.0, 0.0, 1.0],
                hinge(state) - grasp_hinge,
            )
            handle = state["objects"]["handle"]["pos"]
            yaw = door_yaw + hinge(state) - grasp_hinge
            latch_drive = clamp(0.045 * (held_latch - q), 0.012)
            pull = 0.032 if hinge(state) < 0.06 else 0.045
            target = [
                handle[0] + offset[0] + pull * direction[0]
                + latch_drive * sin(q) * cos(yaw),
                handle[1] + offset[1] + pull * direction[1]
                + latch_drive * sin(q) * sin(yaw),
                handle[2] + offset[2] - latch_drive * cos(q),
            ]
            tick(target, orientation, 1.0)
            pull_ticks += 1

            after = sdk.get_state()
            best_hinge = max(best_hinge, hinge(after))
            best_latch = max(best_latch, abs(latch(after)))

            if pull_ticks % 10 == 0:
                gain = hinge(after) - block_hinge
                live = after["objects"]["handle"]["pos"]
                if gain > 0.003:
                    stalled = 0
                    if gain > best_gain:
                        best_gain = gain
                        best_direction = list(direction)
                    dx = live[0] - block_handle[0]
                    dy = live[1] - block_handle[1]
                    if (
                        hypot(dx, dy) > 0.002
                        and abs(latch(after) - held_latch) < 0.15
                    ):
                        tangent = unit(dx, dy)
                        direction = unit(
                            0.7 * direction[0] + 0.3 * tangent[0],
                            0.7 * direction[1] + 0.3 * tangent[1],
                        )
                else:
                    stalled += 1
                    if stalled >= 2:
                        stalled = 0
                        probe += 1
                        if probe >= len(directions) and best_gain > 0.003:
                            direction = list(best_direction)
                        else:
                            direction = list(directions[probe % len(directions)])

                block_hinge = hinge(after)
                block_handle = list(live)

                if (
                    pull_ticks >= 70
                    and hinge(after) < 0.035
                    and remaining() > 85
                ):
                    break

    state = sdk.get_state()
    return {
        "status": "done" if hinge(state) > 0.30 else "failed",
        "hinge_qpos": hinge(state),
        "handle_qpos": latch(state),
        "best_hinge_qpos": max(best_hinge, hinge(state)),
        "best_handle_magnitude": max(best_latch, abs(latch(state))),
        "attempts": attempts,
        "steps_used": state["steps_used"],
    }
