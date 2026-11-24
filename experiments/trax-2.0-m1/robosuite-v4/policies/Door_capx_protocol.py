def run_policy(sdk):
    def clamp(x, limit):
        return max(-limit, min(limit, x))

    def joint(state, name):
        return 0.0 if state[name] is None else float(state[name])

    def budget(state):
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def finished(state):
        return joint(state, "hinge_qpos") > 0.30

    def rotation(current, target):
        x, y, z, w = current
        a, b, c, d = target
        sign = 1.0 if x * a + y * b + z * c + w * d >= 0.0 else -1.0
        return [
            clamp(5.0 * sign * (-d * x + a * w - b * z + c * y), 1.0),
            clamp(5.0 * sign * (-d * y + a * z + b * w - c * x), 1.0),
            clamp(5.0 * sign * (-d * z - a * y + b * x + c * w), 1.0),
        ]

    def servo(state, target, angular, grip):
        if budget(state) > 0 and not finished(state):
            linear = [
                clamp(25.0 * (target[i] - state["eef_pos"][i]), 1.0)
                for i in range(3)
            ]
            sdk.step_raw(linear + angular + [grip], steps=1)

    def approach(offset, quaternion, ticks, grip):
        for tick in range(ticks):
            state = sdk.get_state()
            if budget(state) <= 0 or finished(state):
                return
            handle = state["objects"]["handle"]["pos"]
            target = [handle[i] + offset[i] for i in range(3)]
            angular = rotation(state["eef_quat"], quaternion)
            servo(state, target, angular, grip)
            error = sqrt(sum(
                (target[i] - state["eef_pos"][i]) ** 2
                for i in range(3)
            ))
            if tick >= 5 and error < 0.008 and max(abs(v) for v in angular) < 0.12:
                return

    state = sdk.get_state()
    initial_hinge = joint(state, "hinge_qpos")
    handle = state["objects"]["handle"]["pos"]
    door = state["objects"]["door"]["pos"]
    rx = handle[0] - door[0]
    ry = handle[1] - door[1]
    radius = max(0.05, hypot(rx, ry))
    radial = [rx / radius, ry / radius]

    for attempt in range(3):
        state = sdk.get_state()
        if finished(state) or budget(state) < 20:
            break

        yaw = state["objects"]["door"]["yaw"]
        if attempt == 1:
            yaw += pi / 2.0
        quaternion = [cos(yaw / 2.0), sin(yaw / 2.0), 0.0, 0.0]

        sdk.set_gripper("open", steps=min(6, budget(state)))
        approach([0.0, 0.0, 0.12], quaternion, 26, -1.0)
        approach([0.0, 0.0, 0.005], quaternion, 30, -1.0)

        state = sdk.get_state()
        if finished(state) or budget(state) <= 0:
            break
        sdk.set_gripper("close", steps=min(10, budget(state)))
        state = sdk.get_state()
        offset = [
            clamp(state["eef_pos"][i] - state["objects"]["handle"]["pos"][i], 0.018)
            for i in range(3)
        ]

        yaw = state["objects"]["door"]["yaw"]
        axes = [
            [-0.65 * sin(yaw), 0.65 * cos(yaw), 0.0],
            [0.65 * sin(yaw), -0.65 * cos(yaw), 0.0],
            [0.65 * cos(yaw), 0.65 * sin(yaw), 0.0],
            [-0.65 * cos(yaw), -0.65 * sin(yaw), 0.0],
        ]
        best_axis = axes[0]
        best_gain = -100.0

        for axis in axes:
            state = sdk.get_state()
            if finished(state) or budget(state) <= 0:
                break
            if abs(joint(state, "handle_qpos")) >= 0.45:
                break
            before = abs(joint(state, "handle_qpos"))
            for tick in range(6):
                state = sdk.get_state()
                if finished(state) or budget(state) <= 0:
                    break
                handle = state["objects"]["handle"]["pos"]
                target = [handle[i] + offset[i] for i in range(3)]
                servo(state, target, axis, 1.0)
            state = sdk.get_state()
            gain = abs(joint(state, "handle_qpos")) - before
            if gain > best_gain:
                best_gain = gain
                best_axis = list(axis)

        for tick in range(18):
            state = sdk.get_state()
            if finished(state) or budget(state) <= 0:
                break
            if abs(joint(state, "handle_qpos")) >= 0.50:
                break
            handle = state["objects"]["handle"]["pos"]
            servo(
                state,
                [handle[i] + offset[i] for i in range(3)],
                best_axis,
                1.0,
            )

        state = sdk.get_state()
        held = list(state["eef_quat"])
        held_hinge = joint(state, "hinge_qpos")
        best_hinge = held_hinge
        stagnant = 0

        for tick in range(125):
            state = sdk.get_state()
            if finished(state) or budget(state) <= 0:
                break

            hinge = joint(state, "hinge_qpos")
            angle = hinge - initial_hinge
            ux = cos(angle) * radial[0] - sin(angle) * radial[1]
            uy = sin(angle) * radial[0] + cos(angle) * radial[1]
            tangent = [-uy, ux]
            handle = state["objects"]["handle"]["pos"]
            latch = abs(joint(state, "handle_qpos"))

            if latch < 0.38 and hinge < 0.06:
                target = [
                    handle[0] + offset[0] + 0.008 * tangent[0],
                    handle[1] + offset[1] + 0.008 * tangent[1],
                    handle[2] + offset[2],
                ]
                angular = best_axis
                held = list(state["eef_quat"])
                held_hinge = hinge
            else:
                lead = min(0.075, 0.022 + 0.0015 * tick)
                target = [
                    handle[0] + offset[0] + lead * tangent[0],
                    handle[1] + offset[1] + lead * tangent[1],
                    handle[2] + offset[2],
                ]
                half = 0.5 * (hinge - held_hinge)
                s = sin(half)
                c = cos(half)
                desired = [
                    c * held[0] - s * held[1],
                    c * held[1] + s * held[0],
                    c * held[2] + s * held[3],
                    c * held[3] - s * held[2],
                ]
                angular = rotation(state["eef_quat"], desired)

            servo(state, target, angular, 1.0)
            updated = sdk.get_state()
            new_hinge = joint(updated, "hinge_qpos")
            if new_hinge > best_hinge + 0.002:
                best_hinge = new_hinge
                stagnant = 0
            else:
                stagnant += 1

            distance = sqrt(sum(
                (updated["eef_pos"][i] - updated["objects"]["handle"]["pos"][i]) ** 2
                for i in range(3)
            ))
            if stagnant > 28 and (
                updated["gripper_width"] < 0.012 or distance > 0.075
            ):
                break
            if stagnant > 48:
                break

    state = sdk.get_state()
    return {
        "status": "done" if finished(state) else "failed",
        "hinge_qpos": joint(state, "hinge_qpos"),
        "handle_qpos": joint(state, "handle_qpos"),
        "steps_used": state["steps_used"],
    }
