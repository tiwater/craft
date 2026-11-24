def run_policy(sdk):
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def joint(state, name):
        value = state[name]
        return 0.0 if value is None else float(value)

    def budget(state):
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def success(state):
        return joint(state, "hinge_qpos") > 0.30

    def multiply(a, b):
        return [
            a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
            a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
            a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
            a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
        ]

    def rotate(base, axis, angle):
        scale = sin(angle / 2.0)
        return multiply(
            [axis[0] * scale, axis[1] * scale,
             axis[2] * scale, cos(angle / 2.0)],
            base,
        )

    def control(state, target, orientation, grip=1.0):
        current = state["eef_quat"]
        error = multiply(
            orientation, [-current[0], -current[1], -current[2], current[3]]
        )
        sign = 1.0 if error[3] >= 0.0 else -1.0
        length = sqrt(sum(error[i] ** 2 for i in range(3)))
        scale = 2.0
        if length > 0.000001:
            scale = 2.0 * atan2(length, abs(error[3])) / length
        action = [
            clamp(18.0 * (target[i] - state["eef_pos"][i]), 0.8)
            for i in range(3)
        ]
        action += [
            clamp(2.2 * sign * scale * error[i], 0.55)
            for i in range(3)
        ]
        sdk.step_raw(action + [grip], steps=1)

    state = sdk.get_state()
    best_hinge = joint(state, "hinge_qpos")

    for attempt in range(3):
        state = sdk.get_state()
        if success(state) or budget(state) < 35:
            break

        # Reserve most of the horizon for contact and opening.
        sdk.set_gripper("open", steps=min(5, budget(state)))
        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        door_yaw = state["objects"]["door"]["yaw"]
        yaw = door_yaw + (pi / 2.0 if attempt == 1 else 0.0)
        sdk.move_to(
            [handle[0], handle[1], handle[2] + 0.065],
            yaw=yaw,
            gripper="open",
            max_steps=min(24, budget(state)),
            tol=0.012,
        )
        state = sdk.get_state()
        if success(state) or budget(state) <= 0:
            break
        sdk.move_to(
            list(state["objects"]["handle"]["pos"]),
            yaw=yaw,
            gripper="open",
            max_steps=min(28, budget(state)),
            tol=0.006,
        )
        state = sdk.get_state()
        if success(state) or budget(state) <= 0:
            break

        handle = state["objects"]["handle"]["pos"]
        distance = sqrt(sum(
            (state["eef_pos"][i] - handle[i]) ** 2 for i in range(3)
        ))
        if distance > 0.045:
            continue

        sdk.set_gripper("close", steps=min(10, budget(state)))
        state = sdk.get_state()
        base = list(state["eef_quat"])
        offset = [
            clamp(state["eef_pos"][i] - state["objects"]["handle"]["pos"][i], 0.02)
            for i in range(3)
        ]
        door_yaw = state["objects"]["door"]["yaw"]
        side = [cos(door_yaw), sin(door_yaw), 0.0]
        normal = [-sin(door_yaw), cos(door_yaw), 0.0]
        axis = [-normal[0], -normal[1], 0.0]
        start_hinge = joint(state, "hinge_qpos")
        best_latch = abs(joint(state, "handle_qpos"))
        best_angle = 0.0
        angle = 0.0
        twist_sign = 1.0
        stale = 0

        # Use sustained lever motion; a small transient latch deflection is
        # insufficient evidence that the latch has cleared.
        for tick in range(58):
            state = sdk.get_state()
            if success(state) or budget(state) <= 0:
                break
            latch = abs(joint(state, "handle_qpos"))
            hinge = joint(state, "hinge_qpos")
            if latch > best_latch + 0.015:
                best_latch = latch
                best_angle = angle
                stale = 0
            else:
                stale += 1

            if latch >= 0.55 or hinge > start_hinge + 0.035:
                best_angle = angle
                break

            if tick == 24 and best_latch < 0.10:
                twist_sign = -1.0
                angle = 0.0

            angle = clamp(angle + twist_sign * 0.035, 1.25)
            orientation = rotate(base, axis, angle)
            handle = state["objects"]["handle"]["pos"]
            press = min(0.045, 0.012 + 0.001 * tick)
            target = [
                handle[i] + offset[i] + 0.012 * side[i]
                for i in range(3)
            ]
            target[2] -= press
            control(state, target, orientation)

            if tick > 35 and stale > 25 and best_latch < 0.06:
                break

        state = sdk.get_state()
        if success(state) or budget(state) <= 0:
            break

        # Keep the best sustained wrist pose while pulling toward the
        # front of the door. The supplied goal need not be a pull direction.
        if best_latch < 0.10 and abs(joint(state, "handle_qpos")) < 0.10:
            continue
        if abs(joint(state, "handle_qpos")) >= best_latch - 0.08:
            best_angle = angle
        held_orientation = rotate(base, axis, best_angle)
        reference_hinge = joint(state, "hinge_qpos")
        reference_handle = list(state["objects"]["handle"]["pos"])
        direction = list(normal)
        previous_hinge = reference_hinge
        stagnant = 0
        lost = 0
        pull_sign = 1.0

        for tick in range(180):
            state = sdk.get_state()
            if success(state) or budget(state) <= 0:
                break
            hinge = joint(state, "hinge_qpos")
            latch = abs(joint(state, "handle_qpos"))
            handle = state["objects"]["handle"]["pos"]
            best_hinge = max(best_hinge, hinge)

            if hinge > previous_hinge + 0.006:
                previous_hinge = hinge
                stagnant = 0
            else:
                stagnant += 1

            # Estimate the hinge tangent only when latch motion has settled.
            if hinge > reference_hinge + 0.035 and latch > 0.40:
                vx = handle[0] - reference_handle[0]
                vy = handle[1] - reference_handle[1]
                travel = hypot(vx, vy)
                if travel > 0.008:
                    dx = 0.65 * direction[0] + 0.35 * vx / travel
                    dy = 0.65 * direction[1] + 0.35 * vy / travel
                    norm = hypot(dx, dy)
                    if norm > 0.000001:
                        direction = [dx / norm, dy / norm, 0.0]
                reference_hinge = hinge
                reference_handle = list(handle)

            distance = sqrt(sum(
                (state["eef_pos"][i] - handle[i]) ** 2 for i in range(3)
            ))
            if distance > 0.105:
                lost += 1
            else:
                lost = 0
            if lost >= 10:
                break

            if stagnant == 24 and hinge < 0.06:
                pull_sign = -pull_sign
            if stagnant > 48:
                break

            lead = 0.035 if latch < 0.35 and hinge < 0.06 else 0.065
            target = [
                handle[i] + offset[i] + pull_sign * lead * direction[i]
                for i in range(3)
            ]
            if latch < 0.65:
                target[2] -= 0.035
            else:
                target[2] -= 0.012

            # Rotate with the door while retaining the depressed lever pose.
            orientation = rotate(
                held_orientation,
                [0.0, 0.0, 1.0],
                hinge - start_hinge,
            )
            control(state, target, orientation)

    state = sdk.get_state()
    return {
        "status": "done" if success(state) else "failed",
        "hinge_qpos": joint(state, "hinge_qpos"),
        "handle_qpos": joint(state, "handle_qpos"),
        "best_hinge_qpos": max(best_hinge, joint(state, "hinge_qpos")),
        "steps_used": state["steps_used"],
    }
