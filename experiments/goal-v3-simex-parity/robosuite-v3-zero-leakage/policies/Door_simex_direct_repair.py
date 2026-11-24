def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def hinge(state):
        value = state["hinge_qpos"]
        return value if value is not None else 0.0

    def latch(state):
        value = state["handle_qpos"]
        return value if value is not None else 0.0

    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def normal(state):
        x, y, z, w = state["objects"]["door"]["quat"]
        axis = [2.0 * (x * y - w * z),
                1.0 - 2.0 * (x * x + z * z)]
        length = max(0.00001, hypot(axis[0], axis[1]))
        return [axis[0] / length, axis[1] / length]

    def multiply(a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]

    def rotated(quat, axis, angle):
        scale = sin(angle / 2.0)
        return multiply(
            [axis[0] * scale, axis[1] * scale,
             axis[2] * scale, cos(angle / 2.0)],
            quat,
        )

    def control(target, orientation):
        state = sdk.get_state()
        current = state["eef_quat"]
        error = multiply(
            orientation,
            [-current[0], -current[1], -current[2], current[3]],
        )
        sign = 1.0 if error[3] >= 0.0 else -1.0
        action = [
            clamp(24.0 * (target[i] - state["eef_pos"][i]), 0.7)
            for i in range(3)
        ]
        action += [
            clamp(5.0 * sign * error[i], 0.55)
            for i in range(3)
        ]
        action += [1.0]
        if remaining() > 0:
            sdk.step_raw(action, steps=1)

    def move(target, ticks, grip):
        budget = min(ticks, remaining())
        if budget > 0:
            sdk.move_to(target, gripper=grip, max_steps=budget, tol=0.005)

    def actuate(grip, ticks):
        budget = min(ticks, remaining())
        if budget > 0:
            sdk.set_gripper(grip, steps=budget)

    state = sdk.get_state()
    if hinge(state) > 0.30:
        return {"status": "done", "hinge_qpos": hinge(state)}

    handle = list(state["objects"]["handle"]["pos"])
    approach = [
        state["eef_pos"][0] - handle[0],
        state["eef_pos"][1] - handle[1],
    ]
    move([handle[0], handle[1], handle[2] + 0.075], 30, "open")
    handle = list(sdk.get_state()["objects"]["handle"]["pos"])
    move(handle, 30, "open")
    actuate("close", 12)

    state = sdk.get_state()
    if state["gripper_width"] < 0.012:
        actuate("open", 5)
        handle = list(sdk.get_state()["objects"]["handle"]["pos"])
        move([handle[0], handle[1], handle[2] - 0.01], 18, "open")
        actuate("close", 12)

    state = sdk.get_state()
    anchor = list(state["eef_pos"])
    base_orientation = list(state["eef_quat"])
    axis = normal(state)
    twist_axis = [axis[0], axis[1], 0.0]
    pull_sign = 1.0
    if axis[0] * approach[0] + axis[1] * approach[1] < 0.0:
        pull_sign = -1.0

    twist = 0.0
    twist_sign = 1.0
    best_latch = latch(state)
    best_hinge = hinge(state)
    best_twist = 0.0
    previous_latch = latch(state)
    latch_ticks = 0
    latch_stalled = 0
    pull_ticks = 0
    pull_start = hinge(state)
    recoveries = 0
    pulling = False
    hold_orientation = list(base_orientation)
    hold_hinge = hinge(state)
    offset = [0.0, 0.0, 0.0]

    while remaining() > 0:
        state = sdk.get_state()
        current_hinge = hinge(state)
        current_latch = latch(state)
        best_hinge = max(best_hinge, current_hinge)

        if current_hinge > 0.30:
            return {
                "status": "done",
                "hinge_qpos": current_hinge,
                "handle_qpos": current_latch,
                "steps_used": state["steps_used"],
            }

        if current_latch > best_latch:
            best_latch = current_latch
            best_twist = twist

        if not pulling:
            # Twist toward a bounded absolute wrist orientation. Repeated
            # raw rotation commands otherwise keep spinning after release.
            if current_latch >= 0.43 or current_hinge > 0.06:
                pulling = True
                hold_orientation = list(state["eef_quat"])
                hold_hinge = current_hinge
                handle = state["objects"]["handle"]["pos"]
                offset = [
                    state["eef_pos"][i] - handle[i] for i in range(3)
                ]
                pull_start = current_hinge
                pull_ticks = 0
                continue

            twist = clamp(twist + twist_sign * 0.025, 1.15)
            orientation = rotated(base_orientation, twist_axis, twist)
            target = list(anchor)
            target[2] -= min(0.025, latch_ticks * 0.0004)
            control(target, orientation)
            latch_ticks += 1

            if latch_ticks % 10 == 0:
                updated = sdk.get_state()
                value = latch(updated)
                if value < previous_latch - 0.025:
                    twist_sign = -twist_sign
                    twist = best_twist
                    latch_stalled = 0
                elif value <= previous_latch + 0.006:
                    latch_stalled += 1
                else:
                    latch_stalled = 0
                previous_latch = value
                if latch_stalled >= 3:
                    twist_sign = -twist_sign
                    twist = best_twist
                    latch_stalled = 0

            lost = state["gripper_width"] < 0.010
            if (
                latch_ticks >= 75
                and current_latch < 0.20
                and remaining() > 100
                and recoveries < 2
            ):
                lost = True
            if lost and latch_ticks > 20 and remaining() > 90 and recoveries < 2:
                recoveries += 1
                actuate("open", 5)
                handle = list(sdk.get_state()["objects"]["handle"]["pos"])
                move([handle[0], handle[1], handle[2] + 0.045], 14, "open")
                move(handle, 20, "open")
                actuate("close", 12)
                state = sdk.get_state()
                anchor = list(state["eef_pos"])
                base_orientation = list(state["eef_quat"])
                axis = normal(state)
                twist_axis = [axis[0], axis[1], 0.0]
                twist = 0.0
                best_twist = 0.0
                previous_latch = latch(state)
                latch_ticks = 0
                latch_stalled = 0
            continue

        # Preserve the released wrist pose while following the door.
        # Rotate the grasp offset with the hinge instead of accumulating
        # vertical offsets or continuously twisting the wrist.
        angle = current_hinge - hold_hinge
        ca = cos(angle)
        sa = sin(angle)
        carried_offset = [
            ca * offset[0] - sa * offset[1],
            sa * offset[0] + ca * offset[1],
            offset[2],
        ]
        orientation = rotated(hold_orientation, [0.0, 0.0, 1.0], angle)
        axis = normal(state)
        handle = state["objects"]["handle"]["pos"]
        distance = 0.035 if current_hinge < 0.08 else 0.060
        target = [
            handle[0] + carried_offset[0] + pull_sign * distance * axis[0],
            handle[1] + carried_offset[1] + pull_sign * distance * axis[1],
            handle[2] + carried_offset[2],
        ]
        target[2] = max(anchor[2] - 0.065, min(anchor[2] + 0.065, target[2]))
        control(target, orientation)
        pull_ticks += 1

        if pull_ticks >= 24:
            updated = sdk.get_state()
            progress = hinge(updated) - pull_start
            if progress < 0.004:
                pull_sign = -pull_sign
            pull_start = hinge(updated)
            pull_ticks = 0

        if current_hinge < 0.06 and current_latch < 0.30:
            pulling = False
            anchor = list(state["eef_pos"])
            base_orientation = list(state["eef_quat"])
            axis = normal(state)
            twist_axis = [axis[0], axis[1], 0.0]
            twist = 0.0
            best_twist = 0.0
            previous_latch = current_latch
            latch_ticks = 0
            latch_stalled = 0

    state = sdk.get_state()
    return {
        "status": "done" if hinge(state) > 0.30 else "failed",
        "hinge_qpos": hinge(state),
        "handle_qpos": latch(state),
        "best_hinge_qpos": best_hinge,
        "best_handle_qpos": best_latch,
        "steps_used": state["steps_used"],
    }
