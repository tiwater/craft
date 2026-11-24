def run_policy(sdk):
    def clip(value, limit):
        return max(-limit, min(limit, value))

    def remaining(state):
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def successful(state):
        return state["hinge_qpos"] is not None and state["hinge_qpos"] > 0.30

    def latch_angle(state):
        return state["handle_qpos"] if state["handle_qpos"] is not None else 0.0

    def hinge_angle(state):
        return state["hinge_qpos"] if state["hinge_qpos"] is not None else 0.0

    def door_normal(state):
        x, y, z, w = state["objects"]["door"]["quat"]
        normal = [
            2.0 * (x * y - z * w),
            1.0 - 2.0 * (x * x + z * z),
            0.0
        ]
        length = hypot(normal[0], normal[1])
        if length < 0.1:
            handle = state["objects"]["handle"]["pos"]
            door = state["objects"]["door"]["pos"]
            normal = [-(handle[1] - door[1]), handle[0] - door[0], 0.0]
            length = hypot(normal[0], normal[1])
        if length < 0.001:
            return [0.0, 1.0, 0.0]
        normal = [normal[0] / length, normal[1] / length, 0.0]
        if normal[1] < 0.0:
            normal = [-value for value in normal]
        return normal

    def move(position, yaw, grip, budget):
        state = sdk.get_state()
        ticks = min(budget, remaining(state))
        if ticks > 0 and not successful(state):
            sdk.move_to(
                position, yaw=yaw, gripper=grip,
                max_steps=ticks, tol=0.008
            )

    turn_sign = 1.0

    for attempt in range(3):
        state = sdk.get_state()
        if successful(state) or remaining(state) < 40:
            break

        normal = door_normal(state)
        yaw = atan2(normal[1], normal[0])
        if attempt == 1:
            yaw += pi / 2.0

        handle = state["objects"]["handle"]["pos"]
        approach = [handle[i] + 0.07 * normal[i] for i in range(3)]
        approach[2] += 0.075
        move(approach, yaw, "open", 28)

        state = sdk.get_state()
        handle = state["objects"]["handle"]["pos"]
        approach = [handle[i] + 0.022 * normal[i] for i in range(3)]
        move(approach, yaw, "open", 20)

        state = sdk.get_state()
        move(list(state["objects"]["handle"]["pos"]), yaw, "open", 18)

        state = sdk.get_state()
        if successful(state) or remaining(state) <= 0:
            break
        sdk.set_gripper("close", steps=min(12, remaining(state)))

        state = sdk.get_state()
        handle = state["objects"]["handle"]["pos"]
        offset = [
            clip(state["eef_pos"][i] - handle[i], 0.012)
            for i in range(3)
        ]
        offset[2] = min(0.0, offset[2])

        best_latch = latch_angle(state)
        probe_latch = best_latch
        best_hinge = hinge_angle(state)
        stagnant = 0
        pull_stalled = 0
        lost_ticks = 0
        released = False
        recovering = False
        stable_latch = 0
        pull_ramp = 0.0
        preload = 0.035
        previous_latch = best_latch
        previous_action = None
        pull_origin = list(handle)
        waypoint = list(state["eef_pos"])

        for tick in range(400):
            state = sdk.get_state()
            if successful(state) or remaining(state) <= 0:
                break

            latch = latch_angle(state)
            hinge = hinge_angle(state)
            handle = state["objects"]["handle"]["pos"]
            eef = state["eef_pos"]
            normal = door_normal(state)
            latch_rate = latch - previous_latch
            previous_latch = latch

            if latch > best_latch + 0.008:
                best_latch = latch
                stagnant = 0
            else:
                stagnant += 1

            if not released and latch >= 1.45:
                released = True
                pull_origin = list(handle)
                waypoint = list(eef)
                preload = max(0.018, min(0.045, handle[2] - eef[2]))

            if not released:
                if tick == 12:
                    if latch < probe_latch + 0.015:
                        turn_sign = -turn_sign
                    probe_latch = latch
                elif tick == 28 and latch < probe_latch - 0.025:
                    turn_sign = -turn_sign

                target = [handle[i] + offset[i] for i in range(3)]
                depth = min(0.065, 0.025 + 0.0006 * stagnant)
                if latch >= 1.15:
                    depth = max(depth, 0.045)
                target[2] -= depth
                translation = [
                    clip((target[i] - eef[i]) / 0.05, 0.70)
                    for i in range(3)
                ]
                twist = turn_sign * 0.20
            else:
                # Release is reversible: recover latch pressure before
                # allowing a low-angle handle to carry the full pull.
                if latch < 1.40:
                    recovering = True
                    stable_latch = 0
                elif recovering:
                    if latch >= 1.48:
                        stable_latch += 1
                    else:
                        stable_latch = 0
                    if stable_latch >= 3:
                        recovering = False

                ramp_target = 0.12 if recovering else 1.0
                if latch_rate < -0.025:
                    ramp_target = min(ramp_target, 0.35)
                pull_ramp += clip(ramp_target - pull_ramp, 0.04)

                angle = max(0.0, hinge)
                direction_x = max(0.28, min(0.48, 0.28 + 0.45 * sin(angle)))
                direction_y = max(0.85, cos(angle))
                length = hypot(direction_x, direction_y)
                direction_x /= length
                direction_y /= length

                # Advance through the full relative arc, while limiting
                # each commanded waypoint's lead over the live handle.
                travel_y = max(0.0, handle[1] - pull_origin[1])
                speed = 0.0040 if travel_y < 0.18 else 0.0032
                speed += min(0.0010, 0.00001 * pull_stalled)
                waypoint[0] += speed * direction_x * pull_ramp
                waypoint[1] += speed * direction_y * pull_ramp

                lead_x = 0.012 + 0.020 * pull_ramp
                lead_y = 0.015 + 0.040 * pull_ramp
                base_x = handle[0] + offset[0]
                base_y = handle[1] + offset[1]
                waypoint[0] = max(
                    base_x, min(base_x + lead_x, waypoint[0])
                )
                waypoint[1] = max(
                    base_y, min(base_y + lead_y, waypoint[1])
                )

                if recovering:
                    waypoint[0] = min(waypoint[0], base_x + 0.010)
                    waypoint[1] = min(waypoint[1], base_y + 0.018)

                # Avoid the abrupt upward unloading at first release.
                # Keep shallow downward contact during travel and restore
                # the proven downward stroke if the latch unwinds.
                desired_preload = 0.024
                if latch < 1.45:
                    desired_preload += min(0.026, 0.075 * (1.45 - latch))
                if latch_rate < -0.015:
                    desired_preload = max(desired_preload, 0.035)
                preload += clip(desired_preload - preload, 0.0007)
                target_z = handle[2] - preload

                translation = [
                    clip((waypoint[0] - eef[0]) / 0.05, 0.80),
                    clip((waypoint[1] - eef[1]) / 0.05, 0.95),
                    clip((target_z - eef[2]) / 0.05, 0.55)
                ]

                # Maintain rotation instead of reducing torque immediately
                # after the first threshold crossing.
                twist_strength = (
                    0.15 + 0.65 * (1.53 - latch)
                    - 1.5 * latch_rate
                )
                twist = turn_sign * max(0.06, min(0.27, twist_strength))

            rotation = [clip(twist * normal[i], 0.28) for i in range(3)]
            requested = translation + rotation
            if previous_action is None:
                action = requested
            else:
                action = [
                    previous_action[i] + clip(
                        requested[i] - previous_action[i],
                        0.09 if i < 3 else 0.045
                    )
                    for i in range(6)
                ]

            sdk.step_raw(action + [1.0], steps=1)
            previous_action = action

            state = sdk.get_state()
            if successful(state):
                break

            current_hinge = hinge_angle(state)
            if current_hinge > best_hinge + 0.002:
                best_hinge = current_hinge
                pull_stalled = 0
            elif released:
                pull_stalled += 1

            current_handle = state["objects"]["handle"]["pos"]
            eef = state["eef_pos"]
            separation = sqrt(sum(
                (eef[i] - current_handle[i]) ** 2 for i in range(3)
            ))
            empty = state["gripper_width"] < 0.010

            if separation > 0.11 or (empty and separation > 0.065):
                lost_ticks += 1
            else:
                lost_ticks = 0

            if released:
                # A stalled hinge alone never terminates the pull.
                if lost_ticks >= 16 and remaining(state) >= 100:
                    break
            else:
                if stagnant >= 45:
                    if empty or separation > 0.085 or best_latch < 0.20:
                        break
                if stagnant >= 80:
                    break

        state = sdk.get_state()
        if successful(state) or remaining(state) < 40:
            break
        if attempt < 2:
            sdk.set_gripper("open", steps=min(5, remaining(state)))

    state = sdk.get_state()
    return {
        "status": "done" if successful(state) else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "steps_used": state["steps_used"]
    }
