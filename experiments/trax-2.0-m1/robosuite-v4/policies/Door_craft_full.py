def run_policy(sdk):
    def clamp(value, bound):
        return max(-bound, min(bound, value))

    def joint(state, name):
        value = state[name]
        return 0.0 if value is None else float(value)

    def finished(state):
        return joint(state, "hinge_qpos") > 0.30

    def remaining(state):
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def control(state, offset, motion, strength=1.0):
        handle = state["objects"]["handle"]["pos"]
        eef = state["eef_pos"]
        action = [
            clamp(
                18.0 * (handle[i] + offset[i] - eef[i])
                + strength * motion[i],
                0.8,
            )
            for i in range(3)
        ]
        action += [strength * motion[i] for i in range(3, 6)]
        action += [1.0]
        sdk.step_raw(action, steps=1)

    state = sdk.get_state()
    if finished(state):
        return {"status": "done", "hinge_qpos": joint(state, "hinge_qpos")}

    probes = [
        [0.0, 0.0, 0.0, 0.45, 0.0, 0.0],
        [0.0, 0.0, 0.0, -0.45, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.45, 0.0],
        [0.0, 0.0, 0.0, 0.0, -0.45, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.45],
        [0.0, 0.0, 0.0, 0.0, 0.0, -0.45],
        [0.0, 0.0, -0.35, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.35, 0.0, 0.0, 0.0],
        [0.35, 0.0, 0.0, 0.0, 0.0, 0.0],
        [-0.35, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.35, 0.0, 0.0, 0.0, 0.0],
        [0.0, -0.35, 0.0, 0.0, 0.0, 0.0],
    ]

    for attempt in range(3):
        state = sdk.get_state()
        if finished(state):
            return {"status": "done", "hinge_qpos": joint(state, "hinge_qpos")}
        if remaining(state) < 35:
            break

        sdk.set_gripper("open", steps=min(8, remaining(state)))
        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        yaw = state["objects"]["handle"]["yaw"] + attempt * pi / 2.0

        sdk.move_to(
            [handle[0], handle[1], handle[2] + 0.09],
            yaw=yaw,
            gripper="open",
            max_steps=min(25, remaining(state)),
            tol=0.015,
        )
        state = sdk.get_state()
        if remaining(state) < 15:
            break
        handle = list(state["objects"]["handle"]["pos"])
        sdk.move_to(
            handle,
            yaw=yaw,
            gripper="open",
            max_steps=min(25, remaining(state)),
            tol=0.007,
        )
        state = sdk.get_state()
        if remaining(state) < 10:
            break
        sdk.set_gripper("close", steps=min(10, remaining(state)))
        state = sdk.get_state()

        offset = [
            clamp(state["eef_pos"][i] - state["objects"]["handle"]["pos"][i], 0.025)
            for i in range(3)
        ]
        best_motion = list(probes[0])
        best_gain = -1.0
        unlocked = False

        for motion in probes:
            state = sdk.get_state()
            if finished(state):
                return {"status": "done", "hinge_qpos": joint(state, "hinge_qpos")}
            if remaining(state) < 12:
                break
            if abs(joint(state, "handle_qpos")) >= 0.40:
                unlocked = True
                break

            before = abs(joint(state, "handle_qpos"))
            for tick in range(4):
                state = sdk.get_state()
                control(state, offset, motion)
            state = sdk.get_state()
            gain = abs(joint(state, "handle_qpos")) - before
            if gain > best_gain:
                best_gain = gain
                best_motion = list(motion)

            if gain < -0.015:
                reverse = [-value for value in motion]
                for tick in range(3):
                    state = sdk.get_state()
                    if remaining(state) <= 0:
                        break
                    control(state, offset, reverse)

        for tick in range(24):
            state = sdk.get_state()
            if finished(state):
                return {"status": "done", "hinge_qpos": joint(state, "hinge_qpos")}
            if remaining(state) < 8:
                break
            if abs(joint(state, "handle_qpos")) >= 0.40:
                unlocked = True
                break
            control(state, offset, best_motion)

        state = sdk.get_state()
        reference_hinge = joint(state, "hinge_qpos")
        anchor_hinge = reference_hinge
        anchor_handle = list(state["objects"]["handle"]["pos"])
        anchor_latch = joint(state, "handle_qpos")
        direction_x = 1.0 / sqrt(2.0)
        direction_y = 1.0 / sqrt(2.0)
        direction_hinge = reference_hinge
        press_depth = 0.0
        pull_lead = 0.0
        best_hinge = reference_hinge
        best_latch = joint(state, "handle_qpos")
        stagnant = 0
        lost_contact = 0
        release_target = 1.08
        servo = False
        hinge_stagnant = 0
        progress_hinge = reference_hinge
        servo_anchor = list(anchor_handle)
        servo_hinge = reference_hinge
        servo_latch = anchor_latch
        servo_x = direction_x
        servo_y = direction_y
        servo_z = offset[2]
        servo_lead = 0.018

        for tick in range(400):
            state = sdk.get_state()
            if finished(state):
                return {
                    "status": "done",
                    "hinge_qpos": joint(state, "hinge_qpos"),
                    "steps_used": state["steps_used"],
                }
            if remaining(state) <= 0:
                break

            hinge = joint(state, "hinge_qpos")
            latch = joint(state, "handle_qpos")
            handle = state["objects"]["handle"]["pos"]
            eef = state["eef_pos"]

            if hinge > progress_hinge + 0.002:
                progress_hinge = hinge
                hinge_stagnant = 0
            else:
                hinge_stagnant += 1

            angle = hinge - direction_hinge
            tx = cos(angle) * direction_x - sin(angle) * direction_y
            ty = sin(angle) * direction_x + cos(angle) * direction_y

            hinge_change = hinge - anchor_hinge
            if hinge_change > 0.012 and abs(latch - anchor_latch) < 0.10:
                dx = handle[0] - anchor_handle[0]
                dy = handle[1] - anchor_handle[1]
                distance = hypot(dx, dy)
                if distance > 0.002 and dx * tx + dy * ty > 0.0:
                    tx = 0.65 * tx + 0.35 * dx / distance
                    ty = 0.65 * ty + 0.35 * dy / distance
                    norm = hypot(tx, ty)
                    tx /= norm
                    ty /= norm
                    direction_x = tx
                    direction_y = ty
                    direction_hinge = hinge
                anchor_hinge = hinge
                anchor_handle = list(handle)
                anchor_latch = latch
            elif abs(latch - anchor_latch) >= 0.10 or hinge_change < -0.01:
                anchor_hinge = hinge
                anchor_handle = list(handle)
                anchor_latch = latch

            if (
                not servo
                and progress_hinge > 0.04
                and (hinge_stagnant >= 20 or hinge < progress_hinge - 0.012)
            ):
                servo = True
                servo_x = tx
                servo_y = ty
                servo_anchor = list(handle)
                servo_hinge = hinge
                servo_latch = latch
                servo_z = clamp(eef[2] - handle[2], 0.075)
                servo_lead = 0.018

            if servo:
                change = hinge - servo_hinge
                latch_change = abs(latch - servo_latch)
                if abs(change) >= 0.006 and latch_change < 0.06:
                    dx = (handle[0] - servo_anchor[0]) / change
                    dy = (handle[1] - servo_anchor[1]) / change
                    radius = hypot(dx, dy)
                    if 0.08 < radius < 0.8:
                        measured_x = dx / radius
                        measured_y = dy / radius
                        if measured_x * servo_x + measured_y * servo_y > 0.25:
                            servo_x = 0.35 * servo_x + 0.65 * measured_x
                            servo_y = 0.35 * servo_y + 0.65 * measured_y
                            norm = hypot(servo_x, servo_y)
                            servo_x /= norm
                            servo_y /= norm
                    servo_anchor = list(handle)
                    servo_hinge = hinge
                    servo_latch = latch
                elif latch_change >= 0.06:
                    servo_anchor = list(handle)
                    servo_hinge = hinge
                    servo_latch = latch

                desired_z = offset[2] - 0.012
                if hinge < 0.04 and latch < 1.02:
                    desired_z = offset[2] - 0.035
                servo_z += clamp(desired_z - servo_z, 0.0015)

                relative_x = eef[0] - handle[0] - offset[0]
                relative_y = eef[1] - handle[1] - offset[1]
                along = relative_x * servo_x + relative_y * servo_y
                cross = -relative_x * servo_y + relative_y * servo_x

                contact_error = sqrt(
                    relative_x * relative_x
                    + relative_y * relative_y
                    + (eef[2] - handle[2] - servo_z) ** 2
                )
                if contact_error > 0.065 or abs(cross) > 0.028:
                    servo_lead = max(0.010, servo_lead - 0.002)
                    feed = 0.0
                else:
                    desired_lead = 0.022 if hinge_stagnant < 15 else 0.036
                    servo_lead += clamp(desired_lead - servo_lead, 0.0008)
                    feed = 0.16

                if along > 0.050:
                    feed = 0.0

                target = [
                    handle[0] + offset[0] + servo_x * servo_lead,
                    handle[1] + offset[1] + servo_y * servo_lead,
                    handle[2] + servo_z,
                ]
                action = [
                    clamp(26.0 * (target[0] - eef[0]) + feed * servo_x, 0.50),
                    clamp(26.0 * (target[1] - eef[1]) + feed * servo_y, 0.50),
                    clamp(26.0 * (target[2] - eef[2]), 0.40),
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                ]
                sdk.step_raw(action, steps=1)
                continue

            latch_error = release_target - latch
            press_depth = max(
                0.0,
                min(0.075, press_depth + clamp(0.003 * latch_error, 0.0015)),
            )
            press_force = max(-0.12, min(0.65, 1.4 * latch_error))

            released = latch >= 1.0051
            if released:
                pull_lead = min(0.065, pull_lead + 0.002)
            elif latch >= 0.90:
                pull_lead = min(0.018, pull_lead + 0.0005)
            else:
                pull_lead = max(0.0, pull_lead - 0.003)

            tracking_offset = [
                offset[0] + tx * pull_lead,
                offset[1] + ty * pull_lead,
                offset[2] - press_depth,
            ]
            pull_force = 0.26 if released else 0.06 if latch >= 0.90 else 0.0
            motion = [
                tx * pull_force,
                ty * pull_force,
                -press_force,
                0.0,
                0.0,
                0.0,
            ]

            contact_distance = sqrt(sum(
                (eef[i] - handle[i]) * (eef[i] - handle[i])
                for i in range(3)
            ))
            if contact_distance > 0.115:
                lost_contact += 1
                pull_lead *= 0.8
                press_depth *= 0.9
                motion = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                tracking_offset = list(offset)
            else:
                lost_contact = 0

            if hinge > best_hinge + 0.002 or latch > best_latch + 0.025:
                stagnant = 0
                best_hinge = max(best_hinge, hinge)
                best_latch = max(best_latch, latch)
            else:
                stagnant += 1

            if lost_contact >= 12:
                break
            if stagnant >= 85 and hinge < 0.04:
                break

            control(state, tracking_offset, motion)

    state = sdk.get_state()
    return {
        "status": "done" if finished(state) else "failed",
        "hinge_qpos": joint(state, "hinge_qpos"),
        "handle_qpos": joint(state, "handle_qpos"),
        "steps_used": state["steps_used"],
    }
