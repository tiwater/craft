def run_policy(sdk):
    def remaining(state):
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def joint(state, name):
        value = state[name]
        return 0.0 if value is None else value

    def opened(state):
        return joint(state, "hinge_qpos") > 0.30

    def clamp(value, low, high):
        return max(low, min(high, value))

    def move(target, yaw, action, ticks):
        state = sdk.get_state()
        if remaining(state) > 0 and not opened(state):
            sdk.move_to(
                target,
                yaw=yaw,
                gripper=action,
                max_steps=min(ticks, remaining(state)),
                tol=0.008,
            )

    def grip(action, ticks):
        state = sdk.get_state()
        if remaining(state) > 0 and not opened(state):
            sdk.set_gripper(action, steps=min(ticks, remaining(state)))

    state = sdk.get_state()
    best_hinge = joint(state, "hinge_qpos")
    best_handle = joint(state, "handle_qpos")
    attempts = 0

    # Try a transverse grasp first. Reserve time for contact corrections
    # instead of spending most of the horizon on yaw changes and retreats.
    yaw_offsets = [-pi / 2.0, 0.0, pi / 2.0]
    capture_offsets = [0.0, 0.012, -0.010]

    while remaining(state) > 12 and not opened(state) and attempts < 3:
        handle = list(state["objects"]["handle"]["pos"])
        base_yaw = state["objects"]["handle"]["yaw"]
        if base_yaw is None:
            base_yaw = 0.0
        yaw = base_yaw + yaw_offsets[attempts]
        yaw = atan2(sin(yaw), cos(yaw))
        capture_z = capture_offsets[attempts]
        attempts += 1

        grip("open", 4)

        state = sdk.get_state()
        eef = list(state["eef_pos"])
        handle = list(state["objects"]["handle"]["pos"])

        # Retreat toward the accessible face before changing height.
        if attempts > 1:
            move(
                [eef[0], eef[1] - 0.045, eef[2] + 0.025],
                state["eef_yaw"], "open", 10,
            )

        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        move(
            [handle[0], handle[1] - 0.075, handle[2] + 0.035],
            yaw, "open", 26,
        )

        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        move(
            [handle[0], handle[1] - 0.055, handle[2] + capture_z],
            yaw, "open", 12,
        )

        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        move(
            [handle[0], handle[1] + 0.003, handle[2] + capture_z],
            yaw, "open", 20,
        )
        grip("close", 8)

        state = sdk.get_state()
        progress_hinge = joint(state, "hinge_qpos")
        progress_handle = joint(state, "handle_qpos")
        stagnant = 0
        engagement_ticks = 0
        corrections = 0
        tangent = [-0.30, -0.954]
        latch_seen = progress_handle > 0.45

        while remaining(state) > 0 and not opened(state):
            handle = list(state["objects"]["handle"]["pos"])
            eef = list(state["eef_pos"])
            hinge = joint(state, "hinge_qpos")
            latch = joint(state, "handle_qpos")
            latch_seen = latch_seen or latch > 0.45

            # The handle itself moves during depression. Follow its live
            # horizontal position rather than pulling away from a fixed anchor.
            if hinge > 0.045:
                pull = 0.038
                depression = 0.012 if latch > 0.55 else 0.025
            elif latch > 0.55:
                pull = 0.032
                depression = 0.016
            elif latch > 0.25:
                pull = 0.020
                depression = 0.028
            else:
                pull = 0.006
                depression = 0.030

            target_x = handle[0] + tangent[0] * pull
            target_y = handle[1] - 0.004 + tangent[1] * pull
            target_z = handle[2] + capture_z - depression

            ax = clamp(22.0 * (target_x - eef[0]), -0.65, 0.65)
            ay = clamp(22.0 * (target_y - eef[1]), -0.80, 0.55)
            az = clamp(22.0 * (target_z - eef[2]), -0.65, 0.30)

            # Large separation indicates lost contact, not an unyielding door.
            separation = sqrt(
                (eef[0] - handle[0]) ** 2
                + (eef[1] - handle[1]) ** 2
                + (eef[2] - handle[2]) ** 2
            )
            if separation > 0.105 and stagnant >= 10:
                break

            ticks = min(2, remaining(state))
            sdk.step_raw(
                [ax, ay, az, 0.0, 0.0, 0.0, 1.0],
                steps=ticks,
            )
            after = sdk.get_state()
            next_hinge = joint(after, "hinge_qpos")
            next_latch = joint(after, "handle_qpos")
            best_hinge = max(best_hinge, next_hinge)
            best_handle = max(best_handle, next_latch)
            engagement_ticks += ticks

            hinge_progress = next_hinge > progress_hinge + 0.006
            latch_progress = next_latch > progress_handle + 0.045
            if hinge_progress or latch_progress:
                stagnant = 0
                progress_hinge = max(progress_hinge, next_hinge)
                progress_handle = max(progress_handle, next_latch)
            else:
                stagnant += ticks

            next_handle = after["objects"]["handle"]["pos"]
            dx = next_handle[0] - handle[0]
            dy = next_handle[1] - handle[1]
            distance = hypot(dx, dy)
            if (
                next_hinge > hinge + 0.002
                and abs(next_latch - latch) < 0.025
                and distance > 0.0005
                and dy < 0.0
            ):
                tx = 0.85 * tangent[0] + 0.15 * dx / distance
                ty = 0.85 * tangent[1] + 0.15 * dy / distance
                norm = max(hypot(tx, ty), 0.000001)
                tangent = [tx / norm, ty / norm]

            state = after
            if opened(state):
                break

            # A depressed latch without hinge motion calls for deeper capture.
            # Re-seat locally while there is still budget to use the new grasp.
            if (
                stagnant >= 22
                and latch_seen
                and next_hinge < 0.045
                and corrections == 0
                and remaining(state) > 35
            ):
                corrections += 1
                grip("open", 3)
                state = sdk.get_state()
                handle = list(state["objects"]["handle"]["pos"])
                move(
                    [handle[0], handle[1] + 0.016, handle[2] + capture_z],
                    yaw, "open", 10,
                )
                grip("close", 6)
                state = sdk.get_state()
                stagnant = 0
                continue

            if next_hinge < 0.045 and stagnant >= 36:
                break
            if engagement_ticks >= 70 and next_hinge < 0.025:
                break
            if stagnant >= 48:
                break

        state = sdk.get_state()

    state = sdk.get_state()
    return {
        "status": "done" if opened(state) else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "best_hinge_qpos": max(best_hinge, joint(state, "hinge_qpos")),
        "best_handle_qpos": max(best_handle, joint(state, "handle_qpos")),
        "steps_used": state["steps_used"],
        "attempts": attempts,
    }
