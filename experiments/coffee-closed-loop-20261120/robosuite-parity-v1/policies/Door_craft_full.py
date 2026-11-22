def run_policy(sdk):
    state = sdk.get_state()
    limit = min(400, state["max_steps"])
    best_hinge = state["hinge_qpos"] or 0.0
    best_handle = state["handle_qpos"] or 0.0

    def remaining():
        return max(0, limit - sdk.get_state()["steps_used"])

    def move(target, yaw, grip, ticks, tolerance=0.004):
        budget = min(ticks, remaining())
        if budget > 0:
            return sdk.move_to(
                target,
                yaw=yaw,
                gripper=grip,
                max_steps=budget,
                tol=tolerance,
            )
        return None

    for attempt in range(3):
        state = sdk.get_state()
        if (state["hinge_qpos"] or 0.0) > 0.30:
            break
        if remaining() < 35:
            break

        yaw = state["objects"]["handle"]["yaw"]
        if attempt == 1:
            yaw += pi / 2.0

        if attempt > 0:
            sdk.set_gripper("open", steps=min(4, remaining()))
            state = sdk.get_state()
            eef = state["eef_pos"]
            move(
                [eef[0], eef[1] - 0.055, eef[2] + 0.035],
                yaw,
                "open",
                10,
            )

        state = sdk.get_state()
        handle = state["objects"]["handle"]["pos"]
        move(
            [handle[0], handle[1] - 0.065, handle[2] + 0.010],
            yaw,
            "open",
            24,
            0.008,
        )

        state = sdk.get_state()
        handle = state["objects"]["handle"]["pos"]
        insertion = 0.008 if attempt != 2 else 0.016
        move(
            [handle[0], handle[1] + insertion, handle[2]],
            yaw,
            "open",
            23,
        )

        if remaining() == 0:
            break
        sdk.set_gripper("close", steps=min(9, remaining()))

        state = sdk.get_state()
        anchor = list(state["eef_pos"])
        initial_hinge = state["hinge_qpos"] or 0.0
        previous_hinge = initial_hinge
        depression = 0.0
        travel = 0.0
        activated = False
        cleared = initial_hinge > 0.075
        stalled = 0
        contact_failures = 0
        peak_angle = state["handle_qpos"] or 0.0
        swing = max(0.0, initial_hinge)
        direction_x = -sin(swing)
        direction_y = -cos(swing)

        for segment in range(64):
            state = sdk.get_state()
            hinge = state["hinge_qpos"] or 0.0
            angle = state["handle_qpos"] or 0.0
            eef = state["eef_pos"]
            best_hinge = max(best_hinge, hinge)
            best_handle = max(best_handle, angle)
            peak_angle = max(peak_angle, angle)

            if hinge > 0.30 or remaining() == 0:
                break

            # A brief handle rotation is sufficient to start pulling.
            if angle > 0.55 or hinge > initial_hinge + 0.012:
                activated = True
            if hinge > 0.075:
                cleared = True

            achieved_down = max(0.0, anchor[2] - eef[2])
            if not cleared:
                if angle < 0.85:
                    depression += 0.006
                elif angle < 1.10:
                    depression += 0.002
                elif angle > 1.35:
                    depression -= 0.002
                depression = min(
                    0.105,
                    max(0.0, depression),
                    achieved_down + 0.025,
                )
            elif angle > 1.35:
                depression = max(0.020, depression - 0.002)

            achieved = max(
                0.0,
                (eef[0] - anchor[0]) * direction_x
                + (eef[1] - anchor[1]) * direction_y,
            )

            if cleared:
                increment = 0.012
                lead = 0.035
            elif activated:
                increment = 0.007 if angle >= 0.45 else 0.003
                lead = 0.025
            else:
                increment = 0.002
                lead = 0.014

            travel = min(0.32, travel + increment, achieved + lead)

            # Follow a circular opening arc with a small inward preload.
            radius = 0.42
            bounded_travel = min(travel, radius * 0.90)
            sideways = (
                radius
                - sqrt(max(0.0, radius * radius - bounded_travel * bounded_travel))
                + 0.08 * bounded_travel
            )
            target = [
                anchor[0] - sideways * cos(swing) - travel * sin(swing),
                anchor[1] + sideways * sin(swing) - travel * cos(swing),
                anchor[2] - depression,
            ]

            displacement = [target[i] - eef[i] for i in range(3)]
            distance = sqrt(sum(value * value for value in displacement))
            if distance > 0.025:
                target = [
                    eef[i] + displacement[i] * 0.025 / distance
                    for i in range(3)
                ]

            move(
                target,
                yaw - (hinge - initial_hinge),
                "close",
                4,
                0.002,
            )

            current = sdk.get_state()
            new_hinge = current["hinge_qpos"] or 0.0
            new_angle = current["handle_qpos"] or 0.0
            best_hinge = max(best_hinge, new_hinge)
            best_handle = max(best_handle, new_angle)
            peak_angle = max(peak_angle, new_angle)

            if new_angle > 0.55 or new_hinge > initial_hinge + 0.012:
                activated = True
            if new_hinge > 0.075:
                cleared = True
            if new_hinge > 0.30:
                break

            if new_hinge > previous_hinge + 0.0005:
                stalled = 0
            else:
                stalled += 1
            previous_hinge = new_hinge

            handle = current["objects"]["handle"]["pos"]
            eef = current["eef_pos"]
            separation = sqrt(
                sum((eef[i] - handle[i]) ** 2 for i in range(3))
            )
            if separation > 0.115 and new_angle < 0.20:
                contact_failures += 1
            else:
                contact_failures = 0

            if contact_failures >= 3:
                break
            if segment >= 18 and peak_angle < 0.30 and stalled >= 12:
                break
            if segment >= 30 and stalled >= 22 and not cleared:
                break

    state = sdk.get_state()
    hinge = state["hinge_qpos"] or 0.0
    return {
        "status": "done" if hinge > 0.30 else "failed",
        "hinge_qpos": hinge,
        "handle_qpos": state["handle_qpos"],
        "best_hinge_qpos": max(best_hinge, hinge),
        "best_handle_qpos": max(best_handle, state["handle_qpos"] or 0.0),
        "steps_used": state["steps_used"],
    }
