def run_policy(sdk):
    def clip(value, limit):
        return max(-limit, min(limit, value))

    def remaining(state):
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def successful(state):
        return state["hinge_qpos"] is not None and state["hinge_qpos"] > 0.30

    def door_axis(state):
        x, y, z, w = state["objects"]["door"]["quat"]
        axis = [
            2.0 * (x * y - z * w),
            1.0 - 2.0 * (x * x + z * z),
            0.0
        ]
        length = max(0.001, hypot(axis[0], axis[1]))
        return [axis[0] / length, axis[1] / length, 0.0]

    def tangent(state, fallback):
        handle = state["objects"]["handle"]["pos"]
        door = state["objects"]["door"]["pos"]
        rx = handle[0] - door[0]
        ry = handle[1] - door[1]
        length = hypot(rx, ry)
        if length > 0.06:
            return [-ry / length, rx / length, 0.0]
        return list(fallback)

    def move(target, yaw, grip, ticks):
        state = sdk.get_state()
        budget = min(ticks, remaining(state))
        if budget > 0 and not successful(state):
            sdk.move_to(
                target, yaw=yaw, gripper=grip,
                max_steps=budget, tol=0.008
            )

    for attempt in range(2):
        state = sdk.get_state()
        if successful(state) or remaining(state) < 55:
            break

        handle = state["objects"]["handle"]["pos"]
        axis = door_axis(state)
        if sum((state["eef_pos"][i] - handle[i]) * axis[i]
               for i in range(3)) < 0.0:
            axis = [-value for value in axis]

        yaw = atan2(axis[1], axis[0])
        if attempt == 1:
            yaw += pi / 2.0

        target = [handle[i] + 0.07 * axis[i] for i in range(3)]
        target[2] += 0.065
        move(target, yaw, "open", 24)

        state = sdk.get_state()
        handle = state["objects"]["handle"]["pos"]
        target = [handle[i] + 0.018 * axis[i] for i in range(3)]
        move(target, yaw, "open", 20)

        state = sdk.get_state()
        move(list(state["objects"]["handle"]["pos"]), yaw, "open", 16)

        state = sdk.get_state()
        if remaining(state) <= 0:
            break
        sdk.set_gripper("close", steps=min(12, remaining(state)))

        state = sdk.get_state()
        handle = state["objects"]["handle"]["pos"]
        offset = [state["eef_pos"][i] - handle[i] for i in range(3)]
        previous_axis = list(axis)
        previous_hinge = state["hinge_qpos"] or 0.0
        turn_sign = 1.0
        turn_confirmed = False
        latch_checkpoint = state["handle_qpos"] or 0.0
        best_latch = latch_checkpoint
        hinge_checkpoint = previous_hinge
        lead = 0.0
        pull_sign = 1.0
        pull_ticks = 0
        stalled = 0
        reversals = 0
        clearance = False

        for tick in range(330):
            state = sdk.get_state()
            if successful(state) or remaining(state) <= 0:
                break

            handle = state["objects"]["handle"]["pos"]
            latch = state["handle_qpos"] or 0.0
            hinge = state["hinge_qpos"] or 0.0
            axis = door_axis(state)
            if sum(axis[i] * previous_axis[i] for i in range(3)) < 0.0:
                axis = [-value for value in axis]
            previous_axis = list(axis)

            angle = hinge - previous_hinge
            previous_hinge = hinge
            offset = [
                cos(angle) * offset[0] - sin(angle) * offset[1],
                sin(angle) * offset[0] + cos(angle) * offset[1],
                offset[2]
            ]
            best_latch = max(best_latch, latch)

            if tick % 12 == 11:
                progress = latch - latch_checkpoint
                if progress > 0.025:
                    turn_confirmed = True
                elif not turn_confirmed and latch < 0.50:
                    turn_sign = -turn_sign
                latch_checkpoint = latch

            if latch >= 0.60:
                clearance = True
            elif latch < 0.35 and hinge < 0.08:
                clearance = False

            if clearance or hinge > 0.08:
                lead = min(0.16, lead + 0.006)
            else:
                lead = max(0.0, lead - 0.012)

            pull = tangent(state, axis)
            twist_error = 0.90 - latch
            twist = turn_sign * clip(1.8 * twist_error, 0.60)
            if abs(twist_error) < 0.04:
                twist = 0.0

            target = [
                handle[i] + offset[i] + pull_sign * lead * pull[i]
                for i in range(3)
            ]
            action = [
                clip((target[i] - state["eef_pos"][i]) / 0.05, 1.0)
                for i in range(3)
            ]
            rotation = [twist * axis[i] for i in range(3)]
            rotation[2] += clip(angle / 0.5, 0.25)
            action += [clip(value, 0.65) for value in rotation]
            action += [1.0]
            sdk.step_raw(action, steps=1)

            state = sdk.get_state()
            if successful(state):
                break

            if lead >= 0.05:
                pull_ticks += 1

            if pull_ticks >= 18:
                current_hinge = state["hinge_qpos"] or 0.0
                progress = current_hinge - hinge_checkpoint
                if progress > 0.005:
                    stalled = 0
                else:
                    stalled += 1

                if progress < -0.008 or stalled >= 2:
                    pull_sign = -pull_sign
                    lead = 0.0
                    stalled = 0
                    reversals += 1

                hinge_checkpoint = current_hinge
                pull_ticks = 0

            separation = sqrt(sum(
                (state["eef_pos"][i] -
                 state["objects"]["handle"]["pos"][i]) ** 2
                for i in range(3)
            ))
            if separation > 0.18:
                break

            if attempt == 0 and remaining(state) > 120:
                if tick >= 80 and best_latch < 0.50:
                    break
                if reversals >= 2:
                    break

        state = sdk.get_state()
        if successful(state):
            break
        if remaining(state) > 0:
            sdk.set_gripper("open", steps=min(5, remaining(state)))

    state = sdk.get_state()
    return {
        "status": "done" if successful(state) else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "steps_used": state["steps_used"]
    }
