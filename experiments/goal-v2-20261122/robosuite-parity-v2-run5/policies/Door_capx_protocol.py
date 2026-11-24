def run_policy(sdk):
    def clip(value, limit):
        return max(-limit, min(limit, value))

    def remaining(state):
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def success(state):
        return (state["hinge_qpos"] or 0.0) > 0.30

    def distance(a, b):
        return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    def drive(target, grip, ticks, tolerance):
        for tick in range(ticks):
            state = sdk.get_state()
            if success(state) or remaining(state) == 0:
                return
            if distance(target, state["eef_pos"]) < tolerance:
                return
            action = [
                clip(12.0 * (target[i] - state["eef_pos"][i]), 0.8)
                for i in range(3)
            ]
            sdk.step_raw(action + [0.0, 0.0, 0.0, grip], steps=1)

    def actuate(action, ticks):
        state = sdk.get_state()
        budget = min(ticks, remaining(state))
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def handle_axis(state):
        x, y, z, w = state["objects"]["door"]["quat"]
        return [
            2.0 * (x * y - z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z + x * w)
        ]

    attempts = 0
    while attempts < 3:
        state = sdk.get_state()
        if success(state) or remaining(state) < 25:
            break

        actuate("open", 6)
        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        clearance = max(state["eef_pos"][2], handle[2] + 0.075)
        drive(
            [state["eef_pos"][0], state["eef_pos"][1], clearance],
            -1.0, 25, 0.018
        )
        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        drive([handle[0], handle[1], clearance], -1.0, 65, 0.012)

        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        depth = -0.005 if attempts == 0 else -0.015
        target = [handle[0], handle[1], handle[2] + depth]
        drive(target, -1.0, 45, 0.008)

        state = sdk.get_state()
        if success(state):
            break
        if distance(state["eef_pos"], target) > 0.035:
            attempts += 1
            continue

        actuate("close", 12)
        state = sdk.get_state()
        if state["gripper_width"] < 0.009:
            attempts += 1
            continue

        handle = state["objects"]["handle"]["pos"]
        offset = [
            clip(state["eef_pos"][i] - handle[i], 0.02)
            for i in range(3)
        ]
        axis = handle_axis(state)
        goal = state["goal_pos"]
        pull_sign = -1.0
        if goal is not None:
            projection = sum(
                (goal[i] - handle[i]) * axis[i] for i in range(2)
            )
            pull_sign = 1.0 if projection > 0.0 else -1.0

        latch = state["handle_qpos"] or 0.0
        turn_sign = -1.0 if latch < -0.08 else 1.0
        best_latch = abs(latch)
        best_hinge = state["hinge_qpos"] or 0.0
        stagnant = 0
        empty = 0
        turn_reversed = False
        pull_reversed = False
        pull_age = 0

        for tick in range(min(230, remaining(state))):
            state = sdk.get_state()
            if success(state) or remaining(state) == 0:
                break

            handle = state["objects"]["handle"]["pos"]
            hinge = state["hinge_qpos"] or 0.0
            latch = state["handle_qpos"] or 0.0
            axis = handle_axis(state)

            if abs(latch) > 0.16:
                turn_sign = 1.0 if latch > 0.0 else -1.0

            target = [handle[i] + offset[i] for i in range(3)]
            pulling = abs(latch) > 0.24 or hinge > 0.025 or tick > 65
            if pulling:
                pull_age += 1
                amount = min(0.075, 0.012 + 0.001 * pull_age)
                horizontal = max(hypot(axis[0], axis[1]), 0.0001)
                target[0] += pull_sign * amount * axis[0] / horizontal
                target[1] += pull_sign * amount * axis[1] / horizontal

            torque = clip(1.5 * (turn_sign * 0.85 - latch), 0.45)
            if hinge > 0.12:
                torque = clip(0.8 * (turn_sign * 0.55 - latch), 0.22)

            translation = [
                clip(15.0 * (target[i] - state["eef_pos"][i]), 0.65)
                for i in range(3)
            ]
            rotation = [torque * axis[i] for i in range(3)]
            sdk.step_raw(translation + rotation + [1.0], steps=1)

            updated = sdk.get_state()
            new_hinge = updated["hinge_qpos"] or 0.0
            best_latch = max(
                best_latch, abs(updated["handle_qpos"] or 0.0)
            )
            if new_hinge > best_hinge + 0.002:
                best_hinge = new_hinge
                stagnant = 0
            elif pulling:
                stagnant += 1

            if tick >= 30 and best_latch < 0.10 and not turn_reversed:
                turn_sign = -turn_sign
                turn_reversed = True

            if stagnant >= 35 and best_hinge < 0.03 and not pull_reversed:
                pull_sign = -pull_sign
                pull_age = 0
                stagnant = 0
                pull_reversed = True

            empty = empty + 1 if updated["gripper_width"] < 0.009 else 0
            separation = distance(
                updated["eef_pos"],
                updated["objects"]["handle"]["pos"]
            )
            if empty >= 6 or separation > 0.12:
                break
            if tick >= 110 and best_latch < 0.12 and best_hinge < 0.03:
                break

        attempts += 1

    state = sdk.get_state()
    return {
        "status": "done" if success(state) else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "steps_used": state["steps_used"]
    }
