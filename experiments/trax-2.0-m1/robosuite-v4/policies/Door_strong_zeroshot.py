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
            clamp(18.0 * (handle[i] + offset[i] - eef[i])
                  + strength * motion[i], 0.8)
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
        previous_hinge = joint(state, "hinge_qpos")
        stagnant = 0

        for tick in range(110):
            state = sdk.get_state()
            if finished(state):
                return {
                    "status": "done",
                    "hinge_qpos": joint(state, "hinge_qpos"),
                    "steps_used": state["steps_used"],
                }
            if remaining(state) <= 0:
                break

            handle = state["objects"]["handle"]["pos"]
            door = state["objects"]["door"]["pos"]
            rx = handle[0] - door[0]
            ry = handle[1] - door[1]
            radius = hypot(rx, ry)
            if radius < 0.04:
                rx = handle[0] - state["eef_pos"][0]
                ry = handle[1] - state["eef_pos"][1]
                radius = hypot(rx, ry)
            if radius < 0.01:
                break

            latch_strength = clamp(
                (0.48 - abs(joint(state, "handle_qpos"))) * 5.0, 1.0
            )
            latch_strength = max(0.0, latch_strength)
            motion = [value * latch_strength for value in best_motion]
            pull = 0.32 if unlocked or tick > 15 else 0.15
            motion[0] += -ry * pull / radius
            motion[1] += rx * pull / radius
            control(state, offset, motion)

            hinge = joint(state, "hinge_qpos")
            if hinge > previous_hinge + 0.003:
                stagnant = 0
                previous_hinge = hinge
            else:
                stagnant += 1
            if stagnant >= 35:
                break

    state = sdk.get_state()
    return {
        "status": "done" if finished(state) else "failed",
        "hinge_qpos": joint(state, "hinge_qpos"),
        "handle_qpos": joint(state, "handle_qpos"),
        "steps_used": state["steps_used"],
    }
