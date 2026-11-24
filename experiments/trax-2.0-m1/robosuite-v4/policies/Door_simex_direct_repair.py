def run_policy(sdk):
    def clamp(value, bound):
        return max(-bound, min(bound, value))

    def joint(state, name):
        value = state[name]
        return 0.0 if value is None else float(value)

    def remaining(state):
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def finished(state):
        return joint(state, "hinge_qpos") > 0.30

    def control(state, offset, lead, rotation):
        handle = state["objects"]["handle"]["pos"]
        eef = state["eef_pos"]
        action = [
            clamp(18.0 * (handle[i] + offset[i] + lead[i] - eef[i]), 1.0)
            for i in range(3)
        ]
        action += [clamp(value, 1.0) for value in rotation]
        action += [1.0]
        sdk.step_raw(action, steps=1)

    motions = [
        [0.85, 0.0, 0.0],
        [-0.85, 0.0, 0.0],
        [0.0, 0.85, 0.0],
        [0.0, -0.85, 0.0],
        [0.0, 0.0, 0.85],
        [0.0, 0.0, -0.85],
    ]

    for attempt in range(3):
        state = sdk.get_state()
        if finished(state) or remaining(state) <= 0:
            break

        sdk.set_gripper("open", steps=min(6, remaining(state)))
        state = sdk.get_state()
        if remaining(state) <= 0:
            break

        handle = list(state["objects"]["handle"]["pos"])
        yaw = state["objects"]["handle"]["yaw"] + attempt * pi / 2.0
        sdk.move_to(
            [handle[0], handle[1], handle[2] + 0.085],
            yaw=yaw,
            gripper="open",
            max_steps=min(25, remaining(state)),
            tol=0.012,
        )
        state = sdk.get_state()
        if remaining(state) <= 0:
            break

        sdk.move_to(
            list(state["objects"]["handle"]["pos"]),
            yaw=yaw,
            gripper="open",
            max_steps=min(30, remaining(state)),
            tol=0.006,
        )
        state = sdk.get_state()
        if remaining(state) <= 0:
            break

        sdk.set_gripper("close", steps=min(12, remaining(state)))
        state = sdk.get_state()
        offset = [
            clamp(state["eef_pos"][i] - state["objects"]["handle"]["pos"][i], 0.025)
            for i in range(3)
        ]

        best_motion = list(motions[0])
        best_rate = -1000.0
        for motion in motions:
            state = sdk.get_state()
            if finished(state) or remaining(state) <= 0:
                break
            before = joint(state, "handle_qpos")
            count = 0
            for tick in range(8):
                state = sdk.get_state()
                if finished(state) or remaining(state) <= 0:
                    break
                control(state, offset, [0.0, 0.0, -0.005], motion)
                count += 1
                if joint(sdk.get_state(), "handle_qpos") >= 0.50:
                    break
            state = sdk.get_state()
            rate = (joint(state, "handle_qpos") - before) / max(1, count)
            if rate > best_rate:
                best_rate = rate
                best_motion = list(motion)
            if joint(state, "handle_qpos") >= 0.50 or rate > 0.006:
                break

        peak_handle = joint(state, "handle_qpos")
        stalled = 0
        for tick in range(35):
            state = sdk.get_state()
            if finished(state) or remaining(state) <= 0:
                break
            if joint(state, "handle_qpos") >= 0.55:
                break
            control(state, offset, [0.0, 0.0, -0.005], best_motion)
            state = sdk.get_state()
            angle = joint(state, "handle_qpos")
            if angle > peak_handle + 0.004:
                peak_handle = angle
                stalled = 0
            else:
                stalled += 1
            if stalled >= 15:
                break

        state = sdk.get_state()
        if finished(state) or remaining(state) <= 0:
            break
        if joint(state, "handle_qpos") < 0.25 and remaining(state) > 120:
            continue

        handle = state["objects"]["handle"]["pos"]
        door = state["objects"]["door"]["pos"]
        rx = handle[0] - door[0]
        ry = handle[1] - door[1]
        radius = hypot(rx, ry)
        if radius > 0.025:
            tangent = [-ry / radius, rx / radius]
        else:
            yaw = state["objects"]["door"]["yaw"]
            tangent = [-cos(yaw), -sin(yaw)]

        candidates = [
            list(tangent),
            [-tangent[0], -tangent[1]],
            [-tangent[1], tangent[0]],
            [tangent[1], -tangent[0]],
        ]
        direction = list(candidates[0])
        selected_hinge = joint(state, "hinge_qpos")
        probe_hinge = selected_hinge
        probe_ticks = 0
        candidate_index = 0
        searching = True
        lead_size = 0.0
        peak_hinge = selected_hinge
        stagnant = 0
        lost = 0
        previous_hinge = selected_hinge
        previous_handle = joint(state, "handle_qpos")
        anchor_hinge = selected_hinge
        anchor_handle = list(handle)

        while remaining(state) > 0 and not finished(state):
            hinge = joint(state, "hinge_qpos")
            angle = joint(state, "handle_qpos")
            turn = hinge - selected_hinge
            dx = cos(turn) * direction[0] - sin(turn) * direction[1]
            dy = sin(turn) * direction[0] + cos(turn) * direction[1]

            latch = clamp(
                5.0 * (0.60 - angle) - 10.0 * (angle - previous_handle),
                1.0,
            )
            rotation = [value * latch for value in best_motion]
            rotation[2] += clamp(10.0 * (hinge - previous_hinge), 0.35)
            previous_hinge = hinge
            previous_handle = angle

            if angle >= 0.32 or hinge > 0.08:
                lead_size = min(0.15, lead_size + 0.008)
            else:
                lead_size = max(0.015, lead_size - 0.012)

            control(
                state,
                offset,
                [lead_size * dx, lead_size * dy, -0.003],
                rotation,
            )
            state = sdk.get_state()
            hinge = joint(state, "hinge_qpos")
            probe_ticks += 1

            if hinge > peak_hinge + 0.002:
                peak_hinge = hinge
                stagnant = 0
            else:
                stagnant += 1

            distance = sqrt(sum(
                (state["eef_pos"][i] - state["objects"]["handle"]["pos"][i]) ** 2
                for i in range(3)
            ))
            if distance > 0.17 or state["gripper_width"] < 0.009:
                lost += 1
            else:
                lost = 0

            if lost >= 8 and attempt < 2 and remaining(state) > 95:
                break

            if searching and probe_ticks >= 26:
                gain = hinge - probe_hinge
                if gain > 0.004:
                    searching = False
                    direction = [dx, dy]
                    selected_hinge = hinge
                    stagnant = 0
                else:
                    candidate_index = (candidate_index + 1) % len(candidates)
                    direction = list(candidates[candidate_index])
                    selected_hinge = hinge
                    lead_size = 0.0
                probe_hinge = hinge
                probe_ticks = 0
                anchor_hinge = hinge
                anchor_handle = list(state["objects"]["handle"]["pos"])

            if not searching and hinge - anchor_hinge > 0.025:
                handle = state["objects"]["handle"]["pos"]
                vx = handle[0] - anchor_handle[0]
                vy = handle[1] - anchor_handle[1]
                length = hypot(vx, vy)
                if length > 0.006:
                    turn = 0.5 * (hinge - anchor_hinge)
                    direction = [
                        (cos(turn) * vx - sin(turn) * vy) / length,
                        (sin(turn) * vx + cos(turn) * vy) / length,
                    ]
                    selected_hinge = hinge
                anchor_hinge = hinge
                anchor_handle = list(handle)

            if not searching and stagnant >= 24:
                searching = True
                candidate_index = (candidate_index + 1) % len(candidates)
                direction = list(candidates[candidate_index])
                selected_hinge = hinge
                probe_hinge = hinge
                probe_ticks = 0
                lead_size = 0.0
                stagnant = 0

    state = sdk.get_state()
    return {
        "status": "done" if finished(state) else "failed",
        "hinge_qpos": joint(state, "hinge_qpos"),
        "handle_qpos": joint(state, "handle_qpos"),
        "steps_used": state["steps_used"],
    }
