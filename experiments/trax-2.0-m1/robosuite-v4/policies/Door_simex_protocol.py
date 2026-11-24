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

    def result(state):
        return {
            "status": "done" if finished(state) else "failed",
            "hinge_qpos": joint(state, "hinge_qpos"),
            "handle_qpos": joint(state, "handle_qpos"),
            "steps_used": state["steps_used"],
        }

    def drive(state, target, rotation, grip=1.0):
        action = [
            clamp(24.0 * (target[i] - state["eef_pos"][i]), 0.75)
            for i in range(3)
        ]
        action += [clamp(value, 0.5) for value in rotation]
        action += [grip]
        sdk.step_raw(action, steps=1)

    state = sdk.get_state()
    if finished(state) or remaining(state) == 0:
        return result(state)

    handle = state["objects"]["handle"]["pos"]
    goal = state["goal_pos"]
    door = state["objects"]["door"]["pos"]
    destination = goal if goal is not None else door
    pull_x = destination[0] - handle[0]
    pull_y = destination[1] - handle[1]
    length = hypot(pull_x, pull_y)
    if length < 0.001:
        pull_x, pull_y = 0.0, -1.0
    else:
        pull_x /= length
        pull_y /= length

    probes = [
        [0.0, 0.0, -0.025, 0.0, 0.0, 0.0],
        [0.0, 0.0, -0.010, 0.45, 0.0, 0.0],
        [0.0, 0.0, -0.010, -0.45, 0.0, 0.0],
        [0.0, 0.0, -0.010, 0.0, 0.45, 0.0],
        [0.0, 0.0, -0.010, 0.0, -0.45, 0.0],
    ]

    for attempt in range(2):
        state = sdk.get_state()
        if finished(state) or remaining(state) < 15:
            return result(state)

        attempt_start = state["steps_used"]
        attempt_budget = min(remaining(state), 145)

        sdk.set_gripper("open", steps=min(5, remaining(state)))
        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        yaw = state["objects"]["handle"]["yaw"]
        if attempt == 1:
            yaw += pi / 2.0

        approach = [
            handle[0] - 0.025 * pull_x,
            handle[1] - 0.025 * pull_y,
            handle[2] + 0.055,
        ]
        if remaining(state) > 0:
            sdk.move_to(
                approach,
                yaw=yaw,
                gripper="open",
                max_steps=min(24, remaining(state)),
                tol=0.012,
            )

        for tick in range(25):
            state = sdk.get_state()
            if finished(state) or remaining(state) == 0:
                return result(state)
            handle = state["objects"]["handle"]["pos"]
            target = [
                handle[0] - 0.004 * pull_x,
                handle[1] - 0.004 * pull_y,
                handle[2] - 0.006,
            ]
            error = sqrt(sum(
                (target[i] - state["eef_pos"][i]) ** 2
                for i in range(3)
            ))
            if error < 0.006:
                break
            drive(state, target, [0.0, 0.0, 0.0], -1.0)

        state = sdk.get_state()
        if remaining(state) == 0:
            return result(state)
        sdk.set_gripper("close", steps=min(9, remaining(state)))

        state = sdk.get_state()
        offset = [
            clamp(state["eef_pos"][i] - state["objects"]["handle"]["pos"][i], 0.025)
            for i in range(3)
        ]
        best = list(probes[0])
        best_score = -100.0
        highest_hinge = joint(state, "hinge_qpos")
        stagnant = 0
        unlocked_ticks = 0
        probe_index = 0
        probe_ticks = 0
        probe_start = joint(state, "handle_qpos")
        active = list(probes[0])

        for tick in range(400):
            state = sdk.get_state()
            if finished(state) or remaining(state) == 0:
                return result(state)
            if state["steps_used"] - attempt_start >= attempt_budget:
                break

            handle = state["objects"]["handle"]["pos"]
            latch = joint(state, "handle_qpos")
            hinge = joint(state, "hinge_qpos")
            separation = sqrt(sum(
                (state["eef_pos"][i] - handle[i]) ** 2
                for i in range(3)
            ))

            if hinge > highest_hinge + 0.002:
                highest_hinge = hinge
                stagnant = 0
            else:
                stagnant += 1

            unlocked = latch > 0.36 or hinge > 0.035
            if unlocked:
                unlocked_ticks += 1
                best = list(active)
            else:
                unlocked_ticks = 0

            if not unlocked and probe_index < len(probes):
                if probe_ticks >= 9:
                    score = latch - probe_start
                    if score > best_score:
                        best_score = score
                        best = list(active)
                    if latch > 0.20 and score > 0.02:
                        probe_ticks = 0
                        probe_start = latch
                    else:
                        probe_index += 1
                        probe_ticks = 0
                        probe_start = latch
                        if probe_index < len(probes):
                            active = list(probes[probe_index])
                        else:
                            active = list(best)
                probe_ticks += 1
            elif unlocked or probe_index >= len(probes):
                active = list(best)

            angle = max(0.0, hinge)
            direction_x = cos(angle) * pull_x - sin(angle) * pull_y
            direction_y = sin(angle) * pull_x + cos(angle) * pull_y

            lead = 0.004
            if unlocked:
                lead = min(0.055, 0.012 + 0.002 * unlocked_ticks)
            if separation > 0.065:
                lead = min(lead, 0.012)

            effort = clamp((0.48 - latch) * 6.0, 1.0)
            if hinge > 0.08:
                effort = max(0.0, effort) * 0.35
            elif not unlocked:
                effort = max(0.35, effort)

            target = [
                handle[0] + offset[0] + lead * direction_x,
                handle[1] + offset[1] + lead * direction_y,
                handle[2] + offset[2],
            ]
            for i in range(3):
                target[i] += active[i] * effort
            drive(state, target, [value * effort for value in active[3:6]])

            if separation > 0.10 and tick > 12:
                break
            if stagnant > 65 and latch < 0.12:
                break

    state = sdk.get_state()
    if finished(state) or remaining(state) < 12:
        return result(state)

    sdk.set_gripper("open", steps=min(4, remaining(state)))
    state = sdk.get_state()
    handle = list(state["objects"]["handle"]["pos"])
    door_yaw = state["objects"]["door"]["yaw"]
    normal_x = sin(door_yaw)
    normal_y = -cos(door_yaw)
    if normal_x * pull_x + normal_y * pull_y < 0.0:
        normal_x = -normal_x
        normal_y = -normal_y

    approach = [
        handle[0] + 0.012 * normal_x,
        handle[1] + 0.012 * normal_y,
        handle[2] + 0.040,
    ]
    if remaining(state) > 0:
        sdk.move_to(
            approach,
            yaw=state["objects"]["handle"]["yaw"] + pi / 2.0,
            gripper="open",
            max_steps=min(16, remaining(state)),
            tol=0.010,
        )

    for tick in range(16):
        state = sdk.get_state()
        if finished(state) or remaining(state) == 0:
            return result(state)
        handle = state["objects"]["handle"]["pos"]
        target = [
            handle[0] - 0.010 * normal_x,
            handle[1] - 0.010 * normal_y,
            handle[2] - 0.018,
        ]
        drive(state, target, [0.0, 0.0, 0.0], -1.0)
        if sum(
            (target[i] - state["eef_pos"][i]) ** 2 for i in range(3)
        ) < 0.000036:
            break

    state = sdk.get_state()
    if remaining(state) == 0:
        return result(state)
    sdk.set_gripper("close", steps=min(7, remaining(state)))
    state = sdk.get_state()
    offset = [
        clamp(state["eef_pos"][i] - state["objects"]["handle"]["pos"][i], 0.020)
        for i in range(3)
    ]
    base_hinge = joint(state, "hinge_qpos")
    best_axis = [0.0, 0.0, 0.0]
    best_gain = -100.0
    previous_latch = joint(state, "handle_qpos")
    axis = [0.0, 0.0, 0.0]
    contact_ticks = 0
    progress_hinge = base_hinge
    stagnant = 0
    recovery = 0
    recovery_ticks = 0
    recovered = False
    rescue_start = joint(state, "handle_qpos")
    rescue_axis = 0
    rescue_gain = -100.0
    rescue_best = [0.0, 0.0, 0.0]

    for tick in range(400):
        state = sdk.get_state()
        if finished(state) or remaining(state) == 0:
            return result(state)

        handle = state["objects"]["handle"]["pos"]
        latch = joint(state, "handle_qpos")
        hinge = joint(state, "hinge_qpos")
        separation = sqrt(sum(
            (state["eef_pos"][i] - handle[i]) ** 2 for i in range(3)
        ))

        if hinge > progress_hinge + 0.002:
            progress_hinge = hinge
            stagnant = 0
        else:
            stagnant += 1

        if tick >= 40 and stagnant > 20 and separation > 0.085:
            if recovery == 0 and not recovered and remaining(state) > 22:
                recovery = 1
                recovery_ticks = 0
                recovered = True

        if recovery > 0:
            recovery_ticks += 1
            target = [
                handle[0] - 0.006 * normal_x,
                handle[1] - 0.006 * normal_y,
                handle[2] - 0.008,
            ]
            if recovery == 1:
                drive(state, target, [0.0, 0.0, 0.0], -1.0)
                error = sqrt(sum(
                    (target[i] - state["eef_pos"][i]) ** 2 for i in range(3)
                ))
                if recovery_ticks >= 12 or (
                    recovery_ticks >= 5 and error < 0.012
                ):
                    recovery = 2
                    recovery_ticks = 0
            else:
                drive(state, target, [0.0, 0.0, 0.0])
                if recovery_ticks >= 6:
                    state = sdk.get_state()
                    offset = [
                        clamp(
                            state["eef_pos"][i] -
                            state["objects"]["handle"]["pos"][i],
                            0.015,
                        )
                        for i in range(3)
                    ]
                    recovery = 0
                    contact_ticks = 0
                    stagnant = 0
            continue

        if tick % 8 == 0:
            gain = latch - previous_latch
            if tick > 0 and gain > best_gain:
                best_gain = gain
                best_axis = list(axis)
            previous_latch = latch
            if tick < 40 and latch < 0.85:
                axis = list(probes[min(4, tick // 8)][3:6])
            else:
                axis = list(best_axis)

        angle = hinge - base_hinge
        direction_x = cos(angle) * normal_x - sin(angle) * normal_y
        direction_y = sin(angle) * normal_x + cos(angle) * normal_y

        effort = max(0.0, min(1.0, (1.05 - latch) * 4.0))
        if hinge > 0.06:
            effort *= 0.3

        if latch > 0.75 or hinge > 0.025:
            contact_ticks += 1
        else:
            contact_ticks = max(0, contact_ticks - 1)

        lead = 0.008
        if contact_ticks > 0:
            lead = min(0.085, 0.018 + 0.004 * contact_ticks)
        elif tick > 40:
            lead = 0.035

        downward = 0.038 * effort
        rotation = [value * effort for value in axis]

        if tick >= 40 and stagnant > 20 and hinge < 0.06:
            if latch < 0.85:
                lead = 0.006
                downward = min(0.055, 0.035 + 0.0005 * stagnant)
                phase = (stagnant - 21) % 6
                if phase == 0:
                    if rescue_axis > 0:
                        gain = latch - rescue_start
                        if gain > rescue_gain:
                            rescue_gain = gain
                            rescue_best = list(axis)
                    rescue_start = latch
                    if rescue_axis < 4:
                        axis = list(probes[1 + rescue_axis][3:6])
                        rescue_axis += 1
                    else:
                        axis = list(rescue_best)
                    best_axis = list(axis)
                rotation = list(axis)
            else:
                downward = 0.006
                rotation = [0.0, 0.0, 0.0]
                lead = min(0.070, 0.020 + 0.002 * contact_ticks)
                offset[0] *= 0.85
                offset[1] *= 0.85

        if separation > 0.095:
            lead = 0.008
            downward = min(downward, 0.018)
            offset = [value * 0.85 for value in offset]

        target = [
            handle[0] + offset[0] + lead * direction_x,
            handle[1] + offset[1] + lead * direction_y,
            handle[2] + offset[2] - downward,
        ]
        drive(state, target, rotation)

    return result(sdk.get_state())
