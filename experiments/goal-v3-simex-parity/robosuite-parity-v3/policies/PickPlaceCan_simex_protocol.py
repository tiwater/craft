def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def servo(target, grip, limit, tolerance=0.009, reserve=0):
        settled = 0
        for tick in range(limit):
            if remaining() <= reserve:
                break
            state = sdk.get_state()
            position = state["eef_pos"]
            error = [target[i] - position[i] for i in range(3)]
            distance = sqrt(sum(value * value for value in error))
            if distance < tolerance:
                settled += 1
                if settled >= 2:
                    return True
            else:
                settled = 0
            action = [16.0 * value for value in error]
            largest = max(abs(value) for value in action)
            if largest > 0.85:
                action = [value * 0.85 / largest for value in action]
            sdk.step_raw(
                [action[0], action[1], action[2], 0.0, 0.0, 0.0, grip],
                steps=1,
            )
        state = sdk.get_state()
        return sqrt(
            sum((target[i] - state["eef_pos"][i]) ** 2 for i in range(3))
        ) < tolerance

    def actuate(action, ticks):
        budget = remaining()
        if budget > 0:
            sdk.set_gripper(action, steps=min(ticks, budget))

    def attached(state, offset):
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        return (
            0.012 < state["gripper_width"] < 0.075
            and hypot(
                can[0] - eef[0] - offset[0],
                can[1] - eef[1] - offset[1],
            ) < 0.035
            and abs(can[2] - eef[2] - offset[2]) < 0.04
        )

    def verify_local_grasp(reserve):
        if remaining() < reserve + 32:
            return None
        actuate("close", 6)
        state = sdk.get_state()
        before_can = list(state["objects"]["can"]["pos"])
        before_eef = list(state["eef_pos"])
        if (
            not 0.012 < state["gripper_width"] < 0.075
            or hypot(
                before_can[0] - before_eef[0],
                before_can[1] - before_eef[1],
            ) > 0.055
        ):
            return None

        servo(
            [before_eef[0], before_eef[1], before_eef[2] + 0.045],
            1.0,
            24,
            tolerance=0.006,
            reserve=reserve,
        )
        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        can_motion = [can[i] - before_can[i] for i in range(3)]
        eef_motion = [eef[i] - before_eef[i] for i in range(3)]
        if (
            0.012 < state["gripper_width"] < 0.075
            and can_motion[2] > 0.020
            and eef_motion[2] > 0.025
            and sqrt(
                sum((can_motion[i] - eef_motion[i]) ** 2 for i in range(3))
            ) < 0.018
        ):
            return [can[i] - eef[i] for i in range(3)]
        return None

    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or goal"}

    held = False
    offset = [0.0, 0.0, 0.0]

    for attempt in range(2):
        if remaining() < 175:
            break

        actuate("open", 7)
        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        clearance = max(eef[2], can[2] + 0.15)

        if eef[2] < can[2] + 0.12:
            servo([eef[0], eef[1], clearance], -1.0, 22, reserve=145)

        servo([can[0], can[1], clearance], -1.0, 42, reserve=135)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.022:
            continue

        grasp_height = 0.035 if attempt == 0 else 0.025
        for tick in range(38):
            if remaining() <= 125:
                break
            state = sdk.get_state()
            can = state["objects"]["can"]["pos"]
            eef = state["eef_pos"]
            horizontal_error = hypot(can[0] - eef[0], can[1] - eef[1])
            height_error = can[2] + grasp_height - eef[2]
            if horizontal_error < 0.007 and abs(height_error) < 0.008:
                break
            vertical = height_error
            if horizontal_error > 0.018:
                vertical = max(0.0, vertical)
            sdk.step_raw(
                [
                    max(-0.45, min(0.45, 16.0 * (can[0] - eef[0]))),
                    max(-0.45, min(0.45, 16.0 * (can[1] - eef[1]))),
                    max(-0.45, min(0.45, 16.0 * vertical)),
                    0.0,
                    0.0,
                    0.0,
                    -1.0,
                ],
                steps=1,
            )

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        if (
            hypot(eef[0] - can[0], eef[1] - can[1]) > 0.018
            or not 0.008 < eef[2] - can[2] < 0.060
        ):
            continue

        actuate("close", 16)
        state = sdk.get_state()
        before = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        grasp_offset = [before[i] - eef[i] for i in range(3)]

        servo(
            [eef[0], eef[1], eef[2] + 0.115],
            1.0,
            30,
            reserve=105,
        )
        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        if can[2] - before[2] > 0.055 and attached(state, grasp_offset):
            held = True
            offset = [can[i] - eef[i] for i in range(3)]
            break

    if not held:
        return {
            "status": "failed",
            "reason": "Could not verify lifted grasp",
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    can = list(state["objects"]["can"]["pos"])
    eef = list(state["eef_pos"])
    transit_height = max(can[2], goal[2] + 0.23)
    servo(
        [eef[0], eef[1], transit_height - offset[2]],
        1.0,
        26,
        reserve=85,
    )

    state = sdk.get_state()
    if (
        not attached(state, offset)
        or state["objects"]["can"]["pos"][2] < goal[2] + 0.17
    ):
        # Recovery executes only where the original policy would terminate.
        recovered_offset = verify_local_grasp(150)
        if recovered_offset is None:
            return {
                "status": "failed",
                "reason": "Transport recovery could not verify grasp",
                "steps_used": sdk.get_state()["steps_used"],
            }
        offset = recovered_offset
        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        goal = list(state["goal_pos"])
        transit_height = max(transit_height, can[2], goal[2] + 0.23)
        servo(
            [eef[0], eef[1], transit_height - offset[2]],
            1.0,
            42,
            tolerance=0.008,
            reserve=140,
        )
        state = sdk.get_state()
        if (
            not attached(state, offset)
            or state["objects"]["can"]["pos"][2] < goal[2] + 0.17
        ):
            return {
                "status": "failed",
                "reason": "Transport clearance unavailable after recovery",
                "steps_used": state["steps_used"],
            }

    aligned = False
    for tick in range(110):
        if remaining() <= 55:
            break
        state = sdk.get_state()
        if not attached(state, offset):
            return {"status": "failed", "reason": "Grasp lost during transport"}
        can = state["objects"]["can"]["pos"]
        goal = state["goal_pos"]
        error = [
            goal[0] - can[0],
            goal[1] - can[1],
            transit_height - can[2],
        ]
        if hypot(error[0], error[1]) < 0.012 and abs(error[2]) < 0.016:
            aligned = True
            break
        action = [16.0 * value for value in error]
        largest = max(abs(value) for value in action)
        if largest > 0.75:
            action = [value * 0.75 / largest for value in action]
        sdk.step_raw(
            [action[0], action[1], action[2], 0.0, 0.0, 0.0, 1.0],
            steps=1,
        )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    if not aligned and hypot(can[0] - goal[0], can[1] - goal[1]) > 0.025:
        return {"status": "failed", "reason": "Target alignment incomplete"}

    for tick in range(42):
        if remaining() <= 28:
            break
        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        goal = state["goal_pos"]
        if not attached(state, offset):
            return {"status": "failed", "reason": "Grasp lost before release"}
        error = [
            goal[0] - can[0],
            goal[1] - can[1],
            goal[2] + 0.055 - can[2],
        ]
        if hypot(error[0], error[1]) < 0.012 and abs(error[2]) < 0.012:
            break
        vertical = error[2]
        if hypot(error[0], error[1]) > 0.025:
            vertical = max(0.0, vertical)
        sdk.step_raw(
            [
                max(-0.35, min(0.35, 16.0 * error[0])),
                max(-0.35, min(0.35, 16.0 * error[1])),
                max(-0.50, min(0.50, 16.0 * vertical)),
                0.0,
                0.0,
                0.0,
                1.0,
            ],
            steps=1,
        )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.030
        or not goal[2] - 0.02 < can[2] < goal[2] + 0.11
    ):
        return {"status": "failed", "reason": "Unsafe release position"}

    actuate("open", 10)
    state = sdk.get_state()
    eef = list(state["eef_pos"])
    servo([eef[0], eef[1], eef[2] + 0.14], -1.0, 22, reserve=5)
    actuate("open", min(8, remaining()))

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    placed = (
        hypot(can[0] - goal[0], can[1] - goal[1]) < 0.055
        and goal[2] - 0.025 < can[2] < goal[2] + 0.115
        and state["gripper_width"] > 0.055
        and state["eef_pos"][2] - can[2] > 0.075
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Can released in target bin" if placed else "Placement verification failed",
        "steps_used": state["steps_used"],
    }
