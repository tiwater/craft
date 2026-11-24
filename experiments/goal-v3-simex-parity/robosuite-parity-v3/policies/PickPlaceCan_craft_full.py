def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def move(pos, gripper, limit=35, tol=0.016):
        if remaining() < 1:
            return False
        result = sdk.move_to(
            pos,
            gripper=gripper,
            max_steps=min(limit, remaining()),
            tol=tol,
        )
        return result["reached"] or result["pos_err_m"] < 0.025

    def actuate(action, steps):
        if remaining() > 0:
            return sdk.set_gripper(action, steps=min(steps, remaining()))
        return None

    def level_quat(quat):
        norm = hypot(quat[0], quat[1])
        if norm < 0.1:
            return list(quat)
        return [quat[0] / norm, quat[1] / norm, 0.0, 0.0]

    def raw_tick(target, reference, grip, speed=0.8):
        state = sdk.get_state()
        pos = state["eef_pos"]
        quat = state["eef_quat"]
        tx, ty, tz, tw = reference
        x, y, z, w = quat
        error = [
            -tw * x + tx * w - ty * z + tz * y,
            -tw * y + tx * z + ty * w - tz * x,
            -tw * z - tx * y + ty * x + tz * w,
        ]
        scalar = tw * w + tx * x + ty * y + tz * z
        sign = 1.0 if scalar >= 0.0 else -1.0
        norm = sqrt(sum(value * value for value in error))
        factor = 0.0
        if norm > 0.000001:
            factor = sign * 2.0 * atan2(norm, abs(scalar)) / norm
        action = [clamp(18.0 * (target[i] - pos[i]), speed) for i in range(3)]
        action.extend([clamp(1.6 * factor * value, 0.6) for value in error])
        action.append(grip)
        sdk.step_raw(action, steps=1)

    def object_servo(target, reference, limit, reserve, tolerance, descending=False):
        stable = 0
        for tick in range(limit):
            if remaining() <= reserve:
                break
            state = sdk.get_state()
            can = state["objects"]["can"]["pos"]
            eef = state["eef_pos"]
            error = [target[i] - can[i] for i in range(3)]
            if hypot(error[0], error[1]) < tolerance and abs(error[2]) < tolerance:
                stable += 1
                if stable >= 3:
                    return True
            else:
                stable = 0

            desired = [eef[i] + error[i] for i in range(3)]
            if descending:
                lateral = hypot(error[0], error[1])
                if lateral > 0.025:
                    desired[2] = eef[2] + max(0.0, error[2])
                else:
                    desired[2] = eef[2] + max(-0.014, min(0.014, error[2]))
            raw_tick(desired, reference, 1.0, 0.75)
        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        return (
            hypot(can[0] - target[0], can[1] - target[1]) < tolerance
            and abs(can[2] - target[2]) < tolerance
        )

    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or goal"}

    held = False
    reference = level_quat(state["eef_quat"])

    for attempt in range(2):
        if remaining() < 185:
            break
        actuate("open", 6)
        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        clearance = max(eef[2], can[2] + 0.15)
        if eef[2] < can[2] + 0.085:
            move([eef[0], eef[1], clearance], "open", 22)
        move([can[0], can[1], clearance], "open", 38, 0.018)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.035:
            continue

        move(
            [can[0], can[1], can[2] + (0.005 if attempt == 0 else 0.015)],
            "open",
            38,
            0.014,
        )
        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - can[0], eef[1] - can[1]) > 0.03
            or abs(eef[2] - can[2]) > 0.05
        ):
            continue

        actuate("close", 16)
        state = sdk.get_state()
        before = list(state["objects"]["can"]["pos"])
        reference = level_quat(state["eef_quat"])
        goal = state["goal_pos"]
        transit_z = max(before[2] + 0.16, goal[2] + 0.19)
        object_servo(
            [before[0], before[1], transit_z],
            reference,
            42,
            145,
            0.015,
        )
        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        if (
            can[2] - before[2] > 0.065
            and 0.012 < state["gripper_width"] < 0.075
            and hypot(can[0] - eef[0], can[1] - eef[1]) < 0.065
            and abs(can[2] - eef[2]) < 0.11
        ):
            held = True
            break

    if not held:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    can = state["objects"]["can"]["pos"]
    transit_z = max(can[2], goal[2] + 0.19)
    object_servo(
        [goal[0], goal[1], transit_z],
        reference,
        110,
        95,
        0.014,
    )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.035
        or can[2] < goal[2] + 0.12
        or state["gripper_width"] < 0.012
    ):
        return {"status": "failed", "reason": "Could not align held can above bin"}

    object_servo(
        [goal[0], goal[1], goal[2] + 0.057],
        reference,
        max(0, remaining() - 38),
        38,
        0.009,
        True,
    )
    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.035
        or can[2] > goal[2] + 0.095
        or can[2] < goal[2] - 0.02
    ):
        return {"status": "failed", "reason": "Can is not safely positioned for release"}

    state = sdk.get_state()
    release_pos = list(state["eef_pos"])
    for tick in range(min(12, remaining())):
        raw_tick(release_pos, reference, -1.0, 0.3)

    retreat = [release_pos[0], release_pos[1], release_pos[2] + 0.14]
    for tick in range(min(20, max(0, remaining() - 6))):
        raw_tick(retreat, reference, -1.0, 0.65)
    actuate("open", min(6, remaining()))

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
