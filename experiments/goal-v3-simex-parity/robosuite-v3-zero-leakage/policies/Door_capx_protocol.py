def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def opened(state):
        return state["hinge_qpos"] is not None and state["hinge_qpos"] > 0.30

    def latch(state):
        return abs(state["handle_qpos"]) if state["handle_qpos"] is not None else 0.0

    def move(target, steps, gripper="close"):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.move_to(target, gripper=gripper, max_steps=budget, tol=0.004)

    def grip(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def unit(x, y):
        length = hypot(x, y)
        if length < 0.00001:
            return [0.0, -1.0]
        return [x / length, y / length]

    state = sdk.get_state()
    if opened(state):
        return {"status": "done", "hinge_qpos": state["hinge_qpos"]}

    handle = list(state["objects"]["handle"]["pos"])
    goal = state["goal_pos"]
    if goal is not None:
        preferred = unit(goal[0] - handle[0], goal[1] - handle[1])
    else:
        yaw = state["objects"]["door"]["yaw"]
        preferred = [sin(yaw), -cos(yaw)]

    move([handle[0], handle[1], handle[2] + 0.075], 30, "open")
    handle = list(sdk.get_state()["objects"]["handle"]["pos"])
    move(handle, 30, "open")
    grip("close", 12)

    state = sdk.get_state()
    if state["gripper_width"] < 0.012 and remaining() > 100:
        grip("open", 8)
        handle = list(sdk.get_state()["objects"]["handle"]["pos"])
        move([handle[0], handle[1], handle[2] - 0.012], 18, "open")
        grip("close", 12)

    state = sdk.get_state()
    anchor = list(state["eef_pos"])
    best_target = list(anchor)
    best_latch = latch(state)
    turn_sign = -1.0

    for sign in [-1.0, 1.0]:
        if best_latch >= 0.65 or remaining() < 140:
            break
        for distance in [0.035, 0.070, 0.105, 0.140]:
            target = [anchor[0], anchor[1], anchor[2] + sign * distance]
            move(target, 10)
            state = sdk.get_state()
            if opened(state):
                return {"status": "done", "hinge_qpos": state["hinge_qpos"]}
            value = latch(state)
            if value > best_latch + 0.005:
                best_latch = value
                best_target = list(target)
                turn_sign = sign
            if value >= 0.65:
                break
            if distance >= 0.070 and value < 0.08:
                break
        if best_latch >= 0.25:
            break
        move(anchor, 8)

    move(best_target, 8)
    state = sdk.get_state()
    handle = list(state["objects"]["handle"]["pos"])
    offset = [state["eef_pos"][axis] - handle[axis] for axis in range(3)]
    depression = max(0.035, min(0.09, abs(best_target[2] - state["eef_pos"][2])))
    direction = list(preferred)
    best_hinge = state["hinge_qpos"] if state["hinge_qpos"] is not None else 0.0
    stalls = 0
    probes = 0
    regrasp_count = 0

    while remaining() > 0:
        before = sdk.get_state()
        if opened(before):
            return {
                "status": "done",
                "hinge_qpos": before["hinge_qpos"],
                "steps_used": before["steps_used"],
            }

        handle = list(before["objects"]["handle"]["pos"])
        hinge = before["hinge_qpos"] if before["hinge_qpos"] is not None else 0.0
        separation = sqrt(sum(
            (before["eef_pos"][axis] - handle[axis]) ** 2
            for axis in range(3)
        ))

        if (before["gripper_width"] < 0.010 or separation > 0.11) and remaining() > 75 and regrasp_count < 2:
            grip("open", 6)
            handle = list(sdk.get_state()["objects"]["handle"]["pos"])
            move(handle, 18, "open")
            grip("close", 10)
            regrasp_count += 1
            before = sdk.get_state()
            handle = list(before["objects"]["handle"]["pos"])
            offset = [before["eef_pos"][axis] - handle[axis] for axis in range(3)]
            direction = list(preferred)

        value = latch(before)
        if hinge < 0.08:
            if value < 0.55:
                depression = min(0.12, depression + 0.012)
            elif value > 0.85:
                depression = max(0.025, depression - 0.005)
            vertical = turn_sign * depression
        else:
            vertical = turn_sign * min(depression, 0.035)

        distance = 0.055 if hinge < 0.08 else 0.075
        target = [
            handle[0] + offset[0] + distance * direction[0],
            handle[1] + offset[1] + distance * direction[1],
            handle[2] + offset[2] + vertical,
        ]
        move(target, 10)
        after = sdk.get_state()

        if opened(after):
            return {
                "status": "done",
                "hinge_qpos": after["hinge_qpos"],
                "steps_used": after["steps_used"],
            }

        new_hinge = after["hinge_qpos"] if after["hinge_qpos"] is not None else hinge
        best_hinge = max(best_hinge, new_hinge)
        gain = new_hinge - hinge

        if gain > 0.002:
            stalls = 0
            new_handle = after["objects"]["handle"]["pos"]
            dx = new_handle[0] - handle[0]
            dy = new_handle[1] - handle[1]
            if hypot(dx, dy) > 0.002:
                measured = unit(dx, dy)
                direction = unit(
                    0.5 * direction[0] + 0.5 * measured[0],
                    0.5 * direction[1] + 0.5 * measured[1],
                )
                preferred = list(direction)
        else:
            stalls += 1

        if stalls >= 3 and hinge < 0.08 and latch(after) < 0.60:
            yaw = after["objects"]["door"]["yaw"]
            rotation = -turn_sign * 0.18
            budget = min(6, remaining())
            if budget > 0:
                sdk.step_raw([
                    0.0,
                    0.0,
                    turn_sign * 0.15,
                    rotation * cos(yaw),
                    rotation * sin(yaw),
                    0.0,
                    1.0,
                ], steps=budget)
            stalls = 1
        elif stalls >= 3:
            angles = [0.5, -0.5, 1.0, -1.0, pi]
            angle = angles[probes % len(angles)]
            probes += 1
            direction = [
                cos(angle) * preferred[0] - sin(angle) * preferred[1],
                sin(angle) * preferred[0] + cos(angle) * preferred[1],
            ]
            stalls = 0

    state = sdk.get_state()
    return {
        "status": "done" if opened(state) else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "best_hinge_qpos": best_hinge,
        "steps_used": state["steps_used"],
    }
