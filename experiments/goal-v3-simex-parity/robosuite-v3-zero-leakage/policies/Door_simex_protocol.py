def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(target, steps, grip="close", tol=0.005):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.move_to(target, gripper=grip, max_steps=budget, tol=tol)

    def grip(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def unit(x, y):
        length = hypot(x, y)
        if length < 0.00001:
            return [0.0, -1.0]
        return [x / length, y / length]

    def opened(state):
        return state["hinge_qpos"] is not None and state["hinge_qpos"] > 0.30

    def hinge_value(state):
        value = state["hinge_qpos"]
        return value if value is not None else 0.0

    def latch_value(state):
        value = state["handle_qpos"]
        return value if value is not None else 0.0

    state = sdk.get_state()
    if opened(state):
        return {"status": "done", "hinge_qpos": state["hinge_qpos"]}

    handle = list(state["objects"]["handle"]["pos"])
    door = state["objects"]["door"]["pos"]
    radial = unit(handle[0] - door[0], handle[1] - door[1])
    tangent = [radial[1], -radial[0]]
    goal = state["goal_pos"]
    if goal is not None:
        toward_goal = unit(goal[0] - handle[0], goal[1] - handle[1])
        if tangent[0] * toward_goal[0] + tangent[1] * toward_goal[1] < 0.0:
            tangent = [-tangent[0], -tangent[1]]
    else:
        toward_goal = list(tangent)

    move([handle[0], handle[1], handle[2] + 0.10], 35, "open")
    handle = sdk.get_state()["objects"]["handle"]["pos"]
    move([handle[0], handle[1], handle[2] - 0.004], 30, "open")
    grip("close", 12)

    state = sdk.get_state()
    if state["gripper_width"] < 0.012 and remaining() > 65:
        grip("open", 5)
        handle = sdk.get_state()["objects"]["handle"]["pos"]
        move([handle[0], handle[1], handle[2] - 0.016], 16, "open")
        grip("close", 10)

    # Turn the latch before loading the hinge. Preserve the resulting
    # fingertip-to-handle offset during the subsequent pull.
    state = sdk.get_state()
    latch_start = latch_value(state)
    latch_sign = -1.0
    best_turn = 0.0
    for sign in [-1.0, 1.0]:
        state = sdk.get_state()
        if abs(latch_value(state)) > 0.45 or remaining() < 30:
            break
        anchor = list(state["eef_pos"])
        for depth in [0.025, 0.050, 0.075]:
            move([anchor[0], anchor[1], anchor[2] + sign * depth], 9)
            state = sdk.get_state()
            turn = abs(latch_value(state) - latch_start)
            if turn > best_turn:
                best_turn = turn
                latch_sign = sign
            if opened(state) or abs(latch_value(state)) > 0.45:
                break
        if opened(state) or abs(latch_value(state)) > 0.45:
            break
        move(anchor, 8)

    state = sdk.get_state()
    handle = state["objects"]["handle"]["pos"]
    offset = [
        state["eef_pos"][0] - handle[0],
        state["eef_pos"][1] - handle[1],
        max(-0.065, min(0.065, state["eef_pos"][2] - handle[2])),
    ]
    directions = [
        tangent,
        [-tangent[0], -tangent[1]],
        toward_goal,
        [-radial[0], -radial[1]],
        radial,
    ]
    direction = list(directions[0])
    probe = 0
    stalled = 0
    best_hinge = hinge_value(state)
    command = list(state["eef_pos"])
    recoveries = 0

    while remaining() > 0:
        before = sdk.get_state()
        if opened(before):
            return {
                "status": "done",
                "hinge_qpos": before["hinge_qpos"],
                "steps_used": before["steps_used"],
            }

        handle = list(before["objects"]["handle"]["pos"])
        hinge = hinge_value(before)
        eef = before["eef_pos"]

        # Build sustained horizontal force instead of repeatedly returning
        # to a small displacement from the handle's reported center.
        lead = 0.055 if stalled == 0 else 0.095
        command = [
            handle[0] + offset[0] + direction[0] * lead,
            handle[1] + offset[1] + direction[1] * lead,
            handle[2] + offset[2],
        ]
        if hinge < 0.06 and abs(latch_value(before)) < 0.35:
            command[2] += latch_sign * 0.025

        move(command, 18)
        after = sdk.get_state()
        new_hinge = hinge_value(after)
        best_hinge = max(best_hinge, new_hinge)
        if opened(after):
            return {
                "status": "done",
                "hinge_qpos": new_hinge,
                "steps_used": after["steps_used"],
            }

        gain = new_hinge - hinge
        new_handle = after["objects"]["handle"]["pos"]
        if gain > 0.003:
            stalled = 0
            dx = new_handle[0] - handle[0]
            dy = new_handle[1] - handle[1]
            if hypot(dx, dy) > 0.002:
                measured = unit(dx, dy)
                direction = unit(
                    0.25 * direction[0] + 0.75 * measured[0],
                    0.25 * direction[1] + 0.75 * measured[1],
                )
            if new_hinge > 0.08:
                offset[2] *= 0.8
        else:
            stalled += 1

        if gain < -0.006 or stalled >= 2:
            probe += 1
            direction = list(directions[probe % len(directions)])
            stalled = 0
            # Remove the previous lateral load before changing pull direction.
            move(
                [
                    new_handle[0] + offset[0],
                    new_handle[1] + offset[1],
                    new_handle[2] + offset[2],
                ],
                6,
            )

        if (
            after["gripper_width"] < 0.010
            and remaining() > 55
            and recoveries < 2
        ):
            recoveries += 1
            grip("open", 5)
            handle = sdk.get_state()["objects"]["handle"]["pos"]
            move([handle[0], handle[1], handle[2] + 0.045], 10, "open")
            move([handle[0], handle[1], handle[2] - 0.012], 14, "open")
            grip("close", 10)
            state = sdk.get_state()
            handle = state["objects"]["handle"]["pos"]
            offset = [
                state["eef_pos"][0] - handle[0],
                state["eef_pos"][1] - handle[1],
                state["eef_pos"][2] - handle[2],
            ]
            stalled = 0

    state = sdk.get_state()
    return {
        "status": "done" if opened(state) else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "best_hinge_qpos": best_hinge,
        "steps_used": state["steps_used"],
    }
