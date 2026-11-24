def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def distance(a, b):
        return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    def move(pos, gripper, steps=40, tol=0.015):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.move_to(pos, gripper=gripper, max_steps=budget, tol=tol)
        return sdk.get_state()

    def grip(action, steps=12):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)
        return sdk.get_state()

    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or target position"}

    held = False
    offset = [0.0, 0.0, 0.0]

    for attempt in range(2):
        if remaining() < 170:
            break

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        goal = list(state["goal_pos"])
        clearance = max(can[2] + 0.16, goal[2] + 0.20)
        eef = list(state["eef_pos"])

        state = move(
            [eef[0], eef[1], max(eef[2], clearance)], "open", 25
        )
        can = list(state["objects"]["can"]["pos"])
        state = move([can[0], can[1], clearance], "open", 40)
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]

        if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.025:
            state = move([can[0], can[1], clearance], "open", 20)
            can = list(state["objects"]["can"]["pos"])
            eef = state["eef_pos"]
            if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.025:
                continue

        grasp_height = can[2] + (0.005 if attempt == 0 else -0.008)
        state = move(
            [can[0], can[1], grasp_height], "open", 40, 0.008
        )
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        if (
            hypot(eef[0] - can[0], eef[1] - can[1]) > 0.022
            or abs(eef[2] - grasp_height) > 0.025
        ):
            continue

        state = grip("close", 16)
        before = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        initial_offset = [before[i] - eef[i] for i in range(3)]

        state = move(
            [eef[0], eef[1], eef[2] + 0.12], "close", 30
        )
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        offset = [can[i] - eef[i] for i in range(3)]
        held = (
            can[2] - before[2] > 0.05
            and distance(offset, initial_offset) < 0.04
            and distance(can, eef) < 0.13
        )
        if held:
            break
        grip("open", 10)

    if not held:
        return {"status": "failed", "reason": "Can did not follow the lift"}

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    goal = list(state["goal_pos"])
    eef = list(state["eef_pos"])
    transit_height = max(can[2], goal[2] + 0.20)
    state = move(
        [eef[0], eef[1], transit_height - offset[2]], "close", 25
    )

    # Judge actual object clearance instead of a strict move timeout.
    can = list(state["objects"]["can"]["pos"])
    if can[2] < goal[2] + 0.15:
        eef = list(state["eef_pos"])
        state = move(
            [eef[0], eef[1], goal[2] + 0.21 - offset[2]], "close", 20
        )
        can = list(state["objects"]["can"]["pos"])
        if can[2] < goal[2] + 0.15:
            return {"status": "failed", "reason": "Insufficient object clearance"}

    eef = list(state["eef_pos"])
    offset = [can[i] - eef[i] for i in range(3)]
    transit_eef_height = eef[2]
    state = move(
        [goal[0] - offset[0], goal[1] - offset[1], transit_eef_height],
        "close",
        50,
    )

    for correction in range(2):
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        goal = list(state["goal_pos"])
        current_offset = [can[i] - eef[i] for i in range(3)]
        if distance(current_offset, offset) > 0.05:
            return {"status": "failed", "reason": "Can slipped during transport"}
        if hypot(can[0] - goal[0], can[1] - goal[1]) < 0.025:
            break
        if remaining() < 65:
            break
        state = move(
            [
                eef[0] + goal[0] - can[0],
                eef[1] + goal[1] - can[1],
                transit_eef_height,
            ],
            "close",
            18,
            0.01,
        )

    can = list(state["objects"]["can"]["pos"])
    eef = list(state["eef_pos"])
    goal = list(state["goal_pos"])
    if hypot(can[0] - goal[0], can[1] - goal[1]) > 0.045:
        return {"status": "failed", "reason": "Can is not over target bin"}
    if remaining() < 40:
        return {"status": "failed", "reason": "Insufficient release budget"}

    offset = [can[i] - eef[i] for i in range(3)]
    state = move(
        [
            goal[0] - offset[0],
            goal[1] - offset[1],
            goal[2] + 0.075 - offset[2],
        ],
        "close",
        min(30, remaining() - 35),
        0.015,
    )
    can = list(state["objects"]["can"]["pos"])
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.045
        or can[2] > goal[2] + 0.17
    ):
        return {"status": "failed", "reason": "Unsafe release position"}

    state = grip("open", 12)
    eef = list(state["eef_pos"])
    move([eef[0], eef[1], eef[2] + 0.16], "open", 20)
    grip("open", min(15, remaining()))

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    placed = (
        hypot(can[0] - goal[0], can[1] - goal[1]) < 0.075
        and goal[2] - 0.025 < can[2] < goal[2] + 0.14
        and state["gripper_width"] > 0.05
        and distance(can, state["eef_pos"]) > 0.10
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Can released in target bin" if placed else "Placement unverified",
        "steps_used": state["steps_used"],
    }
