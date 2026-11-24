def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(pos, grip, limit=35, tol=0.012):
        budget = remaining()
        if budget <= 0:
            return False
        return sdk.move_to(
            pos, gripper=grip, max_steps=min(limit, budget), tol=tol
        )["reached"]

    def actuate(action, ticks):
        budget = remaining()
        if budget > 0:
            sdk.set_gripper(action, steps=min(ticks, budget))

    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or goal"}

    held = False
    offset = None

    for attempt in range(3):
        if remaining() < 150:
            break

        actuate("open", 10)
        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        clearance = max(can[2] + 0.14, state["goal_pos"][2] + 0.19)

        if eef[2] < clearance - 0.015:
            move([eef[0], eef[1], clearance], "open", 25)

        move([can[0], can[1], clearance], "open", 35)
        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.025:
            continue

        grasp_height = can[2] + (0.025 if attempt == 0 else 0.01)
        move([can[0], can[1], grasp_height], "open", 32, 0.008)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])

        if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.012:
            move([can[0], can[1], grasp_height], "open", 12, 0.008)
            state = sdk.get_state()
            can = list(state["objects"]["can"]["pos"])
            eef = list(state["eef_pos"])

        if (
            hypot(eef[0] - can[0], eef[1] - can[1]) > 0.025
            or eef[2] > can[2] + 0.065
            or eef[2] < can[2] - 0.035
        ):
            continue

        before = list(can)
        actuate("close", 20)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        move([eef[0], eef[1], eef[2] + 0.12], "close", 30, 0.012)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])

        if (
            can[2] - before[2] > 0.05
            and hypot(can[0] - eef[0], can[1] - eef[1]) < 0.065
            and abs(can[2] - eef[2]) < 0.13
        ):
            held = True
            offset = [can[i] - eef[i] for i in range(3)]
            break

    if not held:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    can = list(state["objects"]["can"]["pos"])
    eef = list(state["eef_pos"])
    transit_z = max(can[2], goal[2] + 0.20) - offset[2]

    move([eef[0], eef[1], transit_z], "close", 25)
    state = sdk.get_state()
    if state["objects"]["can"]["pos"][2] < goal[2] + 0.145:
        return {"status": "failed", "reason": "Insufficient transport clearance"}

    move(
        [goal[0] - offset[0], goal[1] - offset[1], transit_z],
        "close",
        50,
        0.01,
    )

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    eef = list(state["eef_pos"])
    if (
        abs(can[2] - eef[2] - offset[2]) > 0.05
        or hypot(
            can[0] - eef[0] - offset[0],
            can[1] - eef[1] - offset[1],
        ) > 0.045
    ):
        return {"status": "failed", "reason": "Grasp lost during transport"}

    if remaining() < 45:
        return {"status": "failed", "reason": "Insufficient placement time"}

    offset = [can[i] - eef[i] for i in range(3)]
    move(
        [goal[0] - offset[0], goal[1] - offset[1], eef[2]],
        "close",
        min(18, remaining() - 40),
        0.008,
    )

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    eef = list(state["eef_pos"])
    if hypot(can[0] - goal[0], can[1] - goal[1]) > 0.04:
        return {"status": "failed", "reason": "Could not align over target bin"}

    offset = [can[i] - eef[i] for i in range(3)]
    move(
        [
            goal[0] - offset[0],
            goal[1] - offset[1],
            goal[2] + 0.065 - offset[2],
        ],
        "close",
        min(35, remaining() - 32),
        0.01,
    )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.045
        or can[2] < goal[2] - 0.02
        or can[2] > goal[2] + 0.13
    ):
        return {"status": "failed", "reason": "Unsafe release position"}

    actuate("open", 12)
    state = sdk.get_state()
    eef = list(state["eef_pos"])
    move([eef[0], eef[1], eef[2] + 0.15], "open", 20)
    actuate("open", 10)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    placed = (
        hypot(can[0] - goal[0], can[1] - goal[1]) < 0.055
        and goal[2] - 0.025 < can[2] < goal[2] + 0.115
        and state["eef_pos"][2] - can[2] > 0.08
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Can released in target bin" if placed else "Placement verification failed",
        "steps_used": state["steps_used"],
    }
