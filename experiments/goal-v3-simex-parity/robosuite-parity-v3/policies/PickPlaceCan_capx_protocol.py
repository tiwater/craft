def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(pos, grip, limit=40, tol=0.018):
        budget = remaining()
        if budget <= 0:
            return False
        result = sdk.move_to(
            pos, gripper=grip, max_steps=min(limit, budget), tol=tol
        )
        return result["reached"] or result["pos_err_m"] < 0.025

    def actuate(action, steps):
        budget = remaining()
        if budget > 0:
            sdk.set_gripper(action, steps=min(steps, budget))

    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or goal"}

    held = False
    offset = [0.0, 0.0, 0.0]

    for attempt in range(2):
        if remaining() < 190:
            break

        actuate("open", 8)
        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        clearance = max(can[2] + 0.15, state["goal_pos"][2] + 0.17)

        if eef[2] < clearance - 0.025:
            move([eef[0], eef[1], clearance], "open", 25)

        move([can[0], can[1], clearance], "open", 40)
        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.025:
            continue

        grasp = [can[0], can[1], can[2] + 0.005]
        move(grasp, "open", 40, 0.012)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        if (
            hypot(eef[0] - can[0], eef[1] - can[1]) > 0.018
            or abs(eef[2] - can[2] - 0.005) > 0.025
        ):
            move([can[0], can[1], can[2] + 0.005], "open", 15, 0.012)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        if (
            hypot(eef[0] - can[0], eef[1] - can[1]) > 0.025
            or abs(eef[2] - can[2]) > 0.035
        ):
            continue

        before = list(can)
        actuate("close", 16)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        move([eef[0], eef[1], eef[2] + 0.12], "close", 30)

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        if (
            can[2] - before[2] > 0.06
            and 0.012 < state["gripper_width"] < 0.075
            and hypot(can[0] - eef[0], can[1] - eef[1]) < 0.055
            and abs(can[2] - eef[2]) < 0.10
        ):
            held = True
            offset = [can[i] - eef[i] for i in range(3)]
            break

    if not held:
        return {"status": "failed", "reason": "Could not verify lifted grasp"}

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    can = list(state["objects"]["can"]["pos"])
    eef = list(state["eef_pos"])
    transit_z = max(can[2], goal[2] + 0.20) - offset[2]
    move([eef[0], eef[1], transit_z], "close", 25)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if can[2] < goal[2] + 0.14:
        return {"status": "failed", "reason": "Insufficient transport clearance"}

    move(
        [goal[0] - offset[0], goal[1] - offset[1], transit_z],
        "close",
        min(75, max(0, remaining() - 65)),
        0.018,
    )

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    eef = list(state["eef_pos"])
    if (
        abs(can[2] - eef[2] - offset[2]) > 0.045
        or hypot(
            can[0] - eef[0] - offset[0],
            can[1] - eef[1] - offset[1],
        ) > 0.04
        or state["gripper_width"] < 0.012
    ):
        return {"status": "failed", "reason": "Grasp lost during transport"}

    if hypot(can[0] - goal[0], can[1] - goal[1]) > 0.025:
        move(
            [
                eef[0] + goal[0] - can[0],
                eef[1] + goal[1] - can[1],
                eef[2],
            ],
            "close",
            min(20, max(0, remaining() - 55)),
            0.014,
        )

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    eef = list(state["eef_pos"])
    if hypot(can[0] - goal[0], can[1] - goal[1]) > 0.04:
        return {"status": "failed", "reason": "Could not align with target bin"}
    if remaining() < 40:
        return {"status": "failed", "reason": "Insufficient release budget"}

    move(
        [
            eef[0] + goal[0] - can[0],
            eef[1] + goal[1] - can[1],
            eef[2] + goal[2] + 0.065 - can[2],
        ],
        "close",
        min(35, remaining() - 30),
        0.018,
    )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.04
        or can[2] > goal[2] + 0.13
        or can[2] < goal[2] - 0.025
    ):
        return {"status": "failed", "reason": "Unsafe release position"}

    actuate("open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.14], "open", 22)
    actuate("open", 10)

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
