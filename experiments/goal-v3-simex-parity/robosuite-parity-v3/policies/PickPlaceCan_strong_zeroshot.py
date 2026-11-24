def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(pos, gripper, limit=40, tol=0.01):
        budget = remaining()
        if budget < 1:
            return False
        result = sdk.move_to(
            pos,
            gripper=gripper,
            max_steps=min(limit, budget),
            tol=tol,
        )
        return result["reached"]

    def actuate(action, steps):
        budget = remaining()
        if budget > 0:
            return sdk.set_gripper(action, steps=min(steps, budget))
        return None

    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or goal"}

    held = False
    offset = None

    for attempt in range(2):
        if remaining() < 155:
            break

        actuate("open", 8)
        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        clearance = max(eef[2], can[2] + 0.16, state["goal_pos"][2] + 0.22)

        if eef[2] < clearance - 0.015:
            move([eef[0], eef[1], clearance], "open", 25)

        if not move([can[0], can[1], clearance], "open", 40):
            continue

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        grasp_z = can[2] + (0.005 if attempt == 0 else 0.015)

        if not move([can[0], can[1], grasp_z], "open", 38, 0.007):
            continue

        actuate("close", 16)
        state = sdk.get_state()
        before = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])

        move([eef[0], eef[1], eef[2] + 0.11], "close", 30)
        state = sdk.get_state()
        after = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        width = state["gripper_width"]

        if (
            after[2] - before[2] > 0.055
            and 0.012 < width < 0.075
            and hypot(after[0] - eef[0], after[1] - eef[1]) < 0.055
            and abs(after[2] - eef[2]) < 0.11
        ):
            held = True
            offset = [after[i] - eef[i] for i in range(3)]
            break

    if not held:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    eef = list(state["eef_pos"])
    can = state["objects"]["can"]["pos"]
    transit_can_z = max(can[2] + 0.06, goal[2] + 0.24)
    transit_eef_z = transit_can_z - offset[2]

    if not move([eef[0], eef[1], transit_eef_z], "close", 30):
        return {"status": "failed", "reason": "Could not reach safe clearance"}

    target = [goal[0] - offset[0], goal[1] - offset[1], transit_eef_z]
    if not move(target, "close", 55, 0.009):
        return {"status": "failed", "reason": "Could not reach target bin"}

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    if (
        can[2] < goal[2] + 0.12
        or hypot(can[0] - eef[0] - offset[0],
                 can[1] - eef[1] - offset[1]) > 0.04
        or abs(can[2] - eef[2] - offset[2]) > 0.045
    ):
        return {"status": "failed", "reason": "Grasp lost during transport"}

    offset = [can[i] - eef[i] for i in range(3)]
    placement = [
        goal[0] - offset[0],
        goal[1] - offset[1],
        goal[2] + 0.06 - offset[2],
    ]
    if remaining() < 45:
        return {"status": "failed", "reason": "Insufficient time to release safely"}

    move(placement, "close", min(40, remaining() - 30), 0.008)
    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]

    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.035
        or can[2] > goal[2] + 0.11
        or can[2] < goal[2] - 0.02
    ):
        return {"status": "failed", "reason": "Can is not safely positioned for release"}

    actuate("open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.14], "open", 23)
    actuate("open", 12)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    eef = state["eef_pos"]
    placed = (
        hypot(can[0] - goal[0], can[1] - goal[1]) < 0.055
        and goal[2] - 0.025 < can[2] < goal[2] + 0.115
        and state["gripper_width"] > 0.055
        and eef[2] - can[2] > 0.075
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Can released in target bin" if placed else "Placement verification failed",
        "steps_used": state["steps_used"],
    }
