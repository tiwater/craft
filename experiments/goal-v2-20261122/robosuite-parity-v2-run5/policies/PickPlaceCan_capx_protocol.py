def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, action, limit, tolerance=0.01):
        budget = min(limit, remaining())
        if budget > 0:
            return sdk.move_to(
                pos, yaw=yaw, gripper=action,
                max_steps=budget, tol=tolerance
            )
        return {"reached": False}

    def grip(action, ticks):
        budget = min(ticks, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or target"}

    yaw = state["eef_yaw"]
    held = False

    for attempt in range(2):
        if remaining() < 190:
            break

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        goal = state["goal_pos"]
        eef = state["eef_pos"]
        clearance = max(can[2] + 0.20, goal[2] + 0.23)

        move([eef[0], eef[1], max(eef[2], clearance)],
             yaw, "open", 20)
        move([can[0], can[1], clearance], yaw, "open", 35)

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.025:
            continue

        grasp_height = can[2] + (0.005 if attempt == 0 else 0.015)
        move([can[0], can[1], grasp_height], yaw, "open", 30, 0.006)

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        if (hypot(eef[0] - can[0], eef[1] - can[1]) > 0.022
                or abs(eef[2] - grasp_height) > 0.03):
            continue

        initial_height = can[2]
        grip("close", 14)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move([eef[0], eef[1], max(clearance, eef[2] + 0.15)],
             yaw, "close", 35)

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        held = (
            can[2] > initial_height + 0.055
            and hypot(can[0] - eef[0], can[1] - eef[1]) < 0.065
            and abs(can[2] - eef[2]) < 0.12
            and state["gripper_width"] > 0.012
        )
        if held:
            break
        grip("open", 8)

    if not held:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    goal = state["goal_pos"]
    offset = [eef[i] - can[i] for i in range(3)]
    travel_height = max(eef[2], goal[2] + 0.25 + offset[2])

    aligned = False
    for segment in range(5):
        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        goal = state["goal_pos"]

        if (state["gripper_width"] < 0.012
                or abs((eef[2] - can[2]) - offset[2]) > 0.06
                or hypot(eef[0] - can[0], eef[1] - can[1]) > 0.08):
            return {"status": "failed", "reason": "Grasp lost during transport"}

        if hypot(can[0] - goal[0], can[1] - goal[1]) < 0.022:
            aligned = True
            break

        budget = min(35, remaining() - 90)
        if budget <= 0:
            break

        move(
            [eef[0] + goal[0] - can[0],
             eef[1] + goal[1] - can[1],
             travel_height],
            yaw, "close", budget, 0.01
        )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    aligned = hypot(can[0] - goal[0], can[1] - goal[1]) < 0.03
    if not aligned:
        return {"status": "failed", "reason": "Could not align above target bin"}

    for segment in range(3):
        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        goal = state["goal_pos"]
        if (hypot(can[0] - goal[0], can[1] - goal[1]) < 0.03
                and goal[2] + 0.035 <= can[2] <= goal[2] + 0.09):
            break

        budget = min(25, remaining() - 35)
        if budget <= 0:
            break

        move(
            [eef[0] + goal[0] - can[0],
             eef[1] + goal[1] - can[1],
             eef[2] + goal[2] + 0.06 - can[2]],
            yaw, "close", budget, 0.008
        )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    if (hypot(can[0] - goal[0], can[1] - goal[1]) > 0.04
            or can[2] > goal[2] + 0.12
            or can[2] < goal[2] - 0.02
            or remaining() < 25):
        return {"status": "failed", "reason": "Could not reach safe release position"}

    grip("open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.16],
         yaw, "open", min(25, max(0, remaining() - 6)))
    grip("open", 6)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    placed = (
        hypot(can[0] - goal[0], can[1] - goal[1]) < 0.065
        and goal[2] - 0.04 < can[2] < goal[2] + 0.15
        and state["gripper_width"] > 0.055
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Can released in target bin" if placed else "Placement verification failed",
        "steps_used": state["steps_used"],
    }
