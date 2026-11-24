def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, action, ticks, tolerance=0.008):
        budget = min(ticks, remaining())
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
        if remaining() < 175:
            break

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        goal = state["goal_pos"]
        eef = state["eef_pos"]
        clearance = max(can[2] + 0.20, goal[2] + 0.23)

        move([eef[0], eef[1], max(eef[2], clearance)],
             yaw, "open", 20)
        move([can[0], can[1], clearance], yaw, "open", 40)

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.018:
            move([can[0], can[1], clearance], yaw, "open", 15)

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.022:
            continue

        grasp_height = can[2] + (0.005 if attempt == 0 else 0.015)
        move([can[0], can[1], grasp_height], yaw, "open", 35, 0.006)

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - can[0], eef[1] - can[1]) > 0.022
            or abs(eef[2] - grasp_height) > 0.025
        ):
            continue

        initial_height = can[2]
        grip("close", 16)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move([eef[0], eef[1], max(clearance, eef[2] + 0.15)],
             yaw, "close", 40)

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        held = (
            can[2] > initial_height + 0.055
            and hypot(can[0] - eef[0], can[1] - eef[1]) < 0.07
            and abs(can[2] - eef[2]) < 0.13
            and state["gripper_width"] > 0.012
        )
        if held:
            break
        grip("open", 8)

    if not held:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    eef = state["eef_pos"]
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    travel_height = max(eef[2], goal[2] + 0.24 + eef[2] - can[2])

    # Correct against the live can position and allow transport to converge.
    aligned = False
    for correction in range(4):
        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        goal = state["goal_pos"]

        if (
            state["gripper_width"] < 0.01
            or hypot(can[0] - eef[0], can[1] - eef[1]) > 0.10
            or abs(can[2] - eef[2]) > 0.16
        ):
            return {"status": "failed", "reason": "Can lost during transport"}

        if hypot(can[0] - goal[0], can[1] - goal[1]) < 0.022:
            aligned = True
            break

        budget = min(40, remaining() - 75)
        if budget <= 0:
            break
        move(
            [eef[0] + goal[0] - can[0],
             eef[1] + goal[1] - can[1],
             travel_height],
            yaw, "close", budget
        )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    aligned = aligned or hypot(can[0] - goal[0], can[1] - goal[1]) < 0.028
    if not aligned:
        return {"status": "failed", "reason": "Could not align can above target"}

    # Recompute the grasp offset after transport instead of using a stale offset.
    for correction in range(3):
        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        goal = state["goal_pos"]
        if (
            hypot(can[0] - goal[0], can[1] - goal[1]) < 0.028
            and goal[2] + 0.035 < can[2] < goal[2] + 0.09
        ):
            break

        budget = min(25, remaining() - 35)
        if budget <= 0:
            break
        move(
            [eef[0] + goal[0] - can[0],
             eef[1] + goal[1] - can[1],
             eef[2] + goal[2] + 0.065 - can[2]],
            yaw, "close", budget, 0.006
        )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.04
        or not goal[2] + 0.025 < can[2] < goal[2] + 0.12
        or remaining() < 25
    ):
        return {"status": "failed", "reason": "Could not reach safe release position"}

    grip("open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.16],
         yaw, "open", min(22, remaining() - 3))
    grip("open", 3)

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
