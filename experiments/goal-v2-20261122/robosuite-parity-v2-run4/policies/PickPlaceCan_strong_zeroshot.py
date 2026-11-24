def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return min(400, state["max_steps"]) - state["steps_used"]

    def move(pos, gripper, limit=50, tol=0.008):
        budget = remaining()
        if budget <= 0:
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
            sdk.set_gripper(action, steps=min(steps, budget))

    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or target bin"}

    grasped = False
    offset = [0.0, 0.0, 0.0]

    for attempt in range(2):
        if remaining() < 155:
            break

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        goal = list(state["goal_pos"])
        start_height = can[2]
        clearance = max(can[2], goal[2]) + 0.20
        eef = state["eef_pos"]

        if not move([eef[0], eef[1], clearance], None, 40):
            continue

        actuate("open", 10)
        if not move([can[0], can[1], clearance], "open", 45):
            continue

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        grasp_height = can[2] + (0.005 if attempt == 0 else -0.005)

        if not move(
            [can[0], can[1], grasp_height + 0.065], "open", 35
        ):
            continue

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        if not move([can[0], can[1], grasp_height], "open", 30, 0.005):
            continue

        actuate("close", 18)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move([eef[0], eef[1], clearance], "close", 45)

        state = sdk.get_state()
        lifted = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        separation = sqrt(sum((eef[i] - lifted[i]) ** 2 for i in range(3)))

        if (
            lifted[2] > start_height + 0.065
            and separation < 0.12
            and state["gripper_width"] > 0.012
        ):
            offset = [lifted[i] - eef[i] for i in range(3)]
            grasped = True
            break

    if not grasped:
        return {"status": "failed", "reason": "Could not establish a stable grasp"}

    if remaining() < 75:
        return {"status": "failed", "reason": "Insufficient time for placement"}

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    can = state["objects"]["can"]["pos"]
    transport_height = max(can[2], goal[2] + 0.22)
    target = [
        goal[0] - offset[0],
        goal[1] - offset[1],
        transport_height - offset[2],
    ]

    if not move(target, "close", min(55, remaining() - 50)):
        return {"status": "failed", "reason": "Could not reach target bin"}

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    tracking_error = sqrt(
        sum((can[i] - eef[i] - offset[i]) ** 2 for i in range(3))
    )
    if tracking_error > 0.045:
        return {"status": "failed", "reason": "Can slipped during transport"}

    goal = list(state["goal_pos"])
    placement = [
        goal[0] - offset[0],
        goal[1] - offset[1],
        goal[2] + 0.065 - offset[2],
    ]
    move(placement, "close", min(35, max(1, remaining() - 35)), 0.008)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    horizontal_error = hypot(can[0] - goal[0], can[1] - goal[1])
    if horizontal_error > 0.045 or can[2] > goal[2] + 0.15:
        return {"status": "failed", "reason": "Can is not positioned for release"}

    actuate("open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.14],
        "open",
        min(25, max(1, remaining() - 10)),
    )
    actuate("open", 10)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    horizontal_error = hypot(can[0] - goal[0], can[1] - goal[1])
    settled_in_bin = (
        horizontal_error < 0.06
        and goal[2] - 0.04 < can[2] < goal[2] + 0.13
        and state["gripper_width"] > 0.055
        and state["eef_pos"][2] - can[2] > 0.075
    )

    return {
        "status": "done" if settled_in_bin else "failed",
        "reason": "Can released in target bin" if settled_in_bin else "Placement verification failed",
        "horizontal_error_m": horizontal_error,
        "steps_used": state["steps_used"],
    }
