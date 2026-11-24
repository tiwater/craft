def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(target, gripper, limit=45, tol=0.012):
        budget = remaining()
        if budget <= 0:
            return False
        sdk.move_to(
            target,
            gripper=gripper,
            max_steps=min(limit, budget),
            tol=tol,
        )
        state = sdk.get_state()
        return sqrt(sum(
            (state["eef_pos"][i] - target[i]) ** 2 for i in range(3)
        )) < max(0.020, tol)

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
        if remaining() < 185:
            break

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        goal = list(state["goal_pos"])
        start_height = can[2]
        clearance = max(can[2], goal[2]) + 0.18
        eef = state["eef_pos"]

        actuate("open", 8)
        move([eef[0], eef[1], clearance], "open", 35, 0.015)

        state = sdk.get_state()
        if state["eef_pos"][2] < clearance - 0.035:
            return {"status": "failed", "reason": "Could not reach safe clearance"}

        can = list(state["objects"]["can"]["pos"])
        if not move([can[0], can[1], clearance], "open", 50, 0.012):
            continue

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        grasp_target = [can[0], can[1], can[2] + 0.003]
        move(grasp_target, "open", 55, 0.010)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - can[0], eef[1] - can[1]) > 0.015
            or abs(eef[2] - can[2]) > 0.025
        ):
            move([can[0], can[1], can[2]], "open", 18, 0.010)

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - can[0], eef[1] - can[1]) > 0.020
            or abs(eef[2] - can[2]) > 0.030
        ):
            continue

        actuate("close", 16)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move([eef[0], eef[1], clearance], "close", 45, 0.015)

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        separation = sqrt(sum((eef[i] - can[i]) ** 2 for i in range(3)))
        if (
            can[2] > start_height + 0.065
            and separation < 0.12
            and 0.012 < state["gripper_width"] < 0.075
        ):
            offset = [can[i] - eef[i] for i in range(3)]
            grasped = True
            break

    if not grasped:
        return {"status": "failed", "reason": "Could not establish a stable grasp"}

    if remaining() < 100:
        return {"status": "failed", "reason": "Insufficient placement budget"}

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    can = state["objects"]["can"]["pos"]
    target = [
        goal[0] - offset[0],
        goal[1] - offset[1],
        max(can[2], goal[2] + 0.18) - offset[2],
    ]
    move(target, "close", min(85, remaining() - 65), 0.012)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    tracking_error = sqrt(sum(
        (can[i] - eef[i] - offset[i]) ** 2 for i in range(3)
    ))
    if tracking_error > 0.045 or state["gripper_width"] < 0.012:
        return {"status": "failed", "reason": "Can slipped during transport"}

    goal = list(state["goal_pos"])
    if hypot(can[0] - goal[0], can[1] - goal[1]) > 0.035:
        move(target, "close", min(20, max(0, remaining() - 60)), 0.012)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    if hypot(can[0] - goal[0], can[1] - goal[1]) > 0.040:
        return {"status": "failed", "reason": "Could not align over target bin"}

    offset = [can[i] - eef[i] for i in range(3)]
    placement = [
        goal[0] - offset[0],
        goal[1] - offset[1],
        goal[2] + 0.065 - offset[2],
    ]
    move(placement, "close", min(45, max(0, remaining() - 30)), 0.012)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.045
        or can[2] > goal[2] + 0.15
        or remaining() < 15
    ):
        return {"status": "failed", "reason": "Unsafe release position"}

    actuate("open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.14],
        "open",
        min(25, max(0, remaining() - 5)),
        0.015,
    )
    actuate("open", min(8, remaining()))

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    horizontal_error = hypot(can[0] - goal[0], can[1] - goal[1])
    settled = (
        horizontal_error < 0.06
        and goal[2] - 0.04 < can[2] < goal[2] + 0.13
        and state["gripper_width"] > 0.055
        and state["eef_pos"][2] - can[2] > 0.075
    )
    return {
        "status": "done" if settled else "failed",
        "reason": "Can released in target bin" if settled else "Placement verification failed",
        "horizontal_error_m": horizontal_error,
        "steps_used": state["steps_used"],
    }
