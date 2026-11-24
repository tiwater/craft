def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(pos, grip, steps=35, tol=0.012):
        budget = remaining()
        if budget > 0:
            sdk.move_to(
                pos,
                gripper=grip,
                max_steps=min(steps, budget),
                tol=tol,
            )

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
        if remaining() < 145:
            break

        actuate("open", 8)
        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        goal = list(state["goal_pos"])
        eef = state["eef_pos"]
        start_height = can[2]
        clearance = max(start_height + 0.15, goal[2] + 0.20)

        move([eef[0], eef[1], clearance], "open", 25)
        move([can[0], can[1], clearance], "open", 35)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.025:
            move([can[0], can[1], clearance], "open", 15)
            state = sdk.get_state()
            can = list(state["objects"]["can"]["pos"])
            eef = state["eef_pos"]
            if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.035:
                continue

        move([can[0], can[1], can[2] + 0.055], "open", 22)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        grasp_height = can[2] + (0.0 if attempt == 0 else -0.008)
        move([can[0], can[1], grasp_height], "open", 28, 0.008)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - can[0], eef[1] - can[1]) > 0.025
            or abs(eef[2] - grasp_height) > 0.025
        ):
            move([can[0], can[1], grasp_height], "open", 12, 0.010)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - can[0], eef[1] - can[1]) > 0.035
            or abs(eef[2] - can[2]) > 0.045
        ):
            continue

        start_height = can[2]
        actuate("close", 16)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move([eef[0], eef[1], clearance], "close", 32)

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        separation = sqrt(sum((can[i] - eef[i]) ** 2 for i in range(3)))
        if can[2] > start_height + 0.055 and separation < 0.11:
            offset = [can[i] - eef[i] for i in range(3)]
            grasped = True
            break

    if not grasped:
        return {"status": "failed", "reason": "Can did not follow the gripper during lift"}

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    transport_height = max(can[2], start_height + 0.15, goal[2] + 0.20)
    move(
        [eef[0], eef[1], transport_height - offset[2]],
        "close",
        18,
    )
    move(
        [goal[0] - offset[0], goal[1] - offset[1], transport_height - offset[2]],
        "close",
        min(45, max(1, remaining() - 55)),
    )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    tracking_error = sqrt(
        sum((can[i] - eef[i] - offset[i]) ** 2 for i in range(3))
    )
    if tracking_error > 0.05:
        return {"status": "failed", "reason": "Can slipped during transport"}

    goal = list(state["goal_pos"])
    if hypot(can[0] - goal[0], can[1] - goal[1]) > 0.025:
        move(
            [
                eef[0] + goal[0] - can[0],
                eef[1] + goal[1] - can[1],
                eef[2],
            ],
            "close",
            min(20, max(1, remaining() - 45)),
        )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    offset = [can[i] - eef[i] for i in range(3)]
    move(
        [
            goal[0] - offset[0],
            goal[1] - offset[1],
            goal[2] + 0.065 - offset[2],
        ],
        "close",
        min(32, max(1, remaining() - 30)),
        0.012,
    )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.045
        or can[2] > goal[2] + 0.15
        or remaining() < 15
    ):
        return {"status": "failed", "reason": "Could not reach a safe release position"}

    actuate("open", 10)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.14],
        "open",
        min(22, max(1, remaining() - 8)),
    )
    actuate("open", 8)

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
