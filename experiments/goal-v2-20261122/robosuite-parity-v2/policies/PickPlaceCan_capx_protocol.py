def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, grip, steps, tol=0.015):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.move_to(pos, yaw=yaw, gripper=grip, max_steps=budget, tol=tol)

    def actuate(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def distance_xy(a, b):
        return hypot(a[0] - b[0], a[1] - b[1])

    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or target bin"}

    yaw = state["eef_yaw"]
    goal = list(state["goal_pos"])
    initial_can = list(state["objects"]["can"]["pos"])
    clearance = max(initial_can[2], goal[2]) + 0.21
    grasped = False
    offset = None

    for attempt in range(2):
        if remaining() < 190:
            break

        state = sdk.get_state()
        eef = state["eef_pos"]
        move([eef[0], eef[1], max(clearance, eef[2])], yaw, None, 30)
        actuate("open", 10)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        move([can[0], can[1], clearance], yaw, "open", 40, 0.012)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        if distance_xy(state["eef_pos"], can) > 0.02:
            continue

        grasp_z = can[2] + 0.005
        move([can[0], can[1], grasp_z], yaw, "open", 40, 0.01)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if distance_xy(eef, can) > 0.018 or abs(eef[2] - grasp_z) > 0.025:
            continue

        before_lift = list(can)
        actuate("close", 18)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move([eef[0], eef[1], clearance], yaw, "close", 45)

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        if (
            can[2] - before_lift[2] > 0.065
            and distance_xy(can, eef) < 0.045
            and 0.012 < state["gripper_width"] < 0.075
        ):
            grasped = True
            offset = [can[i] - eef[i] for i in range(3)]
            break

    if not grasped:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    eef = state["eef_pos"]
    can = state["objects"]["can"]["pos"]
    minimum_can_height = max(initial_can[2], goal[2]) + 0.15
    travel_z = max(clearance, minimum_can_height - offset[2], eef[2])

    if can[2] < minimum_can_height:
        move([eef[0], eef[1], travel_z], yaw, "close", 25)
        state = sdk.get_state()
        if state["objects"]["can"]["pos"][2] < minimum_can_height - 0.015:
            return {"status": "failed", "reason": "Insufficient transport clearance"}

    move(
        [goal[0] - offset[0], goal[1] - offset[1], travel_z],
        yaw, "close", min(75, max(0, remaining() - 85)), 0.015
    )

    for correction in range(2):
        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        goal = list(state["goal_pos"])
        slip = sqrt(sum((can[i] - eef[i] - offset[i]) ** 2 for i in range(3)))
        if slip > 0.05 or state["gripper_width"] < 0.01:
            return {"status": "failed", "reason": "Can slipped during transport"}
        if distance_xy(can, goal) < 0.025:
            break
        if remaining() < 95:
            break
        move(
            [eef[0] + goal[0] - can[0], eef[1] + goal[1] - can[1], travel_z],
            yaw, "close", min(25, remaining() - 80), 0.012
        )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    if distance_xy(can, goal) > 0.04 or remaining() < 65:
        return {"status": "failed", "reason": "Could not establish safe release position"}

    offset = [can[i] - eef[i] for i in range(3)]
    release_height = goal[2] + 0.065
    move(
        [goal[0] - offset[0], goal[1] - offset[1], release_height - offset[2]],
        yaw, "close", min(45, remaining() - 40), 0.015
    )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if (
        distance_xy(can, goal) > 0.04
        or can[2] < goal[2] + 0.025
        or can[2] > release_height + 0.04
    ):
        return {"status": "failed", "reason": "Release pose was not reached"}

    actuate("open", 16)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], max(travel_z, eef[2] + 0.12)],
        yaw, "open", min(30, remaining()), 0.02
    )
    actuate("open", 8)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    placed = (
        distance_xy(can, goal) < 0.06
        and goal[2] - 0.025 < can[2] < goal[2] + 0.10
        and state["eef_pos"][2] - can[2] > 0.08
        and state["gripper_width"] > 0.065
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Can released in target bin" if placed else "Placement verification failed",
        "steps_used": state["steps_used"],
    }
