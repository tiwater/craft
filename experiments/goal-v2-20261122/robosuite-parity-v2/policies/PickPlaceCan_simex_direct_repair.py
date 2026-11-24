def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, grip, steps, tol=0.012):
        budget = min(steps, remaining())
        if budget <= 0:
            return False
        return sdk.move_to(
            pos, yaw=yaw, gripper=grip, max_steps=budget, tol=tol
        )["reached"]

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
    initial_can = list(state["objects"]["can"]["pos"])
    goal = list(state["goal_pos"])
    clearance = max(initial_can[2] + 0.15, goal[2] + 0.19)
    grasped = False
    offset = None

    for attempt in range(3):
        if remaining() < 160:
            break

        state = sdk.get_state()
        eef = state["eef_pos"]
        move([eef[0], eef[1], max(clearance, eef[2])], yaw, None, 25)
        actuate("open", 8)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        move([can[0], can[1], clearance], yaw, "open", 35)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        if distance_xy(state["eef_pos"], can) > 0.018:
            continue

        grasp_z = can[2] + (0.0 if attempt == 0 else 0.006)
        move([can[0], can[1], grasp_z], yaw, "open", 35, 0.006)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if distance_xy(eef, can) > 0.014 or abs(eef[2] - grasp_z) > 0.02:
            continue

        before_lift = list(can)
        actuate("close", 16)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move([eef[0], eef[1], eef[2] + 0.09], yaw, "close", 25)

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        if (
            can[2] - before_lift[2] > 0.045
            and distance_xy(can, eef) < 0.05
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
    safe_can_z = max(initial_can[2] + 0.12, goal[2] + 0.17)
    travel_z = max(eef[2], safe_can_z - offset[2])
    move([eef[0], eef[1], travel_z], yaw, "close", 25)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    if can[2] < safe_can_z - 0.025:
        return {"status": "failed", "reason": "Insufficient object clearance"}

    # Use the achieved safe height instead of requiring an exact high pose.
    travel_z = eef[2]
    offset = [can[i] - eef[i] for i in range(3)]
    move(
        [goal[0] - offset[0], goal[1] - offset[1], travel_z],
        yaw, "close", 50
    )

    for correction in range(2):
        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        goal = state["goal_pos"]
        expected_can = [eef[i] + offset[i] for i in range(3)]
        slip = sqrt(sum((can[i] - expected_can[i]) ** 2 for i in range(3)))
        if slip > 0.045:
            return {"status": "failed", "reason": "Can slipped during transport"}
        if distance_xy(can, goal) <= 0.018:
            break
        if remaining() < 85:
            break
        move(
            [eef[0] + goal[0] - can[0],
             eef[1] + goal[1] - can[1],
             travel_z],
            yaw, "close", 18, 0.008
        )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    if distance_xy(can, goal) > 0.035 or remaining() < 55:
        return {"status": "failed", "reason": "Could not establish safe release position"}

    eef = state["eef_pos"]
    release_can_z = goal[2] + 0.065
    move(
        [eef[0] + goal[0] - can[0],
         eef[1] + goal[1] - can[1],
         eef[2] + release_can_z - can[2]],
        yaw, "close", min(30, remaining() - 30), 0.01
    )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    if (
        distance_xy(can, goal) > 0.035
        or can[2] < goal[2] + 0.025
        or can[2] > goal[2] + 0.105
    ):
        return {"status": "failed", "reason": "Release pose was not reached"}

    actuate("open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], max(travel_z, eef[2] + 0.11)],
        yaw, "open", min(22, remaining())
    )
    actuate("open", min(10, remaining()))

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    placed = (
        distance_xy(can, goal) < 0.06
        and goal[2] - 0.025 < can[2] < goal[2] + 0.12
        and state["eef_pos"][2] - can[2] > 0.08
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Can released in target bin" if placed else "Placement verification failed",
        "steps_used": state["steps_used"],
    }
