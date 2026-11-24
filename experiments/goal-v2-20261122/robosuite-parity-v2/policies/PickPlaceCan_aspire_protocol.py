def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, grip, steps, tol=0.01):
        budget = min(steps, remaining())
        if budget <= 0:
            return False
        result = sdk.move_to(
            pos, yaw=yaw, gripper=grip, max_steps=budget, tol=tol
        )
        return result["reached"]

    def actuate(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def distance_xy(a, b):
        return hypot(a[0] - b[0], a[1] - b[1])

    def failure(reason):
        return {
            "status": "failed",
            "reason": reason,
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return failure("Missing can or target bin")

    yaw = state["eef_yaw"]
    initial_can = list(state["objects"]["can"]["pos"])
    goal = list(state["goal_pos"])
    # The failed rollouts retained a lifted can near an apparent height limit.
    # Use modest clearance and verify physical clearance, not exact convergence.
    approach_z = max(initial_can[2] + 0.15, goal[2] + 0.18)
    grasped = False
    offset = None

    for attempt in range(2):
        if remaining() < 190:
            break

        state = sdk.get_state()
        eef = state["eef_pos"]
        move(
            [eef[0], eef[1], max(approach_z, eef[2])],
            yaw, None, 24
        )
        actuate("open", 10)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        move([can[0], can[1], approach_z], yaw, "open", 36)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        if distance_xy(state["eef_pos"], can) > 0.018:
            continue

        grasp_z = can[2] + 0.004 * attempt
        move([can[0], can[1], grasp_z], yaw, "open", 32, 0.006)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if distance_xy(eef, can) > 0.014 or abs(eef[2] - grasp_z) > 0.018:
            continue

        before_lift = list(can)
        actuate("close", 18)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move([eef[0], eef[1], eef[2] + 0.10], yaw, "close", 26)

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        if (
            can[2] - before_lift[2] > 0.05
            and distance_xy(can, eef) < 0.045
            and 0.012 < state["gripper_width"] < 0.075
        ):
            grasped = True
            offset = [can[i] - eef[i] for i in range(3)]
            break

    if not grasped:
        return failure("Could not verify a lifted grasp")

    state = sdk.get_state()
    eef = state["eef_pos"]
    goal = list(state["goal_pos"])
    desired_can_z = max(initial_can[2] + 0.14, goal[2] + 0.18)
    travel_z = max(eef[2], desired_can_z - offset[2])
    move([eef[0], eef[1], travel_z], yaw, "close", 24)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    if (
        can[2] < max(initial_can[2] + 0.10, goal[2] + 0.14)
        or distance_xy(can, eef) > 0.05
    ):
        return failure("Insufficient object clearance")

    # Continue from the achieved height even if the requested height saturated.
    travel_z = eef[2]
    offset = [can[i] - eef[i] for i in range(3)]
    if remaining() < 105:
        return failure("Insufficient transport and release budget")

    for segment in range(2):
        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        goal = list(state["goal_pos"])
        fraction = 0.5 if segment == 0 else 1.0
        target = [
            eef[0] + fraction * (goal[0] - can[0]),
            eef[1] + fraction * (goal[1] - can[1]),
            travel_z,
        ]
        move(target, yaw, "close", min(36, max(0, remaining() - 65)))

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        slip = sqrt(sum(
            (can[i] - eef[i] - offset[i]) ** 2 for i in range(3)
        ))
        if slip > 0.045 or state["gripper_width"] < 0.01:
            return failure("Can slipped during transport")

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    goal = list(state["goal_pos"])
    if distance_xy(can, goal) > 0.018:
        move(
            [
                eef[0] + goal[0] - can[0],
                eef[1] + goal[1] - can[1],
                travel_z,
            ],
            yaw, "close", min(24, max(0, remaining() - 60)), 0.008
        )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    goal = list(state["goal_pos"])
    if distance_xy(can, goal) > 0.035 or remaining() < 50:
        return failure("Could not establish safe release position")

    offset = [can[i] - eef[i] for i in range(3)]
    release_can_z = goal[2] + 0.075
    move(
        [
            goal[0] - offset[0],
            goal[1] - offset[1],
            release_can_z - offset[2],
        ],
        yaw, "close", min(30, remaining() - 35), 0.01
    )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if (
        distance_xy(can, goal) > 0.04
        or can[2] < goal[2] + 0.035
        or can[2] > goal[2] + 0.125
    ):
        return failure("Release pose was not reached")

    actuate("open", 16)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], max(travel_z, eef[2] + 0.10)],
        yaw, "open", min(24, max(0, remaining() - 8))
    )
    actuate("open", min(12, remaining()))

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    placed = (
        distance_xy(can, goal) < 0.06
        and goal[2] - 0.025 < can[2] < goal[2] + 0.12
        and state["eef_pos"][2] - can[2] > 0.08
        and state["gripper_width"] > 0.06
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Can released in target bin" if placed else "Placement verification failed",
        "steps_used": state["steps_used"],
    }
