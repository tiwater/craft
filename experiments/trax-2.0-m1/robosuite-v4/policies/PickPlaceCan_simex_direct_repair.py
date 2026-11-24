def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, state["max_steps"] - state["steps_used"])

    def move(position, yaw, grip, steps, reserve=0, tol=0.008):
        budget = min(steps, max(0, remaining() - reserve))
        if budget > 0:
            return sdk.move_to(
                position, yaw=yaw, gripper=grip,
                max_steps=budget, tol=tol
            )
        return {"reached": False}

    def actuate(action, steps, reserve=0):
        budget = min(steps, max(0, remaining() - reserve))
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or target bin"}

    yaw = state["eef_yaw"]
    grasped = False
    offset = [0.0, 0.0, 0.0]

    for attempt in range(2):
        if remaining() < 190:
            break

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        hand = list(state["eef_pos"])
        approach_z = max(hand[2], can[2] + 0.15)
        actuate("open", 8, 170)
        move([hand[0], hand[1], approach_z], yaw, "open", 30, 160)
        move([can[0], can[1], approach_z], yaw, "open", 45, 145)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        hand = state["eef_pos"]
        if hypot(hand[0] - can[0], hand[1] - can[1]) > 0.018:
            continue

        grasp_z = can[2] + (0.005 if attempt == 0 else 0.02)
        move([can[0], can[1], grasp_z], yaw, "open", 40, 125)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        hand = list(state["eef_pos"])
        if (
            hypot(hand[0] - can[0], hand[1] - can[1]) > 0.018
            or abs(hand[2] - grasp_z) > 0.018
        ):
            continue

        baseline_z = can[2]
        actuate("close", 22, 110)
        state = sdk.get_state()
        hand = list(state["eef_pos"])
        lift_target = [hand[0], hand[1], hand[2] + 0.12]
        move(lift_target, yaw, "close", 45, 95)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        hand = list(state["eef_pos"])
        separation = sqrt(sum((can[i] - hand[i]) ** 2 for i in range(3)))
        if can[2] > baseline_z + 0.055 and separation < 0.12:
            offset = [can[i] - hand[i] for i in range(3)]
            grasped = True
            break

        actuate("open", 12, 155)

    if not grasped:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    can = list(state["objects"]["can"]["pos"])
    hand = list(state["eef_pos"])
    clearance_z = max(can[2], goal[2] + 0.20)
    clearance_target = [hand[0], hand[1], clearance_z - offset[2]]

    for correction in range(2):
        move(clearance_target, yaw, "close", 35, 85)
        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        hand = list(state["eef_pos"])
        if can[2] >= clearance_z - 0.02:
            break
        if remaining() <= 85:
            break

    if can[2] < clearance_z - 0.025:
        return {"status": "failed", "reason": "Insufficient bin clearance"}

    offset = [can[i] - hand[i] for i in range(3)]
    move(
        [goal[0] - offset[0], goal[1] - offset[1],
         clearance_z - offset[2]],
        yaw, "close", 65, 60
    )

    for correction in range(2):
        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        hand = list(state["eef_pos"])
        drift = sqrt(sum(
            (can[i] - hand[i] - offset[i]) ** 2 for i in range(3)
        ))
        if drift > 0.055:
            return {"status": "failed", "reason": "Can slipped during transport"}
        if hypot(can[0] - goal[0], can[1] - goal[1]) <= 0.025:
            break
        move(
            [hand[0] + goal[0] - can[0],
             hand[1] + goal[1] - can[1], hand[2]],
            yaw, "close", 25, 55
        )

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    hand = list(state["eef_pos"])
    if hypot(can[0] - goal[0], can[1] - goal[1]) > 0.04:
        return {"status": "failed", "reason": "Could not center can over bin"}

    offset = [can[i] - hand[i] for i in range(3)]
    move(
        [goal[0] - offset[0], goal[1] - offset[1],
         goal[2] + 0.085 - offset[2]],
        yaw, "close", 45, 30
    )

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.045
        or can[2] > goal[2] + 0.145
        or remaining() < 25
    ):
        return {"status": "failed", "reason": "Unsafe release position"}

    actuate("open", 15, 15)
    state = sdk.get_state()
    hand = list(state["eef_pos"])
    move([hand[0], hand[1], hand[2] + 0.15],
         yaw, "open", 25, 5)
    actuate("open", 8)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    placed = (
        hypot(can[0] - goal[0], can[1] - goal[1]) < 0.065
        and goal[2] - 0.025 < can[2] < goal[2] + 0.13
        and state["gripper_width"] > 0.06
        and state["eef_pos"][2] - can[2] > 0.09
    )
    return {
        "status": "done" if placed else "failed",
        "steps_used": state["steps_used"],
    }
