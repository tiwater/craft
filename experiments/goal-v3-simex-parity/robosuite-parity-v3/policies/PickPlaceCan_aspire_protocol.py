def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(pos, gripper, limit=35, tol=0.012):
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

    def offset_from(state):
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        return [can[i] - eef[i] for i in range(3)]

    def retained(state, offset):
        current = offset_from(state)
        return (
            0.012 < state["gripper_width"] < 0.075
            and hypot(current[0] - offset[0], current[1] - offset[1]) < 0.035
            and abs(current[2] - offset[2]) < 0.04
        )

    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or goal"}

    held = False
    offset = None

    for attempt in range(2):
        if remaining() < 165:
            break

        actuate("open", 8)
        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        clearance = max(eef[2], can[2] + 0.15)

        move([eef[0], eef[1], clearance], "open", 22)
        move([can[0], can[1], clearance], "open", 40)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.025:
            continue

        # The previous descent stopped against contact near can_z + 0.024.
        # Grasp above that contact limit and judge the live grasp geometry.
        grasp_height = 0.030 if attempt == 0 else 0.023
        move(
            [can[0], can[1], can[2] + grasp_height],
            "open",
            32,
            0.009,
        )

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.012:
            move([can[0], can[1], eef[2]], "open", 12, 0.008)
            state = sdk.get_state()
            can = list(state["objects"]["can"]["pos"])
            eef = list(state["eef_pos"])

        if (
            hypot(eef[0] - can[0], eef[1] - can[1]) > 0.022
            or eef[2] - can[2] < 0.005
            or eef[2] - can[2] > 0.055
        ):
            continue

        before = list(can)
        actuate("close", 18)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        move([eef[0], eef[1], eef[2] + 0.12], "close", 30, 0.012)

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        if (
            can[2] - before[2] > 0.065
            and 0.012 < state["gripper_width"] < 0.075
            and hypot(can[0] - eef[0], can[1] - eef[1]) < 0.045
            and abs(can[2] - eef[2]) < 0.09
        ):
            held = True
            offset = offset_from(state)
            break

    if not held:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    can = state["objects"]["can"]["pos"]
    eef = list(state["eef_pos"])
    transit_z = max(can[2], goal[2] + 0.20)
    move([eef[0], eef[1], transit_z - offset[2]], "close", 25)

    state = sdk.get_state()
    if (
        not retained(state, offset)
        or state["objects"]["can"]["pos"][2] < goal[2] + 0.15
    ):
        return {"status": "failed", "reason": "Safe transport clearance not established"}

    move(
        [goal[0] - offset[0], goal[1] - offset[1], transit_z - offset[2]],
        "close",
        65,
        0.012,
    )

    state = sdk.get_state()
    if not retained(state, offset):
        return {"status": "failed", "reason": "Grasp lost during transport"}

    # Correct using the observed can position, including any grip settling.
    for correction in range(2):
        state = sdk.get_state()
        goal = list(state["goal_pos"])
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        if hypot(can[0] - goal[0], can[1] - goal[1]) < 0.018:
            break
        if remaining() < 70:
            break
        move(
            [
                eef[0] + goal[0] - can[0],
                eef[1] + goal[1] - can[1],
                eef[2],
            ],
            "close",
            20,
            0.008,
        )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = list(state["goal_pos"])
    if (
        not retained(state, offset)
        or hypot(can[0] - goal[0], can[1] - goal[1]) > 0.035
    ):
        return {"status": "failed", "reason": "Can not centered over target bin"}

    if remaining() < 50:
        return {"status": "failed", "reason": "Insufficient release budget"}

    offset = offset_from(state)
    move(
        [
            goal[0] - offset[0],
            goal[1] - offset[1],
            goal[2] + 0.055 - offset[2],
        ],
        "close",
        min(35, remaining() - 35),
        0.010,
    )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.035
        or can[2] < goal[2] - 0.025
        or can[2] > goal[2] + 0.10
    ):
        return {"status": "failed", "reason": "Unsafe release position"}

    actuate("open", 12)
    state = sdk.get_state()
    eef = list(state["eef_pos"])
    move([eef[0], eef[1], eef[2] + 0.15], "open", 20)
    actuate("open", min(12, remaining()))

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    eef = state["eef_pos"]
    placed = (
        hypot(can[0] - goal[0], can[1] - goal[1]) < 0.055
        and goal[2] - 0.025 < can[2] < goal[2] + 0.09
        and state["gripper_width"] > 0.055
        and eef[2] - can[2] > 0.08
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Can released in target bin" if placed else "Placement verification failed",
        "steps_used": state["steps_used"],
    }
