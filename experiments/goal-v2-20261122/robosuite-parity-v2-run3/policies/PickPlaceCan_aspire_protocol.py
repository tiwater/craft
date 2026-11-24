def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(position, grip, limit=30, tol=0.01):
        budget = min(limit, remaining())
        if budget <= 0:
            return False
        return sdk.move_to(
            position, gripper=grip, max_steps=budget, tol=tol
        )["reached"]

    def actuate(action, ticks):
        budget = min(ticks, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def distance(a, b):
        return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or target"}

    initial_height = state["objects"]["can"]["pos"][2]
    goal = list(state["goal_pos"])
    grasped = False
    offset = [0.0, 0.0, 0.0]

    # A closed gripper followed by an arm lift does not establish a grasp.
    # Seat the fingers lower on retries and verify that the can follows.
    for attempt in range(3):
        if remaining() < 195:
            break

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        hover_z = max(can[2] + 0.12, eef[2])

        if eef[2] < can[2] + 0.085:
            move([eef[0], eef[1], hover_z], None, 18)

        actuate("open", 8)
        move([can[0], can[1], hover_z], "open", 30)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.02:
            move([can[0], can[1], hover_z], "open", 12, 0.008)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.022:
            continue

        grasp_offset = 0.015 if attempt == 0 else 0.0
        move(
            [can[0], can[1], can[2] + grasp_offset],
            "open", 28, 0.006
        )

        # A timed-out descent can leave the fingers above useful contact.
        # Continue toward the live object before deciding whether to close.
        for correction in range(2):
            state = sdk.get_state()
            can = list(state["objects"]["can"]["pos"])
            eef = list(state["eef_pos"])
            lateral = hypot(eef[0] - can[0], eef[1] - can[1])
            height = eef[2] - can[2]
            if lateral <= 0.012 and -0.012 <= height <= 0.03:
                break
            if remaining() < 195:
                break
            move(
                [can[0], can[1], can[2] + grasp_offset],
                "open", 12, 0.005
            )

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        if (
            hypot(eef[0] - can[0], eef[1] - can[1]) > 0.018
            or not (-0.015 <= eef[2] - can[2] <= 0.038)
        ):
            continue

        actuate("close", 16)
        state = sdk.get_state()
        before_can = list(state["objects"]["can"]["pos"])
        before_eef = list(state["eef_pos"])
        move(
            [before_eef[0], before_eef[1], before_eef[2] + 0.085],
            "close", 24, 0.008
        )

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        can_rise = can[2] - before_can[2]
        eef_rise = eef[2] - before_eef[2]
        if (
            can_rise > 0.045
            and abs(can_rise - eef_rise) < 0.025
            and distance(can, eef) < 0.09
            and state["gripper_width"] > 0.012
        ):
            grasped = True
            offset = [can[i] - eef[i] for i in range(3)]
            break

    if not grasped:
        return {"status": "failed", "reason": "Could not verify a lifted can"}

    state = sdk.get_state()
    cruise_can_z = max(initial_height, goal[2]) + 0.19
    cruise_eef_z = max(state["eef_pos"][2], cruise_can_z - offset[2])
    eef = list(state["eef_pos"])
    move([eef[0], eef[1], cruise_eef_z], "close", 25)

    state = sdk.get_state()
    if state["objects"]["can"]["pos"][2] < cruise_can_z - 0.03:
        return {"status": "failed", "reason": "Insufficient bin clearance"}

    start = list(state["eef_pos"])
    target = [goal[0] - offset[0], goal[1] - offset[1], cruise_eef_z]

    for segment in range(1, 4):
        if remaining() < 65:
            return {"status": "failed", "reason": "Insufficient placement budget"}

        fraction = segment / 3.0
        move(
            [
                start[0] + fraction * (target[0] - start[0]),
                start[1] + fraction * (target[1] - start[1]),
                cruise_eef_z,
            ],
            "close", min(28, remaining() - 60), 0.012
        )

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        current_offset = [can[i] - eef[i] for i in range(3)]
        if (
            distance(current_offset, offset) > 0.045
            or can[2] < initial_height + 0.075
        ):
            return {"status": "failed", "reason": "Can slipped during transport"}

    for correction in range(2):
        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        if hypot(can[0] - goal[0], can[1] - goal[1]) <= 0.018:
            break
        if remaining() < 65:
            break
        move(
            [
                eef[0] + goal[0] - can[0],
                eef[1] + goal[1] - can[1],
                cruise_eef_z,
            ],
            "close", min(20, remaining() - 55), 0.008
        )

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    eef = list(state["eef_pos"])
    if hypot(can[0] - goal[0], can[1] - goal[1]) > 0.035:
        return {"status": "failed", "reason": "Can is not over the target"}

    release_height = max(goal[2] + 0.025, initial_height + 0.035)
    move(
        [
            eef[0] + goal[0] - can[0],
            eef[1] + goal[1] - can[1],
            eef[2] + release_height - can[2],
        ],
        "close", min(32, max(0, remaining() - 35)), 0.01
    )

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.04
        or can[2] > release_height + 0.04
        or remaining() < 25
    ):
        return {"status": "failed", "reason": "Could not reach release position"}

    actuate("open", 12)
    state = sdk.get_state()
    eef = list(state["eef_pos"])
    move(
        [eef[0], eef[1], eef[2] + 0.14],
        "open", min(20, max(0, remaining() - 8)), 0.015
    )
    actuate("open", 8)

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    xy_error = hypot(can[0] - goal[0], can[1] - goal[1])
    placed = (
        xy_error < 0.06
        and goal[2] - 0.06 <= can[2] <= goal[2] + 0.04
        and state["eef_pos"][2] - can[2] > 0.09
        and state["gripper_width"] > 0.06
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Placement verified" if placed else "Placement not verified",
        "target_xy_error_m": xy_error,
        "steps_used": state["steps_used"],
    }
