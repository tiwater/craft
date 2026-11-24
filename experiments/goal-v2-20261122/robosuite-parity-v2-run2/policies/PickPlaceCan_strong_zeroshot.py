def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(pos, gripper, limit, tol=0.01):
        budget = min(limit, remaining())
        if budget <= 0:
            return False
        result = sdk.move_to(
            pos, gripper=gripper, max_steps=budget, tol=tol
        )
        return result["reached"]

    def grip(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or target position"}

    held = False
    offset = None

    for attempt in range(2):
        if remaining() < 180:
            break

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        goal = list(state["goal_pos"])
        clearance = max(
            state["eef_pos"][2], can[2] + 0.20, goal[2] + 0.24
        )

        grip("open", 10)
        state = sdk.get_state()
        if not move(
            [state["eef_pos"][0], state["eef_pos"][1], clearance],
            "open", 25
        ):
            continue

        if not move([can[0], can[1], clearance], "open", 40):
            continue

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        grasp_height = can[2] + 0.005

        if not move([can[0], can[1], grasp_height], "open", 35, 0.006):
            continue

        grip("close", 20)
        before = sdk.get_state()
        original_can = list(before["objects"]["can"]["pos"])
        original_eef = list(before["eef_pos"])
        initial_offset = [
            original_can[i] - original_eef[i] for i in range(3)
        ]

        move(
            [original_eef[0], original_eef[1], original_eef[2] + 0.12],
            "close", 30
        )
        state = sdk.get_state()
        lifted_can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        offset = [lifted_can[i] - eef[i] for i in range(3)]
        drift = sqrt(sum(
            (offset[i] - initial_offset[i]) ** 2 for i in range(3)
        ))

        held = (
            lifted_can[2] - original_can[2] > 0.055
            and 0.012 < state["gripper_width"] < 0.075
            and drift < 0.04
        )
        if held:
            break

    if not held:
        return {"status": "failed", "reason": "Could not verify a stable grasp"}

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    eef = state["eef_pos"]
    transit_height = max(
        eef[2], goal[2] + 0.24 - offset[2],
        state["objects"]["can"]["pos"][2] + 0.10 - offset[2]
    )

    if not move([eef[0], eef[1], transit_height], "close", 25):
        return {"status": "failed", "reason": "Could not reach safe clearance"}

    target_x = goal[0] - offset[0]
    target_y = goal[1] - offset[1]
    if not move([target_x, target_y, transit_height], "close", 50):
        return {"status": "failed", "reason": "Could not reach target bin"}

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    current_offset = [
        can[i] - state["eef_pos"][i] for i in range(3)
    ]
    if sqrt(sum(
        (current_offset[i] - offset[i]) ** 2 for i in range(3)
    )) > 0.045:
        return {"status": "failed", "reason": "Can slipped during transport"}

    if remaining() < 55:
        return {"status": "failed", "reason": "Insufficient budget for release"}

    goal = list(state["goal_pos"])
    release_pos = [
        goal[0] - current_offset[0],
        goal[1] - current_offset[1],
        goal[2] + 0.075 - current_offset[2]
    ]
    if not move(release_pos, "close", 30, 0.008):
        return {"status": "failed", "reason": "Could not reach release position"}

    grip("open", 15)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.16], "open", 25)
    grip("open", 15)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    xy_error = hypot(can[0] - goal[0], can[1] - goal[1])
    placed = (
        xy_error < 0.065
        and goal[2] - 0.025 < can[2] < goal[2] + 0.14
        and state["gripper_width"] > 0.06
        and state["eef_pos"][2] - can[2] > 0.09
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Placement verified geometrically" if placed
                  else "Final placement could not be verified",
        "target_xy_error_m": xy_error,
        "steps_used": state["steps_used"]
    }
