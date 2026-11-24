def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return min(400, state["max_steps"]) - state["steps_used"]

    def move(pos, gripper, steps, tol=0.01):
        budget = min(steps, remaining())
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
        return {"status": "failed", "reason": "Missing can or target"}

    held = False
    offset = None

    for attempt in range(2):
        if remaining() < 165:
            break

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        goal = state["goal_pos"]
        approach_z = max(
            state["eef_pos"][2], can[2] + 0.16, goal[2] + 0.20
        )

        # Clear the source bin before moving horizontally.
        if not move(
            [state["eef_pos"][0], state["eef_pos"][1], approach_z],
            "open",
            30,
        ):
            continue

        if not move([can[0], can[1], approach_z], "open", 40):
            continue

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        grasp_z = can[2] + (0.005 if attempt == 0 else -0.005)

        if not move([can[0], can[1], grasp_z], "open", 35, 0.006):
            continue

        grip("close", 18)
        state = sdk.get_state()
        before = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])

        move([eef[0], eef[1], eef[2] + 0.15], "close", 35)
        state = sdk.get_state()
        lifted = state["objects"]["can"]["pos"]
        width = state["gripper_width"]
        separation = sqrt(sum(
            (state["eef_pos"][i] - lifted[i]) ** 2 for i in range(3)
        ))

        if (
            lifted[2] - before[2] > 0.075
            and 0.012 < width < 0.078
            and separation < 0.13
        ):
            held = True
            offset = [
                lifted[i] - state["eef_pos"][i] for i in range(3)
            ]
            break

        grip("open", 10)

    if not held:
        return {"status": "failed", "reason": "Could not verify grasp"}

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    can = state["objects"]["can"]["pos"]
    cruise_z = max(can[2], goal[2] + 0.23)
    eef = state["eef_pos"]

    if not move(
        [eef[0], eef[1], cruise_z - offset[2]], "close", 25
    ):
        return {"status": "failed", "reason": "Could not reach clearance"}

    if not move(
        [goal[0] - offset[0], goal[1] - offset[1], cruise_z - offset[2]],
        "close",
        55,
        0.008,
    ):
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

    if remaining() < 45:
        return {"status": "failed", "reason": "Insufficient placement budget"}

    # Account for the measured grasp offset when lowering the can.
    release_center_z = goal[2] + 0.075
    reached = move(
        [
            goal[0] - current_offset[0],
            goal[1] - current_offset[1],
            release_center_z - current_offset[2],
        ],
        "close",
        min(40, remaining() - 30),
        0.008,
    )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    xy_error = hypot(can[0] - goal[0], can[1] - goal[1])
    if not reached or xy_error > 0.035:
        return {"status": "failed", "reason": "Placement alignment failed"}

    grip("open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.14], "open", 20)
    grip("open", 12)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    xy_error = hypot(can[0] - goal[0], can[1] - goal[1])
    released = state["eef_pos"][2] - can[2] > 0.09
    placed = (
        xy_error < 0.045
        and goal[2] - 0.035 < can[2] < goal[2] + 0.13
        and released
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Can released in target bin" if placed else "Placement unverified",
        "target_xy_error_m": xy_error,
        "steps_used": state["steps_used"],
    }
