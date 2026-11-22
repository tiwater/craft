def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, gripper, steps=40, tol=0.01):
        budget = min(steps, remaining())
        if budget <= 0:
            return False
        result = sdk.move_to(
            pos, yaw=yaw, gripper=gripper, max_steps=budget, tol=tol
        )
        return result["reached"]

    def grip(action, steps=15):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def distance_xy(a, b):
        return hypot(a[0] - b[0], a[1] - b[1])

    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or goal"}

    yaw = state["eef_yaw"]
    held = False

    for attempt in range(2):
        if remaining() < 130:
            break

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        source_z = can[2]
        approach_z = max(1.10, source_z + 0.17)

        grip("open", 10)
        state = sdk.get_state()
        eef = state["eef_pos"]
        if eef[2] < approach_z - 0.015:
            move([eef[0], eef[1], approach_z], yaw, "open", 30)

        if not move([can[0], can[1], approach_z], yaw, "open", 35):
            continue

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        grasp_z = can[2] + (0.005 if attempt == 0 else -0.01)
        move([can[0], can[1], grasp_z], yaw, "open", 40, 0.006)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if distance_xy(eef, can) > 0.018 or abs(eef[2] - grasp_z) > 0.025:
            continue

        grip("close", 18)

        state = sdk.get_state()
        eef = list(state["eef_pos"])
        can = list(state["objects"]["can"]["pos"])
        lift_z = max(1.15, eef[2] + 0.19)
        move([eef[0], eef[1], lift_z], yaw, "close", 45)

        state = sdk.get_state()
        lifted = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        held = (
            lifted[2] > source_z + 0.07
            and distance_xy(lifted, eef) < 0.055
            and 0.012 < state["gripper_width"] < 0.075
        )
        if held:
            break

    if not held:
        return {"status": "failed", "reason": "Could not verify grasp"}

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    offset = [can[i] - eef[i] for i in range(3)]
    goal = list(state["goal_pos"])

    clearance_center_z = max(1.11, goal[2] + 0.22)
    travel_z = max(eef[2], clearance_center_z - offset[2])

    move([eef[0], eef[1], travel_z], yaw, "close", 30)
    state = sdk.get_state()
    if state["objects"]["can"]["pos"][2] < clearance_center_z - 0.015:
        return {"status": "failed", "reason": "Insufficient divider clearance"}

    target = [goal[0] - offset[0], goal[1] - offset[1], travel_z]
    move(target, yaw, "close", 55, 0.008)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    if can[2] < 1.04 or distance_xy(can, eef) > 0.075:
        return {"status": "failed", "reason": "Grasp lost during transfer"}

    if distance_xy(can, goal) > 0.015:
        corrected = [
            eef[0] + goal[0] - can[0],
            eef[1] + goal[1] - can[1],
            eef[2],
        ]
        move(corrected, yaw, "close", 25, 0.008)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    if distance_xy(can, goal) > 0.035:
        return {"status": "failed", "reason": "Could not align over target"}

    release_center_z = goal[2] + 0.065
    lower_target = [
        eef[0] + goal[0] - can[0],
        eef[1] + goal[1] - can[1],
        eef[2] + release_center_z - can[2],
    ]

    if remaining() < 45:
        return {"status": "failed", "reason": "Insufficient placement budget"}

    move(lower_target, yaw, "close", min(40, remaining() - 25), 0.01)
    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if distance_xy(can, goal) > 0.04 or can[2] > release_center_z + 0.04:
        return {"status": "failed", "reason": "Could not lower into target"}

    grip("open", 15)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], max(1.12, eef[2] + 0.16)], yaw, "open", 25)
    grip("open", 10)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    placed = (
        distance_xy(can, goal) < 0.065
        and goal[2] - 0.03 <= can[2] <= goal[2] + 0.14
        and state["gripper_width"] > 0.045
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Can placed in target" if placed else "Placement not verified",
        "steps_used": state["steps_used"],
    }
