def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(pos, gripper=None, steps=40, tol=0.012):
        budget = min(steps, remaining())
        if budget <= 0:
            return False
        result = sdk.move_to(
            pos, gripper=gripper, max_steps=budget, tol=tol
        )
        return result["reached"]

    def grip(action, steps=15):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def distance(a, b):
        return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or goal"}

    held = False
    offset = None

    for attempt in range(2):
        if remaining() < 180:
            break

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        approach_z = max(state["eef_pos"][2], can[2] + 0.14)

        move(
            [state["eef_pos"][0], state["eef_pos"][1], approach_z],
            steps=20,
        )
        grip("open", steps=10)
        move([can[0], can[1], approach_z], "open", steps=35)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        if hypot(
            state["eef_pos"][0] - can[0],
            state["eef_pos"][1] - can[1],
        ) > 0.025:
            continue

        # A small positive offset avoids driving the fingers into the floor.
        grasp_z = can[2] + (0.025 if attempt == 0 else 0.012)
        move([can[0], can[1], grasp_z], "open", steps=35, tol=0.012)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])

        # Judge the actual grasp geometry rather than a strict move timeout.
        if (
            hypot(eef[0] - can[0], eef[1] - can[1]) > 0.018
            or eef[2] - can[2] > 0.055
            or eef[2] - can[2] < -0.025
        ):
            continue

        grip("close", steps=20)
        state = sdk.get_state()
        before = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        move(
            [eef[0], eef[1], eef[2] + 0.12],
            "close",
            steps=30,
            tol=0.012,
        )

        state = sdk.get_state()
        after = list(state["objects"]["can"]["pos"])
        if (
            after[2] - before[2] > 0.05
            and 0.012 < state["gripper_width"] < 0.075
            and distance(after, state["eef_pos"]) < 0.13
        ):
            offset = [after[i] - state["eef_pos"][i] for i in range(3)]
            held = True
            break

    if not held:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    can = list(state["objects"]["can"]["pos"])
    clearance_z = max(can[2], goal[2] + 0.20) - offset[2]
    move(
        [state["eef_pos"][0], state["eef_pos"][1], clearance_z],
        "close",
        steps=25,
    )

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    if can[2] < goal[2] + 0.15:
        return {"status": "failed", "reason": "Insufficient transport clearance"}

    move(
        [goal[0] - offset[0], goal[1] - offset[1], clearance_z],
        "close",
        steps=50,
        tol=0.012,
    )

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    expected = [state["eef_pos"][i] + offset[i] for i in range(3)]
    if distance(can, expected) > 0.045:
        return {"status": "failed", "reason": "Can slipped during transport"}

    offset = [can[i] - state["eef_pos"][i] for i in range(3)]
    goal = list(state["goal_pos"])
    if hypot(can[0] - goal[0], can[1] - goal[1]) > 0.025:
        move(
            [goal[0] - offset[0], goal[1] - offset[1], clearance_z],
            "close",
            steps=20,
        )

    if remaining() < 60:
        return {"status": "failed", "reason": "Insufficient time for release"}

    move(
        [
            goal[0] - offset[0],
            goal[1] - offset[1],
            goal[2] + 0.065 - offset[2],
        ],
        "close",
        steps=30,
        tol=0.012,
    )

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.035
        or can[2] > goal[2] + 0.11
        or can[2] < goal[2] - 0.015
    ):
        return {"status": "failed", "reason": "Can outside safe release region"}

    grip("open", steps=15)
    state = sdk.get_state()
    eef = list(state["eef_pos"])
    move([eef[0], eef[1], eef[2] + 0.14], "open", steps=20)
    grip("open", steps=10)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    placed = (
        hypot(can[0] - goal[0], can[1] - goal[1]) < 0.06
        and goal[2] - 0.025 < can[2] < goal[2] + 0.12
        and state["gripper_width"] > 0.06
        and distance(can, state["eef_pos"]) > 0.09
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Can released in target bin" if placed else "Placement unverified",
        "steps_used": state["steps_used"],
    }
