def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def distance_xy(a, b):
        return hypot(a[0] - b[0], a[1] - b[1])

    def move(pos, action, steps, tol=0.012):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.move_to(
                list(pos), gripper=action, max_steps=budget, tol=tol
            )
        return sdk.get_state()

    def grip(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)
        return sdk.get_state()

    def failure(reason):
        state = sdk.get_state()
        return {
            "status": "failed",
            "reason": reason,
            "steps_used": state["steps_used"],
        }

    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return failure("Missing can or goal")

    source_center_z = state["objects"]["can"]["pos"][2]
    carrying = False

    for attempt in range(2):
        if remaining() < 190:
            break

        state = grip("open", 12)
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        approach_z = max(1.10, can[2] + 0.16, eef[2])

        state = move([eef[0], eef[1], approach_z], "open", 35)
        eef = state["eef_pos"]
        if eef[2] < can[2] + 0.12:
            state = move([eef[0], eef[1], approach_z], "open", 15)
            if state["eef_pos"][2] < can[2] + 0.12:
                return failure("Could not establish approach clearance")

        can = list(state["objects"]["can"]["pos"])
        state = move([can[0], can[1], approach_z], "open", 55)
        can = list(state["objects"]["can"]["pos"])

        if distance_xy(state["eef_pos"], can) > 0.018:
            state = move([can[0], can[1], approach_z], "open", 20)
            can = list(state["objects"]["can"]["pos"])

        if distance_xy(state["eef_pos"], can) > 0.025:
            return failure("Could not align above live can position")

        state = move([can[0], can[1], can[2]], "open", 50, 0.006)

        for correction in range(2):
            can = list(state["objects"]["can"]["pos"])
            eef = state["eef_pos"]
            if (
                distance_xy(eef, can) <= 0.009
                and abs(eef[2] - can[2]) <= 0.010
            ):
                break
            if remaining() < 175:
                break
            state = move([can[0], can[1], can[2]], "open", 12, 0.005)

        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if (
            distance_xy(eef, can) > 0.018
            or abs(eef[2] - can[2]) > 0.020
        ):
            return failure("Could not reach can center accurately")

        before = list(can)
        state = grip("close", 22)
        eef = list(state["eef_pos"])
        state = move([eef[0], eef[1], eef[2] + 0.11], "close", 32)
        can = state["objects"]["can"]["pos"]

        carrying = (
            can[2] - before[2] > 0.055
            and distance_xy(can, state["eef_pos"]) < 0.060
            and 0.012 < state["gripper_width"] < 0.075
        )
        if carrying:
            break

        eef = list(state["eef_pos"])
        state = move(
            [eef[0], eef[1], max(1.10, before[2] + 0.16)],
            "open",
            18,
        )

    if not carrying:
        return failure("Could not verify lifted grasp")

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    eef = list(state["eef_pos"])
    offset = [can[i] - eef[i] for i in range(3)]
    transit_z = max(1.13, 1.115 - offset[2], eef[2])

    state = move([eef[0], eef[1], transit_z], "close", 30)
    can = list(state["objects"]["can"]["pos"])
    if can[2] < 1.10:
        eef = list(state["eef_pos"])
        state = move(
            [eef[0], eef[1], eef[2] + 1.115 - can[2]],
            "close",
            15,
        )
        can = list(state["objects"]["can"]["pos"])

    if can[2] < 1.095 or state["gripper_width"] <= 0.012:
        return failure("Can did not clear divider")

    if remaining() < 95:
        return failure("Insufficient placement budget")

    goal = list(state["goal_pos"])
    eef = list(state["eef_pos"])
    offset = [can[i] - eef[i] for i in range(3)]
    transit_z = max(transit_z, eef[2])
    target = [goal[0] - offset[0], goal[1] - offset[1], transit_z]

    state = move(target, "close", min(65, remaining() - 65))
    can = list(state["objects"]["can"]["pos"])
    if can[2] < 1.095 or state["gripper_width"] <= 0.012:
        return failure("Grasp lost during transfer")

    if distance_xy(can, goal) > 0.025 and remaining() > 80:
        eef = list(state["eef_pos"])
        target = [
            eef[0] + goal[0] - can[0],
            eef[1] + goal[1] - can[1],
            transit_z,
        ]
        state = move(target, "close", 15, 0.008)
        can = list(state["objects"]["can"]["pos"])

    if distance_xy(can, goal) > 0.045:
        return failure("Could not align can inside target bin")

    eef = list(state["eef_pos"])
    offset = [can[i] - eef[i] for i in range(3)]
    release_center_z = max(goal[2] + 0.055, source_center_z + 0.010)
    target = [
        goal[0] - offset[0],
        goal[1] - offset[1],
        release_center_z - offset[2],
    ]
    state = move(target, "close", min(45, remaining() - 35), 0.010)
    can = list(state["objects"]["can"]["pos"])

    if (
        distance_xy(can, goal) > 0.050
        or can[2] > release_center_z + 0.035
        or remaining() < 25
    ):
        return failure("Could not lower can safely into target bin")

    state = grip("open", 16)
    eef = list(state["eef_pos"])
    state = move(
        [eef[0], eef[1], max(eef[2] + 0.13, transit_z)],
        "open",
        min(24, remaining()),
    )
    if remaining() > 0:
        state = grip("open", min(8, remaining()))

    can = list(state["objects"]["can"]["pos"])
    placed = (
        distance_xy(can, goal) < 0.075
        and can[2] <= release_center_z + 0.025
        and state["gripper_width"] > 0.060
        and state["eef_pos"][2] - can[2] > 0.080
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Placement verified" if placed else "Placement not verified",
        "can_pos": can,
        "steps_used": state["steps_used"],
    }
