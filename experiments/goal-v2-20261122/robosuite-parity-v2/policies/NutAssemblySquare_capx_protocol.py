def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def wrap(angle):
        return atan2(sin(angle), cos(angle))

    def square_error(angle):
        return atan2(sin(4.0 * angle), cos(4.0 * angle)) / 4.0

    def move(pos, yaw, grip, steps, tol=0.004):
        budget = min(steps, remaining())
        if budget > 0:
            return sdk.move_to(
                pos, yaw=yaw, gripper=grip,
                max_steps=budget, tol=tol
            )
        return {"reached": False}

    def actuate(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            return sdk.set_gripper(action, steps=budget)
        return {}

    def align(height, steps, tol=0.003):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        offset = [
            state["eef_pos"][i] - nut["pos"][i]
            for i in range(3)
        ]
        turn = -square_error(nut["yaw"])
        target = [
            goal[0] + cos(turn) * offset[0] - sin(turn) * offset[1],
            goal[1] + sin(turn) * offset[0] + cos(turn) * offset[1],
            height + offset[2]
        ]
        return move(
            target, wrap(state["eef_yaw"] + turn),
            "close", steps, tol
        )

    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    resting_z = state["objects"]["nut"]["pos"][2]
    held = False

    for attempt in range(2):
        if remaining() < 210:
            break

        actuate("open", 8)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        offset = 0.052 if attempt == 0 else 0.045
        grasp = [
            nut["pos"][0] + offset * cos(yaw),
            nut["pos"][1] + offset * sin(yaw),
            nut["pos"][2]
        ]
        safe_z = max(state["eef_pos"][2], grasp[2] + 0.085)
        if state["eef_pos"][2] < grasp[2] + 0.065:
            move(
                [state["eef_pos"][0], state["eef_pos"][1], safe_z],
                state["eef_yaw"], "open", 16, 0.008
            )

        move([grasp[0], grasp[1], safe_z], yaw, "open", 32, 0.006)
        move(grasp, yaw, "open", 42, 0.003)

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        grasp = [
            nut["pos"][0] + offset * cos(yaw),
            nut["pos"][1] + offset * sin(yaw),
            nut["pos"][2]
        ]
        error = sqrt(sum(
            (state["eef_pos"][i] - grasp[i]) ** 2
            for i in range(3)
        ))
        if error > 0.005:
            move(grasp, yaw, "open", 16, 0.003)

        state = sdk.get_state()
        if sqrt(sum(
            (state["eef_pos"][i] - grasp[i]) ** 2
            for i in range(3)
        )) > 0.011:
            continue

        actuate("close", 14)
        state = sdk.get_state()
        before_z = state["objects"]["nut"]["pos"][2]
        eef = list(state["eef_pos"])
        clearance_z = max(resting_z + 0.105, state["goal_pos"][2] + 0.075)
        lift = max(0.075, clearance_z - before_z)
        move(
            [eef[0], eef[1], eef[2] + lift],
            state["eef_yaw"], "close", 34, 0.006
        )
        state = sdk.get_state()
        held = (
            state["objects"]["nut"]["pos"][2] - before_z > 0.04
            and state["gripper_width"] > 0.012
        )
        if held:
            break

    if not held:
        actuate("open", 6)
        return {
            "status": "failed",
            "reason": "Lifted grasp could not be verified",
            "steps_used": sdk.get_state()["steps_used"]
        }

    state = sdk.get_state()
    clearance_z = max(
        resting_z + 0.105,
        state["goal_pos"][2] + 0.075,
        state["objects"]["nut"]["pos"][2]
    )
    align(clearance_z, min(48, max(0, remaining() - 115)), 0.004)

    for correction in range(2):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        if (
            hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.004
            and abs(square_error(nut["yaw"])) < 0.04
        ):
            break
        if remaining() < 105:
            break
        align(clearance_z, 12, 0.003)

    stalls = 0
    for stage in range(9):
        if remaining() <= 30:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        pos = list(nut["pos"])
        lateral = hypot(pos[0] - goal[0], pos[1] - goal[1])
        angular = abs(square_error(nut["yaw"]))

        if (
            lateral < 0.006
            and abs(pos[2] - resting_z) < 0.009
            and angular < 0.08
        ):
            break

        if lateral > 0.006 or angular > 0.07:
            align(pos[2] + 0.003, min(14, remaining() - 30), 0.002)
            continue

        target_z = max(resting_z + 0.001, pos[2] - 0.045)
        align(target_z, min(24, remaining() - 30), 0.003)

        updated = sdk.get_state()
        new_nut = updated["objects"]["nut"]
        progress = pos[2] - new_nut["pos"][2]
        if progress < 0.004 and new_nut["pos"][2] > resting_z + 0.012:
            stalls += 1
            if stalls >= 2 and remaining() > 65:
                align(new_nut["pos"][2] + 0.018, 14, 0.003)
                state = sdk.get_state()
                align(state["objects"]["nut"]["pos"][2], 12, 0.002)
                stalls = 0
        else:
            stalls = 0

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    seated = (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.007
        and abs(nut["pos"][2] - resting_z) < 0.010
        and abs(square_error(nut["yaw"])) < 0.08
    )
    if not seated:
        return {
            "status": "failed",
            "reason": "Insertion not verified; grasp retained",
            "steps_used": state["steps_used"]
        }

    actuate("open", 10)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.07],
        state["eef_yaw"], "open", 18, 0.008
    )
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    seated = (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.007
        and abs(nut["pos"][2] - resting_z) < 0.010
        and abs(square_error(nut["yaw"])) < 0.08
    )
    return {
        "status": "done" if seated else "failed",
        "reason": "Released placement verified" if seated else "Placement shifted",
        "steps_used": state["steps_used"]
    }
