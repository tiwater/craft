def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def wrap(angle):
        return atan2(sin(angle), cos(angle))

    def square_error(angle):
        return atan2(sin(4.0 * angle), cos(4.0 * angle)) / 4.0

    def move(pos, yaw, grip, steps, tolerance=0.003):
        budget = min(steps, remaining())
        if budget > 0:
            return sdk.move_to(
                pos, yaw=yaw, gripper=grip,
                max_steps=budget, tol=tolerance
            )
        return {"reached": False}

    def actuate(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def align(target, steps):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        offset = [
            state["eef_pos"][i] - nut["pos"][i]
            for i in range(3)
        ]
        turn = -square_error(nut["yaw"])
        rotated = [
            cos(turn) * offset[0] - sin(turn) * offset[1],
            sin(turn) * offset[0] + cos(turn) * offset[1],
            offset[2]
        ]
        return move(
            [target[i] + rotated[i] for i in range(3)],
            wrap(state["eef_yaw"] + turn),
            "close", steps, 0.002
        )

    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    resting_z = state["objects"]["nut"]["pos"][2]
    held = False

    # Try the transverse handle grasp first, then the orthogonal grasp.
    for attempt in range(3):
        if remaining() < 145:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        eef = list(state["eef_pos"])
        safe_z = max(eef[2], nut["pos"][2] + 0.07)
        move([eef[0], eef[1], safe_z], state["eef_yaw"], None, 16)
        actuate("open", 8)

        angle_offset = pi / 2.0 if attempt != 1 else 0.0
        handle_offset = 0.055 if attempt < 2 else 0.045
        height_offset = 0.0 if attempt < 2 else -0.004

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        grasp_yaw = wrap(yaw + angle_offset)
        grasp = [
            nut["pos"][0] + handle_offset * cos(yaw),
            nut["pos"][1] + handle_offset * sin(yaw),
            nut["pos"][2] + height_offset
        ]
        move([grasp[0], grasp[1], safe_z], grasp_yaw, "open", 26)

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        grasp_yaw = wrap(yaw + angle_offset)
        grasp = [
            nut["pos"][0] + handle_offset * cos(yaw),
            nut["pos"][1] + handle_offset * sin(yaw),
            nut["pos"][2] + height_offset
        ]
        move(
            [grasp[0], grasp[1], grasp[2] + 0.018],
            grasp_yaw, "open", 14
        )
        move(grasp, grasp_yaw, "open", 22, 0.002)

        state = sdk.get_state()
        if (
            hypot(state["eef_pos"][0] - grasp[0],
                  state["eef_pos"][1] - grasp[1]) > 0.009
            or abs(state["eef_pos"][2] - grasp[2]) > 0.012
        ):
            continue

        actuate("close", 14)
        state = sdk.get_state()
        before = list(state["objects"]["nut"]["pos"])
        before_eef = list(state["eef_pos"])
        lift_yaw = state["eef_yaw"]

        move(
            [before_eef[0], before_eef[1], before_eef[2] + 0.045],
            lift_yaw, "close", 22
        )
        state = sdk.get_state()
        after = list(state["objects"]["nut"]["pos"])
        rise = after[2] - before[2]
        eef_rise = state["eef_pos"][2] - before_eef[2]

        if rise > 0.022 and abs(rise - eef_rise) < 0.018:
            offset = [
                state["eef_pos"][i] - after[i] for i in range(3)
            ]
            eef = list(state["eef_pos"])
            move(
                [eef[0], eef[1], eef[2] + 0.03],
                state["eef_yaw"], "close", 16
            )
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            drift = sqrt(sum(
                (state["eef_pos"][i] - nut["pos"][i] - offset[i]) ** 2
                for i in range(3)
            ))
            held = nut["pos"][2] > resting_z + 0.04 and drift < 0.015
            if held:
                break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        eef = list(state["eef_pos"])
        if nut["pos"][2] > resting_z + 0.012:
            move(
                [eef[0], eef[1],
                 eef[2] - (nut["pos"][2] - resting_z)],
                state["eef_yaw"], "close", 14
            )
        actuate("open", 6)

    if not held:
        return {
            "status": "failed",
            "reason": "No verified lifted handle grasp",
            "steps_used": sdk.get_state()["steps_used"]
        }

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    clearance_z = max(
        resting_z + 0.11, goal[2] + 0.085, nut["pos"][2]
    )
    align([nut["pos"][0], nut["pos"][1], clearance_z], 20)
    state = sdk.get_state()
    goal = state["goal_pos"]
    align([goal[0], goal[1], clearance_z], 32)

    stalls = 0
    corrections = 0
    for stage in range(28):
        if remaining() <= 24:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        pos = list(nut["pos"])
        lateral = hypot(pos[0] - goal[0], pos[1] - goal[1])
        yaw_error = abs(square_error(nut["yaw"]))

        if (
            lateral < 0.006
            and abs(pos[2] - resting_z) < 0.009
            and yaw_error < 0.08
        ):
            break

        if lateral > 0.004 or yaw_error > 0.045:
            corrections += 1
            target_z = pos[2]
            if corrections >= 3 and pos[2] < clearance_z - 0.02:
                target_z = min(clearance_z, pos[2] + 0.025)
                corrections = 0
            align(
                [goal[0], goal[1], target_z],
                min(14, remaining() - 24)
            )
            continue

        corrections = 0
        target_z = max(resting_z + 0.001, pos[2] - 0.015)
        align(
            [goal[0], goal[1], target_z],
            min(14, remaining() - 24)
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        if (
            pos[2] - nut["pos"][2] < 0.002
            and nut["pos"][2] > resting_z + 0.012
        ):
            stalls += 1
            if stalls >= 3 or remaining() < 50:
                break
            goal = state["goal_pos"]
            align(
                [goal[0], goal[1], nut["pos"][2] + 0.022],
                min(14, remaining() - 24)
            )
        else:
            stalls = 0

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    seated = (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.008
        and abs(nut["pos"][2] - resting_z) < 0.012
        and abs(square_error(nut["yaw"])) < 0.10
    )
    if not seated:
        return {
            "status": "failed",
            "reason": "Insertion not verified; grasp retained",
            "steps_used": state["steps_used"]
        }

    actuate("open", 10)
    state = sdk.get_state()
    eef = list(state["eef_pos"])
    move(
        [eef[0], eef[1], eef[2] + 0.08],
        state["eef_yaw"], "open", 14
    )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    seated = (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.008
        and abs(nut["pos"][2] - resting_z) < 0.012
        and abs(square_error(nut["yaw"])) < 0.10
    )
    return {
        "status": "done" if seated else "failed",
        "reason": "Released placement verified" if seated else "Placement shifted",
        "steps_used": state["steps_used"]
    }
