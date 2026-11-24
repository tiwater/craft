def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def wrap(angle):
        return atan2(sin(angle), cos(angle))

    def square_error(yaw):
        return atan2(sin(4.0 * yaw), cos(4.0 * yaw)) / 4.0

    def move(pos, yaw, grip, steps, tolerance=0.004):
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

    def align_nut(target, steps, correct_yaw=True):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        eef = state["eef_pos"]
        offset = [eef[i] - nut["pos"][i] for i in range(3)]
        turn = -square_error(nut["yaw"]) if correct_yaw else 0.0
        rotated = [
            cos(turn) * offset[0] - sin(turn) * offset[1],
            sin(turn) * offset[0] + cos(turn) * offset[1],
            offset[2]
        ]
        return move(
            [target[i] + rotated[i] for i in range(3)],
            wrap(state["eef_yaw"] + turn),
            "close", steps
        )

    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    resting_z = state["objects"]["nut"]["pos"][2]
    held = False

    # The square nut's handle extends along its local positive x axis.
    for handle_offset in [0.055, 0.045]:
        if remaining() < 170:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        pos = nut["pos"]
        yaw = nut["yaw"]
        grasp = [
            pos[0] + handle_offset * cos(yaw),
            pos[1] + handle_offset * sin(yaw),
            pos[2]
        ]
        safe_z = max(state["eef_pos"][2], pos[2] + 0.12)

        move(
            [state["eef_pos"][0], state["eef_pos"][1], safe_z],
            state["eef_yaw"], "open", 18
        )
        move([grasp[0], grasp[1], safe_z], yaw, "open", 30)
        move(grasp, yaw, "open", 24, 0.003)
        actuate("close", 12)

        state = sdk.get_state()
        before = list(state["objects"]["nut"]["pos"])
        eef = state["eef_pos"]
        move(
            [eef[0], eef[1], eef[2] + 0.065],
            state["eef_yaw"], "close", 24
        )
        state = sdk.get_state()
        after = state["objects"]["nut"]["pos"]
        held = (
            after[2] - before[2] > 0.035
            and 0.012 < state["gripper_width"] < 0.075
        )
        if held:
            break
        actuate("open", 8)

    if not held:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    goal = state["goal_pos"]
    nut_pos = state["objects"]["nut"]["pos"]
    clearance_z = max(resting_z + 0.13, goal[2] + 0.09, nut_pos[2])

    align_nut([nut_pos[0], nut_pos[1], clearance_z], 25)
    state = sdk.get_state()
    goal = state["goal_pos"]
    align_nut([goal[0], goal[1], clearance_z], 38)

    # Recompute the measured grasp offset after every motion.
    for attempt in range(3):
        if remaining() < 45:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        lateral_error = hypot(
            nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]
        )
        if lateral_error < 0.003 and abs(square_error(nut["yaw"])) < 0.035:
            break
        align_nut([goal[0], goal[1], clearance_z], 16)

    stalls = 0
    for stage in range(15):
        if remaining() < 30:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        pos = nut["pos"]
        lateral_error = hypot(pos[0] - goal[0], pos[1] - goal[1])
        yaw_error = abs(square_error(nut["yaw"]))

        if (
            lateral_error < 0.007
            and abs(pos[2] - resting_z) < 0.008
            and yaw_error < 0.08
        ):
            break

        if lateral_error > 0.005 or yaw_error > 0.06:
            align_nut([goal[0], goal[1], pos[2] + 0.008], 12)
            continue

        target_z = max(resting_z + 0.001, pos[2] - 0.015)
        align_nut([goal[0], goal[1], target_z], 12)
        updated = sdk.get_state()
        new_z = updated["objects"]["nut"]["pos"][2]

        if pos[2] - new_z < 0.002 and pos[2] > resting_z + 0.012:
            stalls += 1
            if stalls > 2:
                break
            # Withdraw before correcting alignment at a contacted peg rim.
            goal = updated["goal_pos"]
            align_nut([goal[0], goal[1], new_z + 0.018], 12)
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
            "reason": "Insertion could not be verified; grasp retained",
            "steps_used": state["steps_used"]
        }

    actuate("open", 10)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.08],
        state["eef_yaw"], "open", 18
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
