def run_policy(sdk):
    def wrap(angle):
        return atan2(sin(angle), cos(angle))

    def square_error(yaw):
        return atan2(sin(4.0 * yaw), cos(4.0 * yaw)) / 4.0

    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, action, steps, tol=0.003):
        budget = min(steps, remaining())
        if budget > 0:
            return sdk.move_to(
                pos, yaw=yaw, gripper=action, max_steps=budget, tol=tol
            )
        return {"reached": False}

    def grip(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def center_move(target, yaw, steps, tol=0.003):
        state = sdk.get_state()
        pos = state["objects"]["nut"]["pos"]
        eef = state["eef_pos"]
        turn = wrap(yaw - state["eef_yaw"])
        offset_x = eef[0] - pos[0]
        offset_y = eef[1] - pos[1]
        return move(
            [
                target[0] + cos(turn) * offset_x - sin(turn) * offset_y,
                target[1] + sin(turn) * offset_x + cos(turn) * offset_y,
                target[2] + eef[2] - pos[2]
            ],
            yaw, "close", steps, tol
        )

    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    resting_z = state["objects"]["nut"]["pos"][2]
    held = False

    for handle_offset in [0.055, 0.045, -0.055]:
        if remaining() < 190:
            break

        grip("open", 8)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        pos = nut["pos"]
        grasp_yaw = wrap(nut["yaw"] + pi / 2.0)
        clearance = max(state["eef_pos"][2], pos[2] + 0.10)

        move(
            [state["eef_pos"][0], state["eef_pos"][1], clearance],
            state["eef_yaw"], "open", 14, 0.006
        )
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        pos = nut["pos"]
        grasp = [
            pos[0] + handle_offset * cos(nut["yaw"]),
            pos[1] + handle_offset * sin(nut["yaw"]),
            pos[2]
        ]
        move(
            [grasp[0], grasp[1], clearance],
            grasp_yaw, "open", 30, 0.004
        )

        # The traces stopped roughly 0.019 m above the commanded grasp.
        # Allow the descent to settle before closing the fingers.
        move(grasp, grasp_yaw, "open", 38, 0.002)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        grasp = [
            nut["pos"][0] + handle_offset * cos(nut["yaw"]),
            nut["pos"][1] + handle_offset * sin(nut["yaw"]),
            nut["pos"][2]
        ]
        grasp_yaw = wrap(nut["yaw"] + pi / 2.0)
        if (
            hypot(
                state["eef_pos"][0] - grasp[0],
                state["eef_pos"][1] - grasp[1]
            ) > 0.004
            or abs(state["eef_pos"][2] - grasp[2]) > 0.004
        ):
            move(grasp, grasp_yaw, "open", 14, 0.002)

        grip("close", 20)
        before = sdk.get_state()
        before_nut = list(before["objects"]["nut"]["pos"])
        before_eef = list(before["eef_pos"])
        move(
            [before_eef[0], before_eef[1], before_eef[2] + 0.085],
            before["eef_yaw"], "close", 30, 0.004
        )
        after = sdk.get_state()
        nut_rise = after["objects"]["nut"]["pos"][2] - before_nut[2]
        eef_rise = after["eef_pos"][2] - before_eef[2]
        held = (
            nut_rise >= 0.05
            and abs(nut_rise - eef_rise) < 0.025
            and after["gripper_width"] > 0.010
        )
        if held:
            break

        if nut_rise > 0.012:
            center_move(
                [
                    after["objects"]["nut"]["pos"][0],
                    after["objects"]["nut"]["pos"][1],
                    resting_z + 0.005
                ],
                after["eef_yaw"], 14
            )
        grip("open", 6)

    if not held:
        return {"status": "failed", "reason": "Could not verify a stable grasp"}

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    hover_z = max(1.05, goal[2] + 0.12, resting_z + 0.16)
    center_move(
        [nut["pos"][0], nut["pos"][1], hover_z],
        state["eef_yaw"], 32, 0.004
    )

    for attempt in range(2):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        error = square_error(nut["yaw"])
        if abs(error) < 0.035:
            break
        center_move(
            [nut["pos"][0], nut["pos"][1], hover_z],
            wrap(state["eef_yaw"] - error), 16, 0.003
        )

    for attempt in range(3):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        center_move(
            [goal[0], goal[1], hover_z],
            wrap(state["eef_yaw"] - square_error(nut["yaw"])),
            min(32 if attempt == 0 else 12, max(0, remaining() - 65)),
            0.002
        )
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        if (
            hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.004
            and abs(square_error(nut["yaw"])) < 0.045
        ):
            break
        if remaining() <= 65:
            break

    # Continue toward the full insertion height while correcting the live
    # hole position. Reserve time for a final descent and release.
    for attempt in range(12):
        if remaining() <= 44:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        pos = nut["pos"]
        goal = state["goal_pos"]
        final_z = goal[2] + 0.015
        lateral = hypot(pos[0] - goal[0], pos[1] - goal[1])
        error = square_error(nut["yaw"])
        if pos[2] <= final_z + 0.003 and lateral < 0.006:
            break

        target_z = max(final_z, pos[2] - 0.025)
        center_move(
            [goal[0], goal[1], target_z],
            wrap(state["eef_yaw"] - error),
            min(9, remaining() - 44), 0.002
        )

    # Never return early merely because an intermediate move timed out.
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    center_move(
        [goal[0], goal[1], goal[2] + 0.015],
        wrap(state["eef_yaw"] - square_error(nut["yaw"])),
        min(24, max(0, remaining() - 20)), 0.002
    )

    grip("open", 10)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.10],
        state["eef_yaw"], "open", min(18, remaining()), 0.006
    )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    lateral = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    success = (
        lateral < 0.012
        and nut["pos"][2] <= min(resting_z + 0.015, goal[2] + 0.015)
        and abs(square_error(nut["yaw"])) < 0.15
    )
    return {
        "status": "done" if success else "failed",
        "reason": (
            "Released and checked seating"
            if success else "Final seating check failed"
        ),
        "lateral_error_m": lateral,
        "steps_used": state["steps_used"]
    }
