def run_policy(sdk):
    def wrap(angle):
        return atan2(sin(angle), cos(angle))

    def square_error(yaw):
        return atan2(sin(4.0 * yaw), cos(4.0 * yaw)) / 4.0

    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, grip, steps, tol=0.004):
        budget = min(steps, remaining())
        if budget > 0:
            return sdk.move_to(
                pos, yaw=yaw, gripper=grip, max_steps=budget, tol=tol
            )
        return {"reached": False}

    def grip(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def center_move(target, yaw, steps, tol=0.004):
        state = sdk.get_state()
        nut = state["objects"]["nut"]["pos"]
        eef = state["eef_pos"]
        return move(
            [eef[i] + target[i] - nut[i] for i in range(3)],
            yaw, "close", steps, tol
        )

    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    resting_z = state["objects"]["nut"]["pos"][2]
    held = False

    # The square nut's grasping handle extends along its local positive X axis.
    for handle_offset in [0.055, 0.045, 0.065]:
        if remaining() < 145:
            break

        grip("open", 6)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        pos = nut["pos"]
        yaw = nut["yaw"]
        grasp = [
            pos[0] + handle_offset * cos(yaw),
            pos[1] + handle_offset * sin(yaw),
            pos[2]
        ]
        clearance = max(state["eef_pos"][2], pos[2] + 0.10)
        move(
            [state["eef_pos"][0], state["eef_pos"][1], clearance],
            state["eef_yaw"], "open", 15
        )
        move([grasp[0], grasp[1], clearance], yaw, "open", 26)
        move(grasp, yaw, "open", 22, 0.003)
        grip("close", 12)

        before = sdk.get_state()
        before_nut = list(before["objects"]["nut"]["pos"])
        before_eef = list(before["eef_pos"])
        move(
            [before_eef[0], before_eef[1], before_eef[2] + 0.075],
            before["eef_yaw"], "close", 24
        )
        after = sdk.get_state()
        nut_rise = after["objects"]["nut"]["pos"][2] - before_nut[2]
        eef_rise = after["eef_pos"][2] - before_eef[2]
        held = (
            nut_rise > 0.035
            and abs(nut_rise - eef_rise) < 0.025
            and after["gripper_width"] > 0.012
        )
        if held:
            break

        # Lower before opening if the attempted grasp only lifted partially.
        if nut_rise > 0.012:
            center_move(
                [after["objects"]["nut"]["pos"][0],
                 after["objects"]["nut"]["pos"][1], resting_z + 0.005],
                after["eef_yaw"], 15
            )
        grip("open", 6)

    if not held:
        return {"status": "failed", "reason": "Could not verify a stable grasp"}

    state = sdk.get_state()
    goal = state["goal_pos"]
    hover_z = max(resting_z + 0.16, goal[2] + 0.12)

    # Lift clear of the peg before changing orientation or translating.
    nut = state["objects"]["nut"]
    center_move(
        [nut["pos"][0], nut["pos"][1], hover_z],
        state["eef_yaw"], 24
    )

    # Square pegs admit four equivalent yaw alignments.
    # Re-observe after rotation because the handle grasp offsets the nut center.
    for attempt in range(3):
        state = sdk.get_state()
        error = square_error(state["objects"]["nut"]["yaw"])
        if abs(error) < 0.035:
            break
        move(
            list(state["eef_pos"]),
            wrap(state["eef_yaw"] - error),
            "close", 15, 0.003
        )

    for attempt in range(3):
        state = sdk.get_state()
        goal = state["goal_pos"]
        nut = state["objects"]["nut"]
        center_move(
            [goal[0], goal[1], hover_z],
            wrap(state["eef_yaw"] - square_error(nut["yaw"])),
            20, 0.002
        )
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        if (
            hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.004
            and abs(square_error(nut["yaw"])) < 0.045
        ):
            break

    stalls = 0
    seated = False
    for attempt in range(24):
        if remaining() < 30:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        pos = nut["pos"]
        goal = state["goal_pos"]
        lateral = hypot(pos[0] - goal[0], pos[1] - goal[1])
        error = square_error(nut["yaw"])

        if lateral < 0.009 and pos[2] <= resting_z + 0.008:
            seated = True
            break

        if state["gripper_width"] < 0.010:
            return {"status": "failed", "reason": "Grasp lost during insertion"}

        target_z = max(resting_z + 0.002, pos[2] - 0.012)
        if lateral > 0.005 or abs(error) > 0.055:
            target_z = pos[2]

        center_move(
            [goal[0], goal[1], target_z],
            wrap(state["eef_yaw"] - error),
            7, 0.002
        )
        updated = sdk.get_state()
        new_z = updated["objects"]["nut"]["pos"][2]
        if target_z < pos[2] - 0.004 and pos[2] - new_z < 0.002:
            stalls += 1
        else:
            stalls = 0

        if stalls >= 2:
            if remaining() < 65:
                break
            # Unload contact, then recenter before another insertion attempt.
            updated_nut = updated["objects"]["nut"]
            center_move(
                [updated_nut["pos"][0], updated_nut["pos"][1],
                 updated_nut["pos"][2] + 0.025],
                updated["eef_yaw"], 10, 0.003
            )
            updated = sdk.get_state()
            goal = updated["goal_pos"]
            center_move(
                [goal[0], goal[1], updated["objects"]["nut"]["pos"][2]],
                wrap(updated["eef_yaw"]
                     - square_error(updated["objects"]["nut"]["yaw"])),
                12, 0.002
            )
            stalls = 0

    if not seated:
        state = sdk.get_state()
        pos = state["objects"]["nut"]["pos"]
        goal = state["goal_pos"]
        seated = (
            hypot(pos[0] - goal[0], pos[1] - goal[1]) < 0.009
            and pos[2] <= resting_z + 0.008
        )

    if not seated:
        return {"status": "failed", "reason": "Insertion did not reach seating height"}

    grip("open", 10)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.10],
        state["eef_yaw"], "open", 18
    )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    lateral = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    success = (
        lateral < 0.012
        and nut["pos"][2] <= resting_z + 0.015
        and abs(square_error(nut["yaw"])) < 0.15
    )
    return {
        "status": "done" if success else "failed",
        "reason": "Released and checked seating" if success else "Final seating check failed",
        "lateral_error_m": lateral,
        "steps_used": state["steps_used"]
    }
