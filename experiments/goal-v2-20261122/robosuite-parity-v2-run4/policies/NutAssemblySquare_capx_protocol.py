def run_policy(sdk):
    def wrap(angle):
        return atan2(sin(angle), cos(angle))

    def square_error(angle):
        return atan2(sin(4.0 * angle), cos(4.0 * angle)) / 4.0

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

    def center_move(target, steps, align=True):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        eef = state["eef_pos"]
        yaw = state["eef_yaw"]
        correction = -square_error(nut["yaw"]) if align else 0.0
        offset_x = eef[0] - nut["pos"][0]
        offset_y = eef[1] - nut["pos"][1]
        target_eef = [
            target[0] + cos(correction) * offset_x - sin(correction) * offset_y,
            target[1] + sin(correction) * offset_x + cos(correction) * offset_y,
            target[2] + eef[2] - nut["pos"][2]
        ]
        return move(target_eef, wrap(yaw + correction), "close", steps)

    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or goal pose"}

    resting_z = state["objects"]["nut"]["pos"][2]
    held = False

    for handle_offset in [0.055, 0.045]:
        if remaining() < 190:
            break

        grip("open", 12)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        pos = nut["pos"]
        grasp_x = pos[0] + handle_offset * cos(yaw)
        grasp_y = pos[1] + handle_offset * sin(yaw)
        clearance = max(pos[2] + 0.085, state["eef_pos"][2])

        if state["eef_pos"][2] < pos[2] + 0.06:
            move(
                [state["eef_pos"][0], state["eef_pos"][1], clearance],
                state["eef_yaw"], "open", 20, 0.006
            )

        move([grasp_x, grasp_y, clearance], yaw, "open", 35, 0.005)

        # Complete the descent before closing; a short descent can stop
        # above the handle and produce a misleading gripper-width hint.
        move([grasp_x, grasp_y, pos[2]], yaw, "open", 48, 0.002)

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        target = [
            nut["pos"][0] + handle_offset * cos(yaw),
            nut["pos"][1] + handle_offset * sin(yaw),
            nut["pos"][2]
        ]
        error = sqrt(sum(
            (state["eef_pos"][i] - target[i]) ** 2 for i in range(3)
        ))
        if error > 0.005:
            move(target, yaw, "open", 25, 0.002)

        state = sdk.get_state()
        if state["eef_pos"][2] > state["objects"]["nut"]["pos"][2] + 0.009:
            continue

        grip("close", 18)
        state = sdk.get_state()
        before_z = state["objects"]["nut"]["pos"][2]
        eef = state["eef_pos"]
        move(
            [eef[0], eef[1], eef[2] + 0.09],
            state["eef_yaw"], "close", 32, 0.005
        )
        state = sdk.get_state()
        held = (
            state["objects"]["nut"]["pos"][2] > before_z + 0.045
            and state["gripper_width"] > 0.012
        )
        if held:
            break
        grip("open", 12)

    if not held:
        return {
            "status": "failed",
            "reason": "Nut did not follow the verified lift",
            "steps_used": sdk.get_state()["steps_used"]
        }

    state = sdk.get_state()
    goal = state["goal_pos"]
    hover_z = max(resting_z + 0.13, goal[2] + 0.11)
    nut_pos = state["objects"]["nut"]["pos"]
    center_move([nut_pos[0], nut_pos[1], hover_z], 25)
    center_move([goal[0], goal[1], hover_z], 45)

    for attempt in range(3):
        if remaining() < 55:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        lateral = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
        if lateral < 0.0035 and abs(square_error(nut["yaw"])) < 0.04:
            break
        center_move([goal[0], goal[1], hover_z], 14)

    seated = False
    stalls = 0
    for attempt in range(15):
        if remaining() <= 28:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        pos = nut["pos"]
        goal = state["goal_pos"]
        lateral = hypot(pos[0] - goal[0], pos[1] - goal[1])

        if lateral < 0.008 and pos[2] <= resting_z + 0.008:
            seated = True
            break

        if state["gripper_width"] < 0.010:
            break

        target_z = max(resting_z + 0.001, pos[2] - 0.025)
        if lateral > 0.005 or abs(square_error(nut["yaw"])) > 0.06:
            target_z = pos[2]

        center_move(
            [goal[0], goal[1], target_z],
            min(12, remaining() - 28)
        )

        updated = sdk.get_state()
        new_pos = updated["objects"]["nut"]["pos"]
        if target_z < pos[2] - 0.005 and new_pos[2] > pos[2] - 0.002:
            stalls += 1
        else:
            stalls = 0

        if stalls >= 2 and remaining() > 65:
            center_move([new_pos[0], new_pos[1], new_pos[2] + 0.018], 10)
            updated = sdk.get_state()
            goal = updated["goal_pos"]
            center_move(
                [goal[0], goal[1], updated["objects"]["nut"]["pos"][2]],
                12
            )
            stalls = 0

    state = sdk.get_state()
    pos = state["objects"]["nut"]["pos"]
    goal = state["goal_pos"]
    seated = (
        hypot(pos[0] - goal[0], pos[1] - goal[1]) < 0.008
        and pos[2] <= resting_z + 0.010
    )
    if not seated:
        return {
            "status": "failed",
            "reason": "Could not verify seating",
            "steps_used": state["steps_used"]
        }

    grip("open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.08],
        state["eef_yaw"], "open", 16, 0.008
    )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    lateral = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    success = (
        lateral < 0.010
        and abs(nut["pos"][2] - resting_z) < 0.015
        and abs(square_error(nut["yaw"])) < 0.15
    )
    return {
        "status": "done" if success else "failed",
        "reason": "Released and checked seating" if success else "Seating check failed",
        "lateral_error_m": lateral,
        "steps_used": state["steps_used"]
    }
