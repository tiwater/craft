def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def wrap(angle):
        return atan2(sin(angle), cos(angle))

    def square_error(angle):
        return atan2(sin(4.0 * angle), cos(4.0 * angle)) / 4.0

    def move(pos, yaw, grip, steps, tolerance=0.004, reserve=0):
        budget = min(steps, max(0, remaining() - reserve))
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

    def align_nut(target, steps, reserve=28):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        eef = state["eef_pos"]
        offset = [eef[i] - nut["pos"][i] for i in range(3)]
        turn = -square_error(nut["yaw"])
        rotated = [
            cos(turn) * offset[0] - sin(turn) * offset[1],
            sin(turn) * offset[0] + cos(turn) * offset[1],
            offset[2]
        ]
        return move(
            [target[i] + rotated[i] for i in range(3)],
            wrap(state["eef_yaw"] + turn),
            "close", steps, 0.003, reserve
        )

    def insertion_jammed(target_z):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        quat = nut["quat"]
        upright = 1.0 - 2.0 * (quat[0] * quat[0] + quat[1] * quat[1])
        return (
            nut["pos"][2] > target_z + 0.035
            and upright < cos(0.35)
            and state["gripper_width"] > 0.01
            and remaining() >= 100
        )

    def recover_insertion():
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        eef = state["eef_pos"]
        lift = max(0.055, clearance_z - nut["pos"][2])
        move(
            [eef[0], eef[1], eef[2] + lift],
            state["eef_yaw"], "close", 24, 0.005, 76
        )

        state = sdk.get_state()
        if state["objects"]["nut"]["pos"][2] < state["goal_pos"][2] + 0.13:
            return

        for tick in range(12):
            if remaining() <= 76:
                break
            state = sdk.get_state()
            quat = state["objects"]["nut"]["quat"]
            axis_x = 2.0 * (quat[0] * quat[2] + quat[3] * quat[1])
            axis_y = 2.0 * (quat[1] * quat[2] - quat[3] * quat[0])
            axis_z = 1.0 - 2.0 * (
                quat[0] * quat[0] + quat[1] * quat[1]
            )
            if axis_z > cos(0.06):
                break
            correction_x = atan2(axis_y, axis_z)
            correction_y = -atan2(axis_x, axis_z)
            sdk.step_raw(
                [
                    0.0, 0.0, 0.0,
                    max(-0.5, min(0.5, 2.0 * correction_x)),
                    max(-0.5, min(0.5, 2.0 * correction_y)),
                    0.0, 1.0
                ],
                steps=1
            )

        state = sdk.get_state()
        goal = state["goal_pos"]
        align_nut([goal[0], goal[1], clearance_z], 16, 60)

    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    resting_z = state["objects"]["nut"]["pos"][2]
    held = False

    for handle_offset in [0.055, 0.055, -0.055]:
        if remaining() < 155:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        pos = list(nut["pos"])
        yaw = nut["yaw"]
        grasp_yaw = wrap(yaw + pi / 2.0)
        alternative_yaw = wrap(grasp_yaw + pi)
        if abs(wrap(alternative_yaw - state["eef_yaw"])) < abs(
            wrap(grasp_yaw - state["eef_yaw"])
        ):
            grasp_yaw = alternative_yaw

        grasp = [
            pos[0] + handle_offset * cos(yaw),
            pos[1] + handle_offset * sin(yaw),
            pos[2]
        ]
        safe_z = max(state["eef_pos"][2], pos[2] + 0.12)
        move(
            [state["eef_pos"][0], state["eef_pos"][1], safe_z],
            state["eef_yaw"], "open", 14, 0.008
        )
        move([grasp[0], grasp[1], safe_z], grasp_yaw, "open", 28)

        move(grasp, grasp_yaw, "open", 44, 0.003)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        grasp = [
            nut["pos"][0] + handle_offset * cos(nut["yaw"]),
            nut["pos"][1] + handle_offset * sin(nut["yaw"]),
            nut["pos"][2]
        ]
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - grasp[0], eef[1] - grasp[1]) > 0.005
            or abs(eef[2] - grasp[2]) > 0.005
        ):
            grasp_yaw = wrap(
                grasp_yaw + atan2(
                    sin(2.0 * (nut["yaw"] + pi / 2.0 - grasp_yaw)),
                    cos(2.0 * (nut["yaw"] + pi / 2.0 - grasp_yaw))
                ) / 2.0
            )
            move(grasp, grasp_yaw, "open", 28, 0.003)

        before_z = sdk.get_state()["objects"]["nut"]["pos"][2]
        actuate("close", 20)
        state = sdk.get_state()
        eef = state["eef_pos"]
        nut_z = state["objects"]["nut"]["pos"][2]
        move(
            [eef[0], eef[1], eef[2] + max(0.09, 1.065 - nut_z)],
            state["eef_yaw"], "close", 40, 0.005
        )

        state = sdk.get_state()
        held = (
            state["objects"]["nut"]["pos"][2] - before_z >= 0.05
            and state["gripper_width"] > 0.01
        )
        if held:
            break
        actuate("open", 10)

    if not held:
        return {
            "status": "failed",
            "reason": "Could not verify a lifted grasp",
            "steps_used": sdk.get_state()["steps_used"]
        }

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    clearance_z = max(1.055, state["goal_pos"][2] + 0.12, nut["pos"][2])
    if nut["pos"][2] < clearance_z - 0.005:
        align_nut([nut["pos"][0], nut["pos"][1], clearance_z], 20, 120)

    state = sdk.get_state()
    goal = state["goal_pos"]
    align_nut([goal[0], goal[1], clearance_z], 40, 110)

    for attempt in range(2):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        if (
            hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.003
            and abs(square_error(nut["yaw"])) < 0.035
        ):
            break
        if remaining() <= 122:
            break
        align_nut([goal[0], goal[1], clearance_z], 12, 110)

    state = sdk.get_state()
    start_z = state["objects"]["nut"]["pos"][2]
    final_z = state["goal_pos"][2] + 0.015
    descent_budget = max(0, remaining() - 76)
    stages = min(5, max(1, int(descent_budget / 12)))
    stage_steps = min(12, max(1, int(descent_budget / stages)))
    recovered = False

    for stage in range(stages):
        state = sdk.get_state()
        goal = state["goal_pos"]
        fraction = float(stage + 1) / float(stages)
        target_z = start_z + fraction * (final_z - start_z)
        align_nut([goal[0], goal[1], target_z], stage_steps, 76)
        if insertion_jammed(target_z):
            recover_insertion()
            recovered = True
            break

    for attempt in range(2):
        state = sdk.get_state()
        goal = state["goal_pos"]
        insertion_steps = max(1, int(max(0, remaining() - 28) / (2 - attempt)))
        align_nut(
            [goal[0], goal[1], goal[2] + 0.015],
            insertion_steps, 28
        )
        if not recovered and insertion_jammed(goal[2] + 0.015):
            recover_insertion()
            recovered = True

    actuate("open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.08],
        state["eef_yaw"], "open", 16, 0.008
    )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    seated = (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.008
        and resting_z - 0.01 <= nut["pos"][2] <= goal[2] + 0.025
        and abs(square_error(nut["yaw"])) < 0.10
    )
    return {
        "status": "done" if seated else "failed",
        "reason": "Released placement verified" if seated else "Placement not verified",
        "steps_used": state["steps_used"]
    }
