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
            return sdk.set_gripper(action, steps=budget)
        return {"gripper_width": 0.0, "grasped_hint": False}

    def center_move(target, yaw, steps, tol=0.002):
        state = sdk.get_state()
        pos = state["objects"]["nut"]["pos"]
        eef = state["eef_pos"]
        angle = wrap(yaw - state["eef_yaw"])
        dx = pos[0] - eef[0]
        dy = pos[1] - eef[1]
        return move(
            [
                target[0] - cos(angle) * dx + sin(angle) * dy,
                target[1] - sin(angle) * dx - cos(angle) * dy,
                eef[2] + target[2] - pos[2]
            ],
            yaw, "close", steps, tol
        )

    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    resting_z = state["objects"]["nut"]["pos"][2]
    held = False

    # Close across the handle's narrow dimension before trying a cross-grasp.
    settings = [
        [0.055, 0.0, 0.002],
        [0.060, 0.0, -0.004],
        [0.055, pi / 2.0, 0.002]
    ]

    for setting in settings:
        if remaining() < 125:
            break

        # Retract before opening so a failed low approach cannot sweep the nut.
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        clearance = max(state["eef_pos"][2], nut["pos"][2] + 0.070)
        move(
            [state["eef_pos"][0], state["eef_pos"][1], clearance],
            state["eef_yaw"], None, 16, 0.005
        )
        grip("open", 8)

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        desired_yaw = nut["yaw"] + setting[1]
        # The parallel jaw grasp is equivalent after a half-turn.
        yaw = wrap(
            state["eef_yaw"]
            + 0.5 * atan2(
                sin(2.0 * (desired_yaw - state["eef_yaw"])),
                cos(2.0 * (desired_yaw - state["eef_yaw"]))
            )
        )
        move(
            [
                nut["pos"][0] + setting[0] * cos(nut["yaw"]),
                nut["pos"][1] + setting[0] * sin(nut["yaw"]),
                clearance
            ],
            yaw, "open", 26, 0.003
        )

        # Recompute the handle position after each approach segment.
        for height in [0.025, setting[2]]:
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            desired_yaw = nut["yaw"] + setting[1]
            yaw = wrap(
                state["eef_yaw"]
                + 0.5 * atan2(
                    sin(2.0 * (desired_yaw - state["eef_yaw"])),
                    cos(2.0 * (desired_yaw - state["eef_yaw"]))
                )
            )
            target = [
                nut["pos"][0] + setting[0] * cos(nut["yaw"]),
                nut["pos"][1] + setting[0] * sin(nut["yaw"]),
                nut["pos"][2] + height
            ]
            move(target, yaw, "open", 18, 0.002)

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        target = [
            nut["pos"][0] + setting[0] * cos(nut["yaw"]),
            nut["pos"][1] + setting[0] * sin(nut["yaw"]),
            nut["pos"][2] + setting[2]
        ]
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - target[0], eef[1] - target[1]) > 0.006
            or abs(eef[2] - target[2]) > 0.006
            or abs(wrap(yaw - state["eef_yaw"])) > 0.12
        ):
            move(target, yaw, "open", 14, 0.002)

        grip("close", 14)
        before = sdk.get_state()
        before_pos = list(before["objects"]["nut"]["pos"])
        before_eef = list(before["eef_pos"])
        move(
            [before_eef[0], before_eef[1], before_eef[2] + 0.060],
            before["eef_yaw"], "close", 20, 0.003
        )
        after = sdk.get_state()
        nut_rise = after["objects"]["nut"]["pos"][2] - before_pos[2]
        eef_rise = after["eef_pos"][2] - before_eef[2]
        held = (
            nut_rise > 0.025
            and abs(nut_rise - eef_rise) < 0.020
            and after["gripper_width"] > 0.010
        )
        if held:
            break

        if nut_rise > 0.010:
            pos = after["objects"]["nut"]["pos"]
            center_move(
                [pos[0], pos[1], resting_z + 0.003],
                after["eef_yaw"], 12
            )
        grip("open", 6)

    if not held:
        return {
            "status": "failed",
            "reason": "Nut did not follow handle grasp",
            "steps_used": sdk.get_state()["steps_used"]
        }

    state = sdk.get_state()
    goal = state["goal_pos"]
    hover_z = max(resting_z + 0.110, goal[2] + 0.085)
    nut = state["objects"]["nut"]
    center_move(
        [nut["pos"][0], nut["pos"][1], hover_z],
        state["eef_yaw"], 18
    )

    for attempt in range(4):
        if remaining() < 42:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        error = square_error(nut["yaw"])
        lateral = hypot(
            nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]
        )
        if lateral < 0.003 and abs(error) < 0.045:
            break
        center_move(
            [goal[0], goal[1], hover_z],
            wrap(state["eef_yaw"] - error),
            22 if attempt == 0 else 10, 0.0015
        )

    stalls = 0
    for attempt in range(28):
        if remaining() <= 22:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        pos = list(nut["pos"])
        goal = state["goal_pos"]
        lateral = hypot(pos[0] - goal[0], pos[1] - goal[1])
        error = square_error(nut["yaw"])

        if lateral < 0.006 and pos[2] <= resting_z + 0.008:
            break

        aligned = lateral < 0.004 and abs(error) < 0.055
        target_z = pos[2]
        if aligned:
            target_z = max(resting_z + 0.001, pos[2] - 0.020)

        center_move(
            [goal[0], goal[1], target_z],
            wrap(state["eef_yaw"] - error),
            min(8, remaining() - 22), 0.0015
        )
        updated = sdk.get_state()
        new_pos = updated["objects"]["nut"]["pos"]
        if aligned and pos[2] - new_pos[2] < 0.002:
            stalls += 1
        else:
            stalls = 0

        if stalls >= 2 and remaining() > 48:
            center_move(
                [new_pos[0], new_pos[1], new_pos[2] + 0.018],
                updated["eef_yaw"], 8
            )
            updated = sdk.get_state()
            nut = updated["objects"]["nut"]
            goal = updated["goal_pos"]
            center_move(
                [goal[0], goal[1], nut["pos"][2]],
                wrap(updated["eef_yaw"] - square_error(nut["yaw"])),
                10, 0.001
            )
            stalls = 0

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    seated = (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.006
        and abs(nut["pos"][2] - resting_z) < 0.010
        and abs(square_error(nut["yaw"])) < 0.12
    )
    if not seated:
        return {
            "status": "failed",
            "reason": "Insertion did not reach seating height",
            "steps_used": state["steps_used"]
        }

    grip("open", 9)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.080],
        state["eef_yaw"], "open", 13
    )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    lateral = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    success = (
        lateral < 0.010
        and abs(nut["pos"][2] - resting_z) < 0.012
        and abs(square_error(nut["yaw"])) < 0.12
    )
    return {
        "status": "done" if success else "failed",
        "reason": "Released and checked seating" if success else "Final seating check failed",
        "lateral_error_m": lateral,
        "steps_used": state["steps_used"]
    }
