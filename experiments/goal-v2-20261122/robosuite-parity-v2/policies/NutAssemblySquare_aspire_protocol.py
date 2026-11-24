def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def wrap(angle):
        return atan2(sin(angle), cos(angle))

    def square_error(angle):
        return atan2(sin(4.0 * angle), cos(4.0 * angle)) / 4.0

    def upright(nut):
        quat = nut["quat"]
        return 1.0 - 2.0 * (quat[0] ** 2 + quat[1] ** 2)

    def move(pos, yaw, grip, steps, tol=0.003):
        budget = min(steps, remaining())
        if budget > 0:
            return sdk.move_to(
                pos, yaw=wrap(yaw), gripper=grip,
                max_steps=budget, tol=tol
            )
        return {"reached": False}

    def grip(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def fail(reason):
        return {
            "status": "failed",
            "reason": reason,
            "steps_used": sdk.get_state()["steps_used"]
        }

    def heading(state):
        # The jaw closing direction must cross the handle, rather than
        # squeeze its ends. Choose the equivalent wrist angle nearest now.
        yaw = state["objects"]["nut"]["yaw"]
        current = state["eef_yaw"]
        return current + atan2(
            sin(2.0 * (yaw - current)),
            cos(2.0 * (yaw - current))
        ) / 2.0

    def handle(state, distance):
        nut = state["objects"]["nut"]
        return [
            nut["pos"][0] + distance * cos(nut["yaw"]),
            nut["pos"][1] + distance * sin(nut["yaw"]),
            nut["pos"][2]
        ]

    def place(target, steps, turn_limit=0.14):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        offset = [
            state["eef_pos"][index] - nut["pos"][index]
            for index in range(3)
        ]
        turn = max(-turn_limit, min(turn_limit, -square_error(nut["yaw"])))
        move(
            [
                target[0] + cos(turn) * offset[0] - sin(turn) * offset[1],
                target[1] + sin(turn) * offset[0] + cos(turn) * offset[1],
                target[2] + offset[2]
            ],
            state["eef_yaw"] + turn, "close", steps, 0.002
        )

    def seated(state):
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        return (
            hypot(nut["pos"][0] - goal[0],
                  nut["pos"][1] - goal[1]) < 0.008
            and abs(nut["pos"][2] - floor_z) < 0.012
            and abs(square_error(nut["yaw"])) < 0.10
            and upright(nut) > 0.95
        )

    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return fail("Missing nut or peg pose")

    floor_z = state["objects"]["nut"]["pos"][2]
    if seated(state):
        grip("open", 8)
        return {
            "status": "done" if seated(sdk.get_state()) else "failed",
            "steps_used": sdk.get_state()["steps_used"]
        }

    held = False
    grip("open", 6)

    # The previous rollouts mostly never lifted the nut. Change the jaw
    # heading, verify a small lift, then verify sustained attachment.
    for attempt in range(2):
        if remaining() < 175:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        distance = 0.043 if attempt == 0 else 0.035
        point = handle(state, distance)
        safe_z = max(state["eef_pos"][2], nut["pos"][2] + 0.070)

        if state["eef_pos"][2] < nut["pos"][2] + 0.045:
            move(
                [state["eef_pos"][0], state["eef_pos"][1], safe_z],
                state["eef_yaw"], "open", 12
            )

        move([point[0], point[1], safe_z],
             heading(state), "open", 25)
        state = sdk.get_state()
        point = handle(state, distance)
        move([point[0], point[1], point[2]],
             heading(state), "open", 24, 0.002)
        grip("close", 16)

        state = sdk.get_state()
        before_nut = list(state["objects"]["nut"]["pos"])
        before_eef = list(state["eef_pos"])
        move(
            [before_eef[0], before_eef[1], before_eef[2] + 0.035],
            state["eef_yaw"], "close", 15
        )
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        rise = nut["pos"][2] - before_nut[2]
        eef_rise = state["eef_pos"][2] - before_eef[2]
        captured = (
            rise > 0.018
            and abs(rise - eef_rise) < 0.012
            and 0.012 < state["gripper_width"] < 0.070
            and upright(nut) > 0.94
        )

        if captured:
            before_nut = list(nut["pos"])
            before_eef = list(state["eef_pos"])
            move(
                [before_eef[0], before_eef[1], before_eef[2] + 0.045],
                state["eef_yaw"], "close", 17
            )
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            held = (
                nut["pos"][2] > floor_z + 0.055
                and abs(
                    (nut["pos"][2] - before_nut[2])
                    - (state["eef_pos"][2] - before_eef[2])
                ) < 0.012
                and state["gripper_width"] > 0.012
                and upright(nut) > 0.94
            )
        if held:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        if nut["pos"][2] > floor_z + 0.012:
            eef = state["eef_pos"]
            move(
                [eef[0], eef[1], eef[2] + floor_z + 0.002 - nut["pos"][2]],
                state["eef_yaw"], "close", 15
            )
        grip("open", 8)

    if not held:
        return fail("Handle grasp failed lift verification")

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    attachment = hypot(
        state["eef_pos"][0] - nut["pos"][0],
        state["eef_pos"][1] - nut["pos"][1]
    )
    attachment_z = state["eef_pos"][2] - nut["pos"][2]
    clearance_z = max(floor_z + 0.105, state["goal_pos"][2] + 0.075)

    # Translate in bounded segments while correcting yaw. A single long
    # transfer can conceal a slipping grasp until the nut is far behind.
    for stage in range(20):
        if remaining() < 85:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        pos = nut["pos"]
        goal = state["goal_pos"]
        offset_xy = hypot(
            state["eef_pos"][0] - pos[0],
            state["eef_pos"][1] - pos[1]
        )
        offset_z = state["eef_pos"][2] - pos[2]
        if (
            state["gripper_width"] < 0.010
            or abs(offset_xy - attachment) > 0.022
            or abs(offset_z - attachment_z) > 0.020
            or upright(nut) < 0.90
        ):
            return fail("Attachment lost during transfer")

        distance = hypot(goal[0] - pos[0], goal[1] - pos[1])
        if (
            distance < 0.004
            and abs(pos[2] - clearance_z) < 0.007
            and abs(square_error(nut["yaw"])) < 0.045
        ):
            break

        if pos[2] < clearance_z - 0.012:
            target = [pos[0], pos[1], clearance_z]
        else:
            fraction = min(1.0, 0.065 / max(distance, 0.001))
            target = [
                pos[0] + fraction * (goal[0] - pos[0]),
                pos[1] + fraction * (goal[1] - pos[1]),
                clearance_z
            ]
        place(target, 12, 0.16)

    stalls = 0
    recoveries = 0
    for stage in range(30):
        if remaining() <= 24:
            break
        state = sdk.get_state()
        if seated(state):
            break
        nut = state["objects"]["nut"]
        pos = list(nut["pos"])
        goal = state["goal_pos"]
        lateral = hypot(pos[0] - goal[0], pos[1] - goal[1])
        angular = abs(square_error(nut["yaw"]))

        if upright(nut) < 0.90:
            return fail("Nut tilted during insertion")
        if pos[2] < floor_z + 0.020 and lateral > 0.020:
            return fail("Nut dropped before insertion")

        budget = min(12, remaining() - 24)
        if lateral > 0.004 or angular > 0.045:
            target_z = pos[2]
            if lateral > 0.012:
                target_z = max(clearance_z, target_z)
            place([goal[0], goal[1], target_z], budget, 0.10)
            continue

        place(
            [goal[0], goal[1], max(floor_z + 0.001, pos[2] - 0.022)],
            budget, 0.06
        )
        updated = sdk.get_state()
        new_z = updated["objects"]["nut"]["pos"][2]
        if pos[2] - new_z < 0.002 and new_z > floor_z + 0.012:
            stalls += 1
        else:
            stalls = 0

        if stalls >= 2:
            if recoveries >= 1 or remaining() < 65:
                break
            recoveries += 1
            stalls = 0
            goal = updated["goal_pos"]
            place(
                [goal[0], goal[1], max(clearance_z, new_z + 0.025)],
                15, 0.10
            )

    state = sdk.get_state()
    if not seated(state):
        return fail("Insertion not verified")

    grip("open", 10)
    state = sdk.get_state()
    eef = list(state["eef_pos"])
    move([eef[0], eef[1], eef[2] + 0.065],
         state["eef_yaw"], "open", 10)
    grip("open", 4)
    state = sdk.get_state()
    success = seated(state)
    return {
        "status": "done" if success else "failed",
        "reason": "Released insertion verified" if success else "Placement shifted",
        "steps_used": state["steps_used"]
    }
