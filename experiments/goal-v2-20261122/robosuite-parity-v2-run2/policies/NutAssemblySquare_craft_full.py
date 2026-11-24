def angle_wrap(angle):
    return atan2(sin(angle), cos(angle))


def square_yaw_error(yaw):
    return -atan2(sin(4 * yaw), cos(4 * yaw)) / 4


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or goal pose"}

    initial_nut_z = state["objects"]["nut"]["pos"][2]
    limit = min(400, state["max_steps"])

    def remaining():
        return max(0, limit - sdk.get_state()["steps_used"])

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
        return {"grasped_hint": False}

    def attached(state):
        nut_pos = state["objects"]["nut"]["pos"]
        eef = state["eef_pos"]
        return (
            0.015 <= state["gripper_width"] <= 0.055
            and hypot(nut_pos[0] - eef[0], nut_pos[1] - eef[1]) < 0.085
            and abs(nut_pos[2] - eef[2]) < 0.055
            and nut_pos[2] >= initial_nut_z + 0.06
        )

    held = False
    handle_sign = 1
    previous_width = 0.0

    for attempt in range(3):
        if remaining() < 205:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]

        if attempt == 2:
            handle_sign = -1
        elif attempt == 1 and previous_width < 0.012:
            handle_sign = -1

        orientation_offset = pi / 2
        if attempt == 1 and previous_width >= 0.012:
            orientation_offset = 0.0

        grasp_yaw = angle_wrap(yaw + orientation_offset)
        grasp = [
            nut["pos"][0] + handle_sign * 0.055 * cos(yaw),
            nut["pos"][1] + handle_sign * 0.055 * sin(yaw),
            nut["pos"][2]
        ]

        move(
            [grasp[0], grasp[1], grasp[2] + 0.10],
            grasp_yaw, "open", 38, 0.005
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        grasp_yaw = angle_wrap(yaw + orientation_offset)
        grasp = [
            nut["pos"][0] + handle_sign * 0.055 * cos(yaw),
            nut["pos"][1] + handle_sign * 0.055 * sin(yaw),
            nut["pos"][2]
        ]
        pickup_z = nut["pos"][2]

        move(grasp, grasp_yaw, "open", 36, 0.003)
        state = sdk.get_state()
        eef = state["eef_pos"]

        if (
            hypot(eef[0] - grasp[0], eef[1] - grasp[1]) > 0.008
            or abs(eef[2] - grasp[2]) > 0.008
        ):
            move(grasp, grasp_yaw, "open", 12, 0.003)

        actuate("close", 20)
        state = sdk.get_state()
        previous_width = state["gripper_width"]

        if not 0.015 <= previous_width <= 0.055:
            actuate("open", 8)
            continue

        eef = state["eef_pos"]
        goal = state["goal_pos"]
        lift_z = max(1.075, goal[2] + 0.18, pickup_z + 0.23)
        move(
            [eef[0], eef[1], lift_z],
            state["eef_yaw"], "close", 44, 0.004
        )

        state = sdk.get_state()
        held = (
            attached(state)
            and state["objects"]["nut"]["pos"][2] >= pickup_z + 0.06
        )
        if held:
            break

        if state["objects"]["nut"]["pos"][2] > pickup_z + 0.02:
            eef = state["eef_pos"]
            move(
                [eef[0], eef[1],
                 eef[2] + pickup_z - state["objects"]["nut"]["pos"][2]],
                state["eef_yaw"], "close", 18
            )
        actuate("open", 10)

    if not held:
        return {"status": "failed", "reason": "No verified handle grasp"}

    state = sdk.get_state()
    clearance_z = max(1.06, state["goal_pos"][2] + 0.16)
    nut = state["objects"]["nut"]
    eef = state["eef_pos"]

    if nut["pos"][2] < clearance_z - 0.004:
        move(
            [eef[0], eef[1], eef[2] + clearance_z - nut["pos"][2]],
            state["eef_yaw"], "close", 20, 0.003
        )

    aligned = False
    for iteration in range(4):
        if remaining() < 105:
            break

        state = sdk.get_state()
        if not attached(state):
            return {"status": "failed", "reason": "Grasp lost before alignment"}

        nut = state["objects"]["nut"]
        eef = state["eef_pos"]
        goal = state["goal_pos"]
        yaw_error = square_yaw_error(nut["yaw"])

        handle_x = eef[0] - nut["pos"][0]
        handle_y = eef[1] - nut["pos"][1]
        local_x = cos(nut["yaw"]) * handle_x + sin(nut["yaw"]) * handle_y
        local_y = -sin(nut["yaw"]) * handle_x + cos(nut["yaw"]) * handle_y
        target_nut_yaw = nut["yaw"] + yaw_error

        rotated_x = cos(target_nut_yaw) * local_x - sin(target_nut_yaw) * local_y
        rotated_y = sin(target_nut_yaw) * local_x + cos(target_nut_yaw) * local_y

        move(
            [
                goal[0] + rotated_x,
                goal[1] + rotated_y,
                eef[2] + clearance_z - nut["pos"][2]
            ],
            angle_wrap(state["eef_yaw"] + yaw_error),
            "close",
            min(48 if iteration == 0 else 16, remaining() - 100),
            0.003
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        if (
            attached(state)
            and hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.005
            and abs(square_yaw_error(nut["yaw"])) < 0.045
            and nut["pos"][2] >= clearance_z - 0.008
        ):
            aligned = True
            break

    if not aligned:
        return {"status": "failed", "reason": "Could not align the hole above the peg"}

    seating_z = initial_nut_z + 0.001
    goal = sdk.get_state()["goal_pos"]
    levels = [
        max(seating_z, goal[2] + 0.055),
        max(seating_z, goal[2] + 0.018),
        seating_z
    ]
    stalled = 0

    for level in levels:
        for correction in range(3):
            if remaining() <= 30:
                break

            state = sdk.get_state()
            nut = state["objects"]["nut"]
            nut_pos = nut["pos"]
            eef = state["eef_pos"]
            goal = state["goal_pos"]
            width = state["gripper_width"]

            if width < 0.012:
                return {"status": "failed", "reason": "Lost grasp during descent"}

            if (
                hypot(nut_pos[0] - eef[0], nut_pos[1] - eef[1]) > 0.085
                or abs(nut_pos[2] - eef[2]) > 0.055
            ):
                return {"status": "failed", "reason": "Nut slipped during descent"}

            xy_error = hypot(nut_pos[0] - goal[0], nut_pos[1] - goal[1])
            yaw_error = square_yaw_error(nut["yaw"])

            if (
                nut_pos[2] <= level + 0.005
                and xy_error < 0.005
                and abs(yaw_error) < 0.06
            ):
                break

            target_z = level
            if xy_error > 0.006 or abs(yaw_error) > 0.06:
                target_z = max(nut_pos[2], goal[2] + 0.055)
            if stalled >= 2:
                target_z = max(nut_pos[2] + 0.025, goal[2] + 0.055)
                stalled = 0

            offset_x = nut_pos[0] - eef[0]
            offset_y = nut_pos[1] - eef[1]
            rotated_x = cos(yaw_error) * offset_x - sin(yaw_error) * offset_y
            rotated_y = sin(yaw_error) * offset_x + cos(yaw_error) * offset_y

            target = [
                goal[0] - rotated_x,
                goal[1] - rotated_y,
                eef[2] + target_z - nut_pos[2]
            ]
            target_yaw = angle_wrap(state["eef_yaw"] + yaw_error)
            steps = 32 if level == levels[0] else 24
            move(
                target, target_yaw, "close",
                min(steps, remaining() - 28), 0.003
            )

            updated = sdk.get_state()["objects"]["nut"]["pos"]
            if target_z < nut_pos[2] - 0.008 and updated[2] > nut_pos[2] - 0.003:
                stalled += 1
            else:
                stalled = 0

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    if (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) >= 0.007
        or abs(nut["pos"][2] - seating_z) > 0.012
        or abs(square_yaw_error(nut["yaw"])) >= 0.10
    ):
        return {"status": "failed", "reason": "Nut did not seat on the peg"}

    actuate("open", 16)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.08],
        state["eef_yaw"], "open", min(18, remaining()), 0.006
    )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    xy_error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    seated = (
        xy_error < 0.008
        and abs(nut["pos"][2] - initial_nut_z) < 0.015
        and abs(square_yaw_error(nut["yaw"])) < 0.10
        and state["gripper_width"] > 0.045
    )
    return {
        "status": "done" if seated else "failed",
        "reason": "Released nut is seated" if seated else "Release verification failed",
        "center_error_m": xy_error,
        "handle_sign": handle_sign,
        "steps_used": state["steps_used"]
    }
