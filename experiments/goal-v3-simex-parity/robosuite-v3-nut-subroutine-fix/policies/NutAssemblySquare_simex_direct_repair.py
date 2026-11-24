def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def angle(target, current):
        return atan2(sin(target - current), cos(target - current))

    def square_error(yaw):
        return -atan2(sin(4.0 * yaw), cos(4.0 * yaw)) / 4.0

    def move(position, yaw, action, steps, tolerance=0.003):
        budget = min(steps, remaining())
        if budget > 0:
            return sdk.move_to(
                position, yaw=yaw, gripper=action,
                max_steps=budget, tol=tolerance
            )
        return {"reached": False}

    def grip(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def place_center(target, steps, align=True):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        eef = state["eef_pos"]
        correction = square_error(nut["yaw"]) if align else 0.0
        correction = max(-0.30, min(0.30, correction))
        dx = eef[0] - nut["pos"][0]
        dy = eef[1] - nut["pos"][1]
        return move(
            [
                target[0] + cos(correction) * dx - sin(correction) * dy,
                target[1] + sin(correction) * dx + cos(correction) * dy,
                target[2] + eef[2] - nut["pos"][2],
            ],
            state["eef_yaw"] + correction,
            "close", steps, 0.002
        )

    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    seat_z = state["objects"]["nut"]["pos"][2]
    grasped = False

    for attempt in range(3):
        if remaining() < 150:
            break

        grip("open", 6)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        high_z = max(state["eef_pos"][2], nut["pos"][2] + 0.080)
        move(
            [state["eef_pos"][0], state["eef_pos"][1], high_z],
            state["eef_yaw"], "open", 16, 0.005
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        offset = 0.055 if attempt < 2 else 0.060
        yaw = nut["yaw"]
        target_yaw = state["eef_yaw"] + angle(yaw, state["eef_yaw"])
        move(
            [
                nut["pos"][0] + offset * cos(yaw),
                nut["pos"][1] + offset * sin(yaw),
                high_z,
            ],
            target_yaw, "open", 32, 0.003
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        target_yaw = state["eef_yaw"] + angle(yaw, state["eef_yaw"])
        grasp_position = [
            nut["pos"][0] + offset * cos(yaw),
            nut["pos"][1] + offset * sin(yaw),
            nut["pos"][2] + (0.016 if attempt == 0 else 0.010),
        ]
        move(grasp_position, target_yaw, "open", 32, 0.003)

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        handle_x = nut["pos"][0] + offset * cos(nut["yaw"])
        handle_y = nut["pos"][1] + offset * sin(nut["yaw"])
        if (
            hypot(state["eef_pos"][0] - handle_x,
                  state["eef_pos"][1] - handle_y) > 0.008
            or state["eef_pos"][2] - nut["pos"][2] > 0.035
        ):
            continue

        baseline = list(nut["pos"])
        grip("close", 12)
        state = sdk.get_state()
        lift_start = list(state["eef_pos"])
        move(
            [lift_start[0], lift_start[1], lift_start[2] + 0.055],
            state["eef_yaw"], "close", 24, 0.003
        )
        state = sdk.get_state()
        rise = state["objects"]["nut"]["pos"][2] - baseline[2]
        eef_rise = state["eef_pos"][2] - lift_start[2]

        if rise > 0.028 and rise > 0.65 * eef_rise:
            position = state["objects"]["nut"]["pos"]
            place_center(
                [position[0], position[1], seat_z + 0.135],
                24, False
            )
            state = sdk.get_state()
            if state["objects"]["nut"]["pos"][2] > seat_z + 0.095:
                grasped = True
                break

        state = sdk.get_state()
        position = state["objects"]["nut"]["pos"]
        if position[2] > baseline[2] + 0.008:
            place_center([position[0], position[1], baseline[2]], 16, False)

    if not grasped:
        grip("open", 6)
        return {
            "status": "failed",
            "reason": "Nut did not follow handle grasp",
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    goal = state["goal_pos"]
    clearance_z = max(seat_z + 0.125, goal[2] + 0.085)

    for iteration in range(7):
        if remaining() <= 95:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        if (
            hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.003
            and abs(square_error(nut["yaw"])) < 0.025
            and abs(nut["pos"][2] - clearance_z) < 0.006
        ):
            break
        place_center([goal[0], goal[1], clearance_z], 20)

    stalls = 0
    for iteration in range(24):
        if remaining() <= 24:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        position = nut["pos"]
        goal = state["goal_pos"]
        lateral = hypot(position[0] - goal[0], position[1] - goal[1])
        yaw_error = abs(square_error(nut["yaw"]))

        if lateral < 0.006 and abs(position[2] - seat_z) < 0.008 and yaw_error < 0.06:
            break

        if lateral > 0.0035 or yaw_error > 0.030:
            target_z = position[2]
            if position[2] < seat_z + 0.075:
                target_z = min(clearance_z, position[2] + 0.020)
            place_center(
                [goal[0], goal[1], target_z],
                min(14, remaining() - 24)
            )
            continue

        previous_z = position[2]
        descent = 0.030 if previous_z > seat_z + 0.060 else 0.010
        place_center(
            [goal[0], goal[1], max(seat_z, previous_z - descent)],
            min(12, remaining() - 24)
        )
        state = sdk.get_state()
        position = state["objects"]["nut"]["pos"]
        if previous_z - position[2] < 0.0015:
            stalls += 1
        else:
            stalls = 0

        if stalls >= 2 and remaining() > 60:
            place_center(
                [position[0], position[1], position[2] + 0.022],
                12, False
            )
            state = sdk.get_state()
            goal = state["goal_pos"]
            position = state["objects"]["nut"]["pos"]
            place_center([goal[0], goal[1], position[2]], 12)
            stalls = 0

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    position = nut["pos"]
    goal = state["goal_pos"]
    seated = (
        hypot(position[0] - goal[0], position[1] - goal[1]) < 0.008
        and abs(position[2] - seat_z) < 0.010
        and abs(square_error(nut["yaw"])) < 0.08
    )
    if not seated:
        return {
            "status": "failed",
            "reason": "Insertion depth was not verified",
            "nut_pos": list(position),
            "goal_pos": list(goal),
            "steps_used": state["steps_used"],
        }

    grip("open", 10)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.075],
        state["eef_yaw"], "open", 14, 0.006
    )
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    position = nut["pos"]
    goal = state["goal_pos"]
    success = (
        hypot(position[0] - goal[0], position[1] - goal[1]) < 0.012
        and abs(position[2] - seat_z) < 0.012
        and abs(square_error(nut["yaw"])) < 0.10
        and state["gripper_width"] > 0.055
    )
    return {
        "status": "done" if success else "failed",
        "reason": "Verified released nut pose" if success else "Final pose check failed",
        "steps_used": state["steps_used"],
    }
