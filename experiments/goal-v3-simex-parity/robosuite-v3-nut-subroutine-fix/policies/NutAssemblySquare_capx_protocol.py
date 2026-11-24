def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def move(sdk, position, yaw, gripper, steps, tolerance=0.004):
    budget = min(steps, remaining_steps(sdk))
    if budget > 0:
        return sdk.move_to(
            position, yaw=yaw, gripper=gripper,
            max_steps=budget, tol=tolerance
        )
    return {"reached": False}


def grip(sdk, action, steps):
    budget = min(steps, remaining_steps(sdk))
    if budget > 0:
        return sdk.set_gripper(action, steps=budget)
    return {"grasped_hint": False}


def square_error(yaw):
    return atan2(sin(4.0 * yaw), cos(4.0 * yaw)) / 4.0


def upright(nut):
    quat = nut["quat"]
    return 1.0 - 2.0 * (quat[0] ** 2 + quat[1] ** 2)


def center_nut(sdk, target, steps, tolerance=0.004):
    state = sdk.get_state()
    nut = state["objects"]["nut"]["pos"]
    eef = state["eef_pos"]
    return move(
        sdk,
        [eef[i] + target[i] - nut[i] for i in range(3)],
        state["eef_yaw"], "close", steps, tolerance
    )


def align_square(sdk, steps):
    state = sdk.get_state()
    correction = -square_error(state["objects"]["nut"]["yaw"])
    if abs(correction) > 0.025:
        move(
            sdk, list(state["eef_pos"]),
            state["eef_yaw"] + correction,
            "close", steps, 0.004
        )


def seated(state, seat_z):
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    return (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.008
        and seat_z - 0.010 <= nut["pos"][2] <= seat_z + 0.012
        and upright(nut) > 0.97
        and abs(square_error(nut["yaw"])) < 0.12
    )


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or goal"}

    seat_z = state["objects"]["nut"]["pos"][2]
    grasped = False

    for attempt in range(2):
        if remaining_steps(sdk) < 205:
            break

        grip(sdk, "open", 8)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        turn = atan2(
            sin(yaw - state["eef_yaw"]),
            cos(yaw - state["eef_yaw"])
        )
        if turn > pi / 2.0:
            turn -= pi
        elif turn < -pi / 2.0:
            turn += pi

        grasp_yaw = state["eef_yaw"] + turn
        offset = 0.055 if attempt == 0 else 0.048
        hover_z = max(nut["pos"][2] + 0.085, state["eef_pos"][2])

        if state["eef_pos"][2] < nut["pos"][2] + 0.060:
            move(
                sdk,
                [state["eef_pos"][0], state["eef_pos"][1], hover_z],
                state["eef_yaw"], "open", 18
            )

        move(
            sdk,
            [
                nut["pos"][0] + offset * cos(yaw),
                nut["pos"][1] + offset * sin(yaw),
                hover_z
            ],
            grasp_yaw, "open", 40, 0.004
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        move(
            sdk,
            [
                nut["pos"][0] + offset * cos(nut["yaw"]),
                nut["pos"][1] + offset * sin(nut["yaw"]),
                nut["pos"][2] + 0.008
            ],
            grasp_yaw, "open", 28, 0.004
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        move(
            sdk,
            [
                nut["pos"][0] + offset * cos(nut["yaw"]),
                nut["pos"][1] + offset * sin(nut["yaw"]),
                nut["pos"][2] - 0.010
            ],
            grasp_yaw, "open", 16, 0.003
        )

        grip(sdk, "close", 18)
        state = sdk.get_state()
        baseline_z = state["objects"]["nut"]["pos"][2]
        eef = state["eef_pos"]
        move(
            sdk, [eef[0], eef[1], eef[2] + 0.070],
            state["eef_yaw"], "close", 24, 0.005
        )
        state = sdk.get_state()
        grasped = (
            state["objects"]["nut"]["pos"][2] > baseline_z + 0.035
            and state["gripper_width"] > 0.012
        )
        if grasped:
            break

    if not grasped:
        grip(sdk, "open", 8)
        return {
            "status": "failed",
            "reason": "Nut did not follow verification lift",
            "steps_used": sdk.get_state()["steps_used"]
        }

    state = sdk.get_state()
    goal = state["goal_pos"]
    clearance = max(seat_z + 0.135, goal[2] + 0.095)
    position = state["objects"]["nut"]["pos"]
    center_nut(sdk, [position[0], position[1], clearance], 22, 0.005)
    align_square(sdk, 18)

    state = sdk.get_state()
    goal = state["goal_pos"]
    center_nut(sdk, [goal[0], goal[1], clearance], 38, 0.005)

    for iteration in range(2):
        state = sdk.get_state()
        position = state["objects"]["nut"]["pos"]
        goal = state["goal_pos"]
        if hypot(position[0] - goal[0], position[1] - goal[1]) < 0.005:
            break
        if remaining_steps(sdk) < 100:
            break
        center_nut(sdk, [goal[0], goal[1], clearance], 14, 0.003)

    align_square(sdk, 12)
    stalls = 0

    for iteration in range(12):
        budget = remaining_steps(sdk) - 28
        if budget <= 0:
            break
        state = sdk.get_state()
        if seated(state, seat_z):
            break

        nut = state["objects"]["nut"]
        position = list(nut["pos"])
        goal = state["goal_pos"]
        lateral = hypot(position[0] - goal[0], position[1] - goal[1])

        if position[2] < seat_z - 0.020:
            return {"status": "failed", "reason": "Nut below seating plane"}
        if state["gripper_width"] < 0.010:
            return {"status": "failed", "reason": "Grasp lost during insertion"}

        if lateral > 0.012:
            target_z = max(clearance, position[2])
            duration = 20
        elif lateral > 0.006:
            target_z = position[2]
            duration = 14
        else:
            target_z = max(seat_z, position[2] - 0.045)
            duration = 22

        center_nut(
            sdk, [goal[0], goal[1], target_z],
            min(duration, budget), 0.003
        )
        after = sdk.get_state()
        after_position = after["objects"]["nut"]["pos"]
        if (
            target_z < position[2] - 0.008
            and position[2] - after_position[2] < 0.004
        ):
            stalls += 1
        else:
            stalls = 0

        if seated(after, seat_z):
            break

        if stalls >= 1 and remaining_steps(sdk) > 65:
            center_nut(
                sdk,
                [after_position[0], after_position[1], after_position[2] + 0.015],
                10, 0.004
            )
            align_square(sdk, 12)
            state = sdk.get_state()
            goal = state["goal_pos"]
            center_nut(
                sdk,
                [goal[0], goal[1], state["objects"]["nut"]["pos"][2]],
                12, 0.002
            )
            stalls = 0

    state = sdk.get_state()
    if not seated(state, seat_z):
        return {
            "status": "failed",
            "reason": "Insertion depth or alignment not verified",
            "steps_used": state["steps_used"]
        }

    grip(sdk, "open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        sdk, [eef[0], eef[1], eef[2] + 0.070],
        state["eef_yaw"], "open", 16, 0.006
    )
    state = sdk.get_state()
    success = seated(state, seat_z) and state["gripper_width"] > 0.050
    return {
        "status": "done" if success else "failed",
        "reason": "Released nut seated on peg" if success else "Final pose check failed",
        "steps_used": state["steps_used"]
    }
