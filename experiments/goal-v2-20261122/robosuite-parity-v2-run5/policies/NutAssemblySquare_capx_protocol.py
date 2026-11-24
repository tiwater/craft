def wrap_angle(angle):
    return (angle + pi) % (2.0 * pi) - pi


def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def move_pose(sdk, position, yaw, grip, ticks, tolerance=0.004):
    budget = remaining_steps(sdk)
    if budget <= 0 or ticks <= 0:
        return False
    result = sdk.move_to(
        position,
        yaw=yaw,
        gripper=grip,
        max_steps=min(ticks, budget),
        tol=tolerance,
    )
    return result["reached"]


def actuate_gripper(sdk, action, ticks):
    budget = remaining_steps(sdk)
    if budget > 0:
        sdk.set_gripper(action, steps=min(ticks, budget))


def move_nut(sdk, target, ticks, align=True, tolerance=0.003):
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    eef = state["eef_pos"]
    correction = 0.0
    if align:
        correction = -((nut["yaw"] + pi / 4.0) % (pi / 2.0) - pi / 4.0)
    offset_x = eef[0] - nut["pos"][0]
    offset_y = eef[1] - nut["pos"][1]
    position = [
        target[0] + cos(correction) * offset_x - sin(correction) * offset_y,
        target[1] + sin(correction) * offset_x + cos(correction) * offset_y,
        target[2] + eef[2] - nut["pos"][2],
    ]
    return move_pose(
        sdk, position, wrap_angle(state["eef_yaw"] + correction),
        "close", ticks, tolerance
    )


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    resting_height = state["objects"]["nut"]["pos"][2]
    clearance = max(resting_height + 0.14, state["goal_pos"][2] + 0.11)
    grasped = False

    for attempt in range(2):
        if remaining_steps(sdk) < 185:
            break

        actuate_gripper(sdk, "open", 12)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        opposite = wrap_angle(yaw + pi)
        if abs(wrap_angle(opposite - state["eef_yaw"])) < abs(
            wrap_angle(yaw - state["eef_yaw"])
        ):
            yaw = opposite

        handle_x = nut["pos"][0] + 0.055 * cos(nut["yaw"])
        handle_y = nut["pos"][1] + 0.055 * sin(nut["yaw"])
        move_pose(
            sdk, [handle_x, handle_y, nut["pos"][2] + 0.085],
            yaw, "open", 30, 0.006
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        initial_height = nut["pos"][2]
        handle_x = nut["pos"][0] + 0.055 * cos(nut["yaw"])
        handle_y = nut["pos"][1] + 0.055 * sin(nut["yaw"])
        grasp_height = nut["pos"][2] - 0.010 - 0.004 * attempt
        move_pose(
            sdk, [handle_x, handle_y, grasp_height],
            yaw, "open", 44, 0.003
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        handle_x = nut["pos"][0] + 0.055 * cos(nut["yaw"])
        handle_y = nut["pos"][1] + 0.055 * sin(nut["yaw"])
        if (
            hypot(state["eef_pos"][0] - handle_x,
                  state["eef_pos"][1] - handle_y) > 0.005
            or state["eef_pos"][2] > nut["pos"][2] + 0.002
        ):
            move_pose(
                sdk, [handle_x, handle_y, grasp_height],
                yaw, "open", 15, 0.002
            )

        actuate_gripper(sdk, "close", 18)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move_pose(
            sdk, [eef[0], eef[1], eef[2] + 0.045],
            state["eef_yaw"], "close", 23, 0.005
        )
        state = sdk.get_state()
        grasped = (
            state["objects"]["nut"]["pos"][2] > initial_height + 0.022
            and state["gripper_width"] > 0.010
        )
        if grasped:
            break

        actuate_gripper(sdk, "open", 10)

    if not grasped:
        return {
            "status": "failed",
            "reason": "Nut did not follow grasp lift",
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    move_nut(
        sdk, [nut["pos"][0], nut["pos"][1], clearance],
        28, True, 0.005
    )
    state = sdk.get_state()
    if state["objects"]["nut"]["pos"][2] < clearance - 0.025:
        return {"status": "failed", "reason": "Insufficient lift clearance"}

    goal = state["goal_pos"]
    move_nut(
        sdk, [goal[0], goal[1], clearance],
        min(58, max(0, remaining_steps(sdk) - 70)), True, 0.004
    )

    for iteration in range(4):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        lateral = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
        angular = abs((nut["yaw"] + pi / 4.0) % (pi / 2.0) - pi / 4.0)
        if lateral < 0.0035 and angular < 0.04:
            break
        if remaining_steps(sdk) <= 65:
            break
        move_nut(
            sdk, [goal[0], goal[1], clearance],
            min(14, remaining_steps(sdk) - 65), True, 0.002
        )

    stalled = 0
    for iteration in range(12):
        if remaining_steps(sdk) <= 23:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        position = nut["pos"]
        lateral = hypot(position[0] - goal[0], position[1] - goal[1])
        angular = abs((nut["yaw"] + pi / 4.0) % (pi / 2.0) - pi / 4.0)

        if lateral < 0.007 and abs(position[2] - resting_height) < 0.008:
            break
        if lateral > 0.020:
            return {
                "status": "failed",
                "reason": "Nut is not centered above peg",
                "steps_used": state["steps_used"],
            }

        target_height = max(resting_height, position[2] - 0.030)
        if lateral > 0.0045 or angular > 0.065:
            target_height = position[2]

        previous_height = position[2]
        move_nut(
            sdk, [goal[0], goal[1], target_height],
            min(17, remaining_steps(sdk) - 23), True, 0.0025
        )

        state = sdk.get_state()
        current_height = state["objects"]["nut"]["pos"][2]
        if target_height < previous_height - 0.010:
            if previous_height - current_height < 0.003:
                stalled += 1
            else:
                stalled = 0

        if stalled >= 2 and remaining_steps(sdk) > 65:
            nut = state["objects"]["nut"]
            goal = state["goal_pos"]
            move_nut(
                sdk, [goal[0], goal[1], nut["pos"][2] + 0.018],
                12, False, 0.003
            )
            state = sdk.get_state()
            goal = state["goal_pos"]
            move_nut(
                sdk,
                [goal[0], goal[1], state["objects"]["nut"]["pos"][2]],
                12, True, 0.002
            )
            stalled = 0

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    seated = (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.008
        and abs(nut["pos"][2] - resting_height) < 0.010
    )
    if not seated:
        return {
            "status": "failed",
            "reason": "Nut has not reached seated height",
            "steps_used": state["steps_used"],
        }

    actuate_gripper(sdk, "open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move_pose(
        sdk, [eef[0], eef[1], eef[2] + 0.060],
        state["eef_yaw"], "open", remaining_steps(sdk), 0.008
    )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    settled = (
        error < 0.010
        and abs(nut["pos"][2] - resting_height) < 0.012
        and state["gripper_width"] > 0.045
    )
    return {
        "status": "done" if settled else "failed",
        "reason": "Released nut is seated" if settled else "Release verification failed",
        "center_error_m": error,
        "steps_used": state["steps_used"],
    }
