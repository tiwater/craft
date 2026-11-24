def wrap_angle(angle):
    return atan2(sin(angle), cos(angle))


def square_yaw_error(yaw):
    return 0.25 * atan2(sin(-4.0 * yaw), cos(-4.0 * yaw))


def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def bounded_move(sdk, position, yaw, gripper, steps, tolerance):
    budget = remaining_steps(sdk)
    if budget > 0:
        return sdk.move_to(
            position,
            yaw=wrap_angle(yaw),
            gripper=gripper,
            max_steps=min(steps, budget),
            tol=tolerance,
        )
    return {"reached": False}


def actuate_gripper(sdk, action, steps):
    budget = remaining_steps(sdk)
    if budget > 0:
        sdk.set_gripper(action, steps=min(steps, budget))


def grasp_target(state):
    nut = state["objects"]["nut"]
    yaw = nut["yaw"]
    target = [
        nut["pos"][0] + 0.055 * cos(yaw),
        nut["pos"][1] + 0.055 * sin(yaw),
        nut["pos"][2],
    ]
    wrist_yaw = state["eef_yaw"] + 0.5 * atan2(
        sin(2.0 * (yaw - state["eef_yaw"])),
        cos(2.0 * (yaw - state["eef_yaw"])),
    )
    return target, wrist_yaw


def acquire_nut(sdk):
    for attempt in range(2):
        if remaining_steps(sdk) < 195:
            break

        actuate_gripper(sdk, "open", 12)
        state = sdk.get_state()
        target, yaw = grasp_target(state)

        hover = [target[0], target[1], target[2] + 0.085]
        bounded_move(sdk, hover, yaw, "open", 40, 0.005)

        state = sdk.get_state()
        target, yaw = grasp_target(state)
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - target[0], eef[1] - target[1]) > 0.008
            or abs(wrap_angle(yaw - state["eef_yaw"])) > 0.08
        ):
            hover = [target[0], target[1], target[2] + 0.085]
            bounded_move(sdk, hover, yaw, "open", 16, 0.004)

        contact_ready = False
        for correction in range(4):
            if remaining_steps(sdk) < 155:
                break
            state = sdk.get_state()
            target, yaw = grasp_target(state)
            bounded_move(
                sdk, target, yaw, "open",
                32 if correction == 0 else 12, 0.003,
            )
            state = sdk.get_state()
            target, yaw = grasp_target(state)
            eef = state["eef_pos"]
            if (
                hypot(eef[0] - target[0], eef[1] - target[1]) < 0.005
                and abs(eef[2] - target[2]) < 0.005
                and abs(wrap_angle(yaw - state["eef_yaw"])) < 0.08
                and state["gripper_width"] > 0.06
            ):
                contact_ready = True
                break

        if not contact_ready:
            return False

        before_height = sdk.get_state()["objects"]["nut"]["pos"][2]
        actuate_gripper(sdk, "close", 16)
        state = sdk.get_state()

        if state["gripper_width"] < 0.012:
            actuate_gripper(sdk, "open", 10)
            continue

        lift = list(state["eef_pos"])
        lift[2] += 0.075
        bounded_move(sdk, lift, state["eef_yaw"], "close", 30, 0.005)
        state = sdk.get_state()
        rise = state["objects"]["nut"]["pos"][2] - before_height

        if rise > 0.035 and 0.012 < state["gripper_width"] < 0.065:
            return True

        actuate_gripper(sdk, "open", 10)

    return False


def move_nut(sdk, target, steps, tolerance, yaw_limit):
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    eef = state["eef_pos"]
    offset = [nut["pos"][i] - eef[i] for i in range(3)]
    turn = max(-yaw_limit, min(yaw_limit, square_yaw_error(nut["yaw"])))
    rotated = [
        cos(turn) * offset[0] - sin(turn) * offset[1],
        sin(turn) * offset[0] + cos(turn) * offset[1],
        offset[2],
    ]
    wrist_target = [target[i] - rotated[i] for i in range(3)]
    bounded_move(
        sdk,
        wrist_target,
        state["eef_yaw"] + turn,
        "close",
        steps,
        tolerance,
    )


def run_policy(sdk):
    initial = sdk.get_state()
    if "nut" not in initial["objects"] or initial["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    resting_height = initial["objects"]["nut"]["pos"][2]

    if not acquire_nut(sdk):
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    goal = state["goal_pos"]
    clearance = max(resting_height + 0.12, goal[2] + 0.10)
    nut = state["objects"]["nut"]

    # Rotate at clearance before translating across the peg.
    move_nut(
        sdk,
        [nut["pos"][0], nut["pos"][1], clearance],
        min(30, max(0, remaining_steps(sdk) - 135)),
        0.005,
        pi / 4.0,
    )

    # Recompute the grasp offset after rotation and translation.
    for iteration in range(3):
        budget = remaining_steps(sdk) - 105
        if budget <= 0:
            break
        state = sdk.get_state()
        goal = state["goal_pos"]
        move_nut(
            sdk,
            [goal[0], goal[1], clearance],
            min(40 if iteration == 0 else 16, budget),
            0.003,
            pi / 4.0,
        )
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        if (
            hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.004
            and abs(square_yaw_error(nut["yaw"])) < 0.045
            and nut["pos"][2] > clearance - 0.012
        ):
            break

    seated_height = resting_height + 0.001

    # Use an absolute seating target. Short relative descent targets spend
    # most of their control ticks approaching each intermediate waypoint.
    for iteration in range(6):
        budget = remaining_steps(sdk) - 32
        if budget <= 0:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        position = nut["pos"]
        lateral_error = hypot(position[0] - goal[0], position[1] - goal[1])
        yaw_error = abs(square_yaw_error(nut["yaw"]))

        if (
            position[2] <= seated_height + 0.008
            and lateral_error < 0.008
            and yaw_error < 0.10
        ):
            break

        if lateral_error > 0.008 or yaw_error > 0.08:
            target_height = max(position[2], clearance)
            allowance = 18
        else:
            target_height = seated_height
            allowance = 42 if iteration == 0 else 24

        move_nut(
            sdk,
            [goal[0], goal[1], target_height],
            min(allowance, budget),
            0.0025,
            pi / 4.0 if target_height >= clearance else 0.08,
        )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    lateral_error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])

    if (
        lateral_error > 0.012
        or nut["pos"][2] > resting_height + 0.025
        or abs(square_yaw_error(nut["yaw"])) > 0.15
    ):
        return {
            "status": "failed",
            "reason": "Insertion did not reach seated pose",
            "lateral_error_m": lateral_error,
            "height_error_m": nut["pos"][2] - resting_height,
            "steps_used": state["steps_used"],
        }

    actuate_gripper(sdk, "open", 12)
    state = sdk.get_state()
    retreat = list(state["eef_pos"])
    retreat[2] += 0.10
    bounded_move(sdk, retreat, state["eef_yaw"], "open", 18, 0.01)

    final = sdk.get_state()
    nut = final["objects"]["nut"]
    goal = final["goal_pos"]
    lateral_error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    seated = (
        lateral_error < 0.015
        and abs(nut["pos"][2] - resting_height) < 0.025
        and abs(square_yaw_error(nut["yaw"])) < 0.15
        and final["gripper_width"] > 0.05
    )
    return {
        "status": "done" if seated else "failed",
        "verification": "Released nut pose relative to peg and initial resting height",
        "lateral_error_m": lateral_error,
        "height_error_m": nut["pos"][2] - resting_height,
        "steps_used": final["steps_used"],
    }
