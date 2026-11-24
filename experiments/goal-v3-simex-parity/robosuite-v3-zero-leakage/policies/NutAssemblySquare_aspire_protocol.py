def wrap_angle(angle):
    return atan2(sin(angle), cos(angle))


def square_error(angle):
    return 0.25 * atan2(sin(4.0 * angle), cos(4.0 * angle))


def clip(value, limit):
    return max(-limit, min(limit, value))


def remaining(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def move(sdk, position, yaw, action, ticks, tolerance=0.004, reserve=0):
    budget = min(ticks, max(0, remaining(sdk) - reserve))
    if budget <= 0:
        return False
    result = sdk.move_to(
        position, yaw=yaw, gripper=action,
        max_steps=budget, tol=tolerance,
    )
    return result["reached"]


def grip(sdk, action, ticks, reserve=0):
    budget = min(ticks, max(0, remaining(sdk) - reserve))
    if budget > 0:
        sdk.set_gripper(action, steps=budget)


def nut_normal(nut):
    q = nut["quat"]
    normal = [
        2.0 * (q[0] * q[2] + q[3] * q[1]),
        2.0 * (q[1] * q[2] - q[3] * q[0]),
        1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1]),
    ]
    if normal[2] < 0.0:
        normal = [-normal[0], -normal[1], -normal[2]]
    return normal


def handle_position(nut, height):
    q = nut["quat"]
    axis = [
        1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]),
        2.0 * (q[0] * q[1] + q[3] * q[2]),
        2.0 * (q[0] * q[2] - q[3] * q[1]),
    ]
    return [
        nut["pos"][0] + 0.055 * axis[0],
        nut["pos"][1] + 0.055 * axis[1],
        nut["pos"][2] + 0.055 * axis[2] + height,
    ]


def servo_center(sdk, target, ticks, reserve=24, tolerance=0.003):
    for iteration in range(ticks):
        if remaining(sdk) <= reserve:
            return False
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        normal = nut_normal(nut)
        error = [target[index] - nut["pos"][index] for index in range(3)]
        yaw_error = square_error(nut["yaw"])
        if (
            hypot(error[0], error[1]) < tolerance
            and abs(error[2]) < tolerance
            and abs(yaw_error) < 0.035
            and hypot(normal[0], normal[1]) < 0.05
        ):
            return True
        sdk.step_raw(
            [
                clip(20.0 * error[0], 0.85),
                clip(20.0 * error[1], 0.85),
                clip(18.0 * error[2], 0.55),
                clip(3.5 * normal[1], 0.28),
                clip(-3.5 * normal[0], 0.28),
                clip(-4.0 * yaw_error, 0.45),
                1.0,
            ],
            steps=1,
        )
    return False


def seated(state, rest_height):
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    normal = nut_normal(nut)
    return (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.007
        and abs(nut["pos"][2] - rest_height) < 0.008
        and abs(square_error(nut["yaw"])) < 0.085
        and hypot(normal[0], normal[1]) < 0.10
    )


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or goal"}

    rest_height = state["objects"]["nut"]["pos"][2]
    clearance = max(rest_height + 0.15, state["goal_pos"][2] + 0.12)
    entry_height = max(rest_height + 0.095, state["goal_pos"][2] + 0.075)
    grasped = False

    # The failed lifts left the nut on the table. Put the finger center
    # at handle height instead of closing 20 mm above the thin handle.
    for attempt in range(2):
        if remaining(sdk) < 190:
            break

        state = sdk.get_state()
        eef = state["eef_pos"]
        grip(sdk, "open", 7, 165)
        move(
            sdk, [eef[0], eef[1], clearance],
            state["eef_yaw"], "open", 18, 0.006, 165,
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = wrap_angle(nut["yaw"])
        if abs(wrap_angle(yaw - state["eef_yaw"])) > pi / 2.0:
            yaw = wrap_angle(yaw + pi)

        target = handle_position(nut, 0.0)
        target[2] = clearance
        move(sdk, target, yaw, "open", 27, 0.004, 155)

        state = sdk.get_state()
        target = handle_position(state["objects"]["nut"], 0.025)
        move(sdk, target, yaw, "open", 18, 0.004, 150)

        state = sdk.get_state()
        depth = 0.001 if attempt == 0 else -0.004
        target = handle_position(state["objects"]["nut"], depth)
        move(sdk, target, yaw, "open", 20, 0.0025, 140)

        state = sdk.get_state()
        target = handle_position(state["objects"]["nut"], depth)
        if (
            hypot(
                state["eef_pos"][0] - target[0],
                state["eef_pos"][1] - target[1],
            ) > 0.005
            or abs(state["eef_pos"][2] - target[2]) > 0.006
        ):
            move(sdk, target, yaw, "open", 10, 0.0025, 135)

        state = sdk.get_state()
        target = handle_position(state["objects"]["nut"], depth)
        if (
            hypot(
                state["eef_pos"][0] - target[0],
                state["eef_pos"][1] - target[1],
            ) > 0.010
            or abs(state["eef_pos"][2] - target[2]) > 0.010
        ):
            continue

        before = list(state["objects"]["nut"]["pos"])
        grip(sdk, "close", 13, 125)

        state = sdk.get_state()
        eef = state["eef_pos"]
        move(
            sdk, [eef[0], eef[1], eef[2] + 0.065],
            state["eef_yaw"], "close", 22, 0.004, 110,
        )
        state = sdk.get_state()
        after = state["objects"]["nut"]["pos"]
        grasped = (
            after[2] > before[2] + 0.035
            and state["gripper_width"] > 0.010
            and hypot(after[0] - before[0], after[1] - before[1]) < 0.055
        )
        if grasped:
            break

    if not grasped:
        return {
            "status": "failed",
            "reason": "Handle grasp did not lift nut",
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    center = state["objects"]["nut"]["pos"]
    servo_center(sdk, [center[0], center[1], clearance], 27, 100)

    state = sdk.get_state()
    if state["objects"]["nut"]["pos"][2] < entry_height:
        return {
            "status": "failed",
            "reason": "Insufficient clearance after lift",
            "steps_used": state["steps_used"],
        }

    # Measure the grasp offset again after leveling and yaw correction.
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    eef = state["eef_pos"]
    goal = state["goal_pos"]
    transport = [
        eef[0] + goal[0] - nut["pos"][0],
        eef[1] + goal[1] - nut["pos"][1],
        eef[2] + clearance - nut["pos"][2],
    ]
    move(
        sdk, transport, state["eef_yaw"], "close",
        38, 0.006, 90,
    )

    state = sdk.get_state()
    goal = state["goal_pos"]
    servo_center(sdk, [goal[0], goal[1], clearance], 32, 65, 0.002)

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    normal = nut_normal(nut)
    if (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) > 0.006
        or abs(square_error(nut["yaw"])) > 0.065
        or hypot(normal[0], normal[1]) > 0.08
    ):
        return {
            "status": "failed",
            "reason": "Hole alignment did not converge",
            "steps_used": state["steps_used"],
        }

    servo_center(sdk, [goal[0], goal[1], entry_height], 20, 45, 0.002)
    stalled = 0
    recoveries = 0
    previous_height = sdk.get_state()["objects"]["nut"]["pos"][2]

    while remaining(sdk) > 25:
        state = sdk.get_state()
        if seated(state, rest_height):
            break

        nut = state["objects"]["nut"]
        center = nut["pos"]
        goal = state["goal_pos"]
        normal = nut_normal(nut)
        aligned = (
            hypot(center[0] - goal[0], center[1] - goal[1]) < 0.0035
            and abs(square_error(nut["yaw"])) < 0.045
            and hypot(normal[0], normal[1]) < 0.065
        )
        target_height = max(rest_height, center[2] - 0.022) if aligned else center[2]
        servo_center(
            sdk, [goal[0], goal[1], target_height],
            5, 25, 0.0015,
        )

        state = sdk.get_state()
        height = state["objects"]["nut"]["pos"][2]
        stalled = stalled + 1 if height > previous_height - 0.001 else 0
        previous_height = height

        if stalled >= 5:
            if recoveries >= 1 or remaining(sdk) < 65:
                break
            recoveries += 1
            state = sdk.get_state()
            goal = state["goal_pos"]
            height = state["objects"]["nut"]["pos"][2]
            servo_center(
                sdk, [goal[0], goal[1], max(entry_height, height + 0.025)],
                18, 40, 0.0015,
            )
            stalled = 0
            previous_height = sdk.get_state()["objects"]["nut"]["pos"][2]

    state = sdk.get_state()
    if not seated(state, rest_height):
        return {
            "status": "failed",
            "reason": "Insertion not seated",
            "center_error_m": hypot(
                state["objects"]["nut"]["pos"][0] - state["goal_pos"][0],
                state["objects"]["nut"]["pos"][1] - state["goal_pos"][1],
            ),
            "height_error_m": state["objects"]["nut"]["pos"][2] - rest_height,
            "steps_used": state["steps_used"],
        }

    grip(sdk, "open", 11, 14)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        sdk, [eef[0], eef[1], eef[2] + 0.075],
        state["eef_yaw"], "open", 14, 0.006,
    )

    state = sdk.get_state()
    success = seated(state, rest_height) and state["gripper_width"] > 0.05
    return {
        "status": "done" if success else "failed",
        "reason": "Released pose verified" if success else "Release verification failed",
        "steps_used": state["steps_used"],
    }
