def wrap_angle(angle):
    return atan2(sin(angle), cos(angle))


def square_yaw_error(yaw):
    return 0.25 * atan2(sin(-4.0 * yaw), cos(-4.0 * yaw))


def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def clamp(value, limit):
    return max(-limit, min(limit, value))


def distance(a, b):
    return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def nut_tilt(nut):
    x, y, z, w = nut["quat"]
    vertical = 1.0 - 2.0 * (x * x + y * y)
    return sqrt(max(0.0, 1.0 - vertical * vertical))


def grip(sdk, action, ticks, reserve=0):
    budget = min(ticks, remaining_steps(sdk) - reserve)
    if budget > 0:
        sdk.set_gripper(action, steps=budget)


def servo_tick(sdk, target, yaw, closed, limit=0.65):
    state = sdk.get_state()
    position = state["eef_pos"]
    error = [target[i] - position[i] for i in range(3)]
    norm = sqrt(sum(value * value for value in error))
    scale = min(18.0, limit / max(norm, 0.000001))
    turn = wrap_angle(yaw - state["eef_yaw"])
    sdk.step_raw(
        [
            error[0] * scale,
            error[1] * scale,
            error[2] * scale,
            0.0,
            0.0,
            clamp(1.5 * turn, 0.45),
            1.0 if closed else -1.0,
        ],
        steps=1,
    )


def pose_servo(sdk, target, yaw, closed, allowance, reserve, tolerance):
    start = sdk.get_state()["steps_used"]
    stable = 0
    for iteration in range(allowance):
        state = sdk.get_state()
        if (
            state["steps_used"] - start >= allowance
            or remaining_steps(sdk) <= reserve
        ):
            break
        position_error = distance(target, state["eef_pos"])
        yaw_error = abs(wrap_angle(yaw - state["eef_yaw"]))
        if position_error < tolerance and yaw_error < 0.08:
            stable += 1
            if stable >= 2:
                return True
        else:
            stable = 0
        before = state["steps_used"]
        servo_tick(sdk, target, yaw, closed)
        if sdk.get_state()["steps_used"] <= before:
            break
    state = sdk.get_state()
    return (
        distance(target, state["eef_pos"]) < tolerance
        and abs(wrap_angle(yaw - state["eef_yaw"])) < 0.12
    )


def handle_pose(state, radius, height, orientation):
    nut = state["objects"]["nut"]
    angle = nut["yaw"]
    desired = angle + orientation
    yaw = state["eef_yaw"] + 0.5 * atan2(
        sin(2.0 * (desired - state["eef_yaw"])),
        cos(2.0 * (desired - state["eef_yaw"])),
    )
    return [
        nut["pos"][0] + radius * cos(angle),
        nut["pos"][1] + radius * sin(angle),
        nut["pos"][2] + height,
    ], yaw


def nut_tick(sdk, target, yaw_limit=0.12, speed=0.60):
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    turn = clamp(square_yaw_error(nut["yaw"]), yaw_limit)
    offset = [nut["pos"][i] - state["eef_pos"][i] for i in range(3)]
    rotated = [
        cos(turn) * offset[0] - sin(turn) * offset[1],
        sin(turn) * offset[0] + cos(turn) * offset[1],
        offset[2],
    ]
    target_eef = [target[i] - rotated[i] for i in range(3)]
    servo_tick(
        sdk,
        target_eef,
        wrap_angle(state["eef_yaw"] + turn),
        True,
        speed,
    )


def run_policy(sdk):
    initial = sdk.get_state()
    if "nut" not in initial["objects"] or initial["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    resting_height = initial["objects"]["nut"]["pos"][2]
    clearance = max(resting_height + 0.13, initial["goal_pos"][2] + 0.10)
    grasped = False
    configurations = [
        (0.055, 0.001, pi / 2.0),
        (0.052, -0.002, 0.0),
        (0.060, 0.002, pi / 2.0),
    ]

    for radius, height, orientation in configurations:
        if remaining_steps(sdk) < 170:
            break
        grip(sdk, "open", 5, 155)
        state = sdk.get_state()
        target, yaw = handle_pose(state, radius, height, orientation)
        hover = [target[0], target[1], target[2] + 0.065]
        hovered = pose_servo(sdk, hover, yaw, False, 65, 155, 0.006)
        if not hovered:
            state = sdk.get_state()
            if (
                distance(hover, state["eef_pos"]) > 0.012
                or abs(wrap_angle(yaw - state["eef_yaw"])) > 0.18
            ):
                continue

        reached = False
        for iteration in range(40):
            if remaining_steps(sdk) <= 135:
                break
            state = sdk.get_state()
            target, yaw = handle_pose(state, radius, height, orientation)
            if (
                distance(target, state["eef_pos"]) < 0.0035
                and abs(wrap_angle(yaw - state["eef_yaw"])) < 0.10
            ):
                reached = True
                break
            servo_tick(sdk, target, yaw, False, 0.35)

        if not reached:
            state = sdk.get_state()
            retreat = list(state["eef_pos"])
            retreat[2] += 0.055
            pose_servo(
                sdk, retreat, state["eef_yaw"], False, 15, 125, 0.008
            )
            continue

        before_height = sdk.get_state()["objects"]["nut"]["pos"][2]
        grip(sdk, "close", 12, 115)
        state = sdk.get_state()
        lift = list(state["eef_pos"])
        lift[2] += 0.045
        pose_servo(sdk, lift, state["eef_yaw"], True, 24, 100, 0.005)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        rise = nut["pos"][2] - before_height
        if rise > 0.027 and state["gripper_width"] > 0.012 and nut_tilt(nut) < 0.22:
            grasped = True
            break

        if rise > 0.010:
            lower = list(state["eef_pos"])
            lower[2] -= max(0.0, nut["pos"][2] - resting_height - 0.002)
            pose_servo(sdk, lower, state["eef_yaw"], True, 18, 90, 0.006)
        grip(sdk, "open", 7, 85)
        state = sdk.get_state()
        retreat = list(state["eef_pos"])
        retreat[2] += 0.060
        pose_servo(sdk, retreat, state["eef_yaw"], False, 18, 80, 0.008)

    if not grasped:
        return {
            "status": "failed",
            "reason": "No stable grasp",
            "steps_used": sdk.get_state()["steps_used"],
        }

    for iteration in range(35):
        if remaining_steps(sdk) <= 85:
            break
        state = sdk.get_state()
        position = state["objects"]["nut"]["pos"]
        if position[2] >= clearance - 0.006:
            break
        nut_tick(sdk, [position[0], position[1], clearance], 0.16)

    aligned = False
    for iteration in range(95):
        if remaining_steps(sdk) <= 55:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        lateral = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
        if (
            lateral < 0.003
            and abs(square_yaw_error(nut["yaw"])) < 0.035
            and abs(nut["pos"][2] - clearance) < 0.008
        ):
            aligned = True
            break
        nut_tick(sdk, [goal[0], goal[1], clearance], 0.35, 0.80)

    if not aligned:
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        aligned = (
            hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.005
            and abs(square_yaw_error(nut["yaw"])) < 0.055
            and nut["pos"][2] > resting_height + 0.075
        )
    if not aligned:
        return {
            "status": "failed",
            "reason": "Alignment incomplete",
            "steps_used": sdk.get_state()["steps_used"],
        }

    stalled = 0
    recoveries = 0
    for iteration in range(180):
        if remaining_steps(sdk) <= 20:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        position = list(nut["pos"])
        lateral = hypot(position[0] - goal[0], position[1] - goal[1])
        yaw_error = abs(square_yaw_error(nut["yaw"]))
        if position[2] < resting_height + 0.009 and lateral < 0.005:
            break

        descent = 0.035 if position[2] > resting_height + 0.075 else 0.012
        target_height = max(resting_height + 0.001, position[2] - descent)
        descending = lateral < 0.0035 and yaw_error < 0.04
        if not descending:
            target_height = position[2]

        nut_tick(sdk, [goal[0], goal[1], target_height], 0.10, 0.35)
        after = sdk.get_state()
        if after["steps_used"] <= state["steps_used"]:
            break
        progress = position[2] - after["objects"]["nut"]["pos"][2]
        if descending and progress < 0.0002:
            stalled += 1
        else:
            stalled = 0

        if stalled >= 10 and recoveries < 2 and remaining_steps(sdk) > 45:
            state = sdk.get_state()
            raised = state["objects"]["nut"]["pos"][2] + 0.012
            for correction in range(10):
                if remaining_steps(sdk) <= 30:
                    break
                state = sdk.get_state()
                goal = state["goal_pos"]
                nut_tick(sdk, [goal[0], goal[1], raised], 0.16, 0.30)
            stalled = 0
            recoveries += 1

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    lateral = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    if (
        lateral > 0.006
        or nut["pos"][2] > resting_height + 0.015
        or nut_tilt(nut) > 0.20
    ):
        return {
            "status": "failed",
            "reason": "Insertion incomplete",
            "steps_used": state["steps_used"],
        }

    grip(sdk, "open", 8)
    state = sdk.get_state()
    retreat = list(state["eef_pos"])
    retreat[2] += 0.075
    pose_servo(sdk, retreat, state["eef_yaw"], False, 12, 0, 0.008)

    final = sdk.get_state()
    nut = final["objects"]["nut"]
    goal = final["goal_pos"]
    lateral = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    seated = (
        lateral < 0.010
        and abs(nut["pos"][2] - resting_height) < 0.016
        and abs(square_yaw_error(nut["yaw"])) < 0.10
        and nut_tilt(nut) < 0.20
    )
    return {
        "status": "done" if seated else "failed",
        "verification": "Released nut center, height, yaw and tilt checked",
        "lateral_error_m": lateral,
        "steps_used": final["steps_used"],
    }
