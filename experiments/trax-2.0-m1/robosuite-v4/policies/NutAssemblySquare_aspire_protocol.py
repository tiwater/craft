def wrap_angle(angle):
    return atan2(sin(angle), cos(angle))


def square_error(yaw):
    return atan2(sin(-4.0 * yaw), cos(-4.0 * yaw)) / 4.0


def clamp(value, limit):
    return max(-limit, min(limit, value))


def remaining(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def move(sdk, target, yaw, action, steps, tolerance=0.004):
    budget = remaining(sdk)
    if budget > 0:
        return sdk.move_to(
            target,
            yaw=wrap_angle(yaw),
            gripper=action,
            max_steps=min(steps, budget),
            tol=tolerance,
        )
    return {"reached": False}


def grip(sdk, action, steps):
    budget = remaining(sdk)
    if budget > 0:
        sdk.set_gripper(action, steps=min(steps, budget))


def handle_target(nut, height):
    x, y, z, w = nut["quat"]
    return [
        nut["pos"][0] + 0.05 * (1.0 - 2.0 * (y * y + z * z)),
        nut["pos"][1] + 0.05 * 2.0 * (x * y + w * z),
        nut["pos"][2] + 0.05 * 2.0 * (x * z - w * y) + height,
    ]


def nut_tilt(nut):
    x, y, z, w = nut["quat"]
    horizontal = hypot(
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
    )
    vertical = 1.0 - 2.0 * (x * x + y * y)
    return atan2(horizontal, vertical)


def level_error(nut, target_yaw):
    x, y, z, w = nut["quat"]
    target_z = sin(target_yaw / 2.0)
    target_w = cos(target_yaw / 2.0)
    error = [
        -target_w * x + target_z * y,
        -target_w * y - target_z * x,
        -target_w * z + target_z * w,
    ]
    scalar = target_w * w + target_z * z
    if scalar < 0.0:
        error = [-value for value in error]
        scalar = -scalar
    magnitude = sqrt(sum(value * value for value in error))
    if magnitude < 0.000001:
        return [0.0, 0.0, 0.0]
    scale = 2.0 * atan2(magnitude, scalar) / magnitude
    return [value * scale for value in error]


def servo_nut(sdk, target, target_yaw, speed=0.012):
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    offset = [
        nut["pos"][index] - state["eef_pos"][index]
        for index in range(3)
    ]

    # Correct all three rotation axes. Yaw alone leaves a tilted hole
    # perched on the peg even when its projected center is accurate.
    rotation = level_error(nut, target_yaw)
    magnitude = sqrt(sum(value * value for value in rotation))
    scale = min(0.45, 0.10 / max(magnitude, 0.000001))
    rotation = [value * scale for value in rotation]
    displacement = [
        0.55 * (target[index] - nut["pos"][index])
        for index in range(3)
    ]
    magnitude = sqrt(sum(value * value for value in displacement))
    scale = min(1.0, speed / max(magnitude, 0.000001))
    displacement = [value * scale for value in displacement]

    # Compensate for rotating about the hand rather than the nut center.
    induced = [
        rotation[1] * offset[2] - rotation[2] * offset[1],
        rotation[2] * offset[0] - rotation[0] * offset[2],
        rotation[0] * offset[1] - rotation[1] * offset[0],
    ]
    action = [
        clamp((displacement[index] - induced[index]) / 0.05, 0.6)
        for index in range(3)
    ]
    action += [clamp(value / 0.5, 0.3) for value in rotation]
    action += [1.0]
    if remaining(sdk) > 0:
        sdk.step_raw(action, steps=1)


def seated(state, resting_height):
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    return (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.006
        and abs(nut["pos"][2] - resting_height) < 0.009
        and nut_tilt(nut) < 0.12
        and abs(square_error(nut["yaw"])) < 0.10
    )


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or goal"}

    resting_height = state["objects"]["nut"]["pos"][2]
    clearance = max(resting_height + 0.14, state["goal_pos"][2] + 0.11)
    grasped = False

    for attempt in range(2):
        if remaining(sdk) < 150:
            break
        grip(sdk, "open", 6)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        desired = nut["yaw"] + pi / 2.0
        yaw = state["eef_yaw"] + atan2(
            sin(2.0 * (desired - state["eef_yaw"])),
            cos(2.0 * (desired - state["eef_yaw"])),
        ) / 2.0

        move(sdk, handle_target(nut, 0.075), yaw, "open", 30, 0.005)
        nut = sdk.get_state()["objects"]["nut"]
        move(sdk, handle_target(nut, 0.0), yaw, "open", 25, 0.003)

        state = sdk.get_state()
        target = handle_target(state["objects"]["nut"], 0.0)
        error = sqrt(sum(
            (state["eef_pos"][index] - target[index]) ** 2
            for index in range(3)
        ))
        if error > 0.006:
            move(sdk, target, yaw, "open", 12, 0.003)

        before = list(sdk.get_state()["objects"]["nut"]["pos"])
        grip(sdk, "close", 14)
        state = sdk.get_state()
        lift = list(state["eef_pos"])
        lift[2] += 0.065
        move(sdk, lift, state["eef_yaw"], "close", 22, 0.004)
        state = sdk.get_state()
        grasped = (
            state["objects"]["nut"]["pos"][2] > before[2] + 0.035
            and 0.012 < state["gripper_width"] < 0.075
        )
        if grasped:
            break

    if not grasped:
        return {
            "status": "failed",
            "reason": "No verified lift",
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    target_yaw = nut["yaw"] + square_error(nut["yaw"])
    lift_center = list(nut["pos"])
    lift_center[2] = clearance

    for tick in range(35):
        if remaining(sdk) <= 100:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        if nut["pos"][2] >= clearance - 0.006 and nut_tilt(nut) < 0.10:
            break
        servo_nut(sdk, lift_center, target_yaw)

    for tick in range(65):
        if remaining(sdk) <= 85:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        lateral = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
        if (
            lateral < 0.003
            and abs(nut["pos"][2] - clearance) < 0.008
            and nut_tilt(nut) < 0.08
            and abs(wrap_angle(target_yaw - nut["yaw"])) < 0.055
        ):
            break
        servo_nut(sdk, [goal[0], goal[1], clearance], target_yaw, 0.016)

    state = sdk.get_state()
    target_height = state["objects"]["nut"]["pos"][2]
    lowest_height = target_height
    stall_ticks = 0
    recoveries = 0
    recovery_ticks = 0
    recovery_height = target_height
    stable_ticks = 0

    for tick in range(400):
        if remaining(sdk) <= 30:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        position = nut["pos"]

        if state["gripper_width"] < 0.009:
            return {
                "status": "failed",
                "reason": "Grasp lost",
                "steps_used": state["steps_used"],
            }

        if seated(state, resting_height):
            stable_ticks += 1
            if stable_ticks >= 5:
                break
        else:
            stable_ticks = 0

        lateral = hypot(position[0] - goal[0], position[1] - goal[1])
        angle_error = abs(wrap_angle(target_yaw - nut["yaw"]))
        tilt = nut_tilt(nut)

        if recovery_ticks > 0:
            servo_nut(
                sdk,
                [goal[0], goal[1], recovery_height],
                target_yaw,
                0.006,
            )
            recovery_ticks -= 1
            if recovery_ticks == 0:
                target_height = sdk.get_state()["objects"]["nut"]["pos"][2]
                lowest_height = target_height
                stall_ticks = 0
            continue

        # Use actual control ticks and a bounded downward lead. This
        # avoids repeated move_to early exits at a stationary hover.
        if lateral < 0.0035 and angle_error < 0.065 and tilt < 0.09:
            target_height = max(
                resting_height,
                max(target_height - 0.003, position[2] - 0.012),
            )
        else:
            target_height = max(target_height, position[2] - 0.002)

        servo_nut(
            sdk,
            [goal[0], goal[1], target_height],
            target_yaw,
            0.005,
        )

        if position[2] < lowest_height - 0.002:
            lowest_height = position[2]
            stall_ticks = 0
        elif target_height < position[2] - 0.006:
            stall_ticks += 1
        else:
            stall_ticks = max(0, stall_ticks - 1)

        if (
            stall_ticks >= 22
            and recoveries < 2
            and remaining(sdk) > 65
            and position[2] > resting_height + 0.015
        ):
            recoveries += 1
            recovery_height = min(clearance, position[2] + 0.022)
            recovery_ticks = 14
            stall_ticks = 0

    state = sdk.get_state()
    if not seated(state, resting_height):
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        return {
            "status": "failed",
            "reason": "Seating not verified",
            "lateral_error_m": hypot(
                nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]
            ),
            "height_error_m": nut["pos"][2] - resting_height,
            "tilt_rad": nut_tilt(nut),
            "steps_used": state["steps_used"],
        }

    grip(sdk, "open", 10)
    state = sdk.get_state()
    retreat = list(state["eef_pos"])
    retreat[2] += 0.085
    move(sdk, retreat, state["eef_yaw"], "open", 16, 0.006)
    grip(sdk, "open", 4)

    state = sdk.get_state()
    verified = (
        seated(state, resting_height)
        and state["eef_pos"][2] - state["objects"]["nut"]["pos"][2] > 0.045
    )
    return {
        "status": "done" if verified else "failed",
        "reason": "Released seating verified" if verified else "Release verification failed",
        "steps_used": state["steps_used"],
    }
