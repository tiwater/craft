def wrap_angle(angle):
    return atan2(sin(angle), cos(angle))


def square_error(angle):
    return 0.25 * atan2(sin(4.0 * angle), cos(4.0 * angle))


def budget(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def clamp(value, limit):
    return max(-limit, min(limit, value))


def grip(sdk, action, steps, reserve=0):
    count = min(steps, max(0, budget(sdk) - reserve))
    if count > 0:
        sdk.set_gripper(action, steps=count)


def move(sdk, target, yaw, action, steps, tol=0.004, reserve=0):
    # Control the complete downward orientation, including roll and pitch.
    a = cos(0.5 * yaw)
    b = sin(0.5 * yaw)
    count = min(steps, max(0, budget(sdk) - reserve))
    stable = 0
    for tick in range(count):
        state = sdk.get_state()
        pos = state["eef_pos"]
        quat = state["eef_quat"]
        dx = target[0] - pos[0]
        dy = target[1] - pos[1]
        dz = target[2] - pos[2]

        rx = a * quat[3] - b * quat[2]
        ry = b * quat[3] + a * quat[2]
        rz = b * quat[0] - a * quat[1]
        rw = a * quat[0] + b * quat[1]
        if rw < 0.0:
            rx = -rx
            ry = -ry
            rz = -rz
            rw = -rw
        norm = sqrt(rx * rx + ry * ry + rz * rz)
        angle = 2.0 * atan2(norm, max(0.0, rw))
        scale = 1.4 * angle / norm if norm > 0.000001 else 0.0

        error = sqrt(dx * dx + dy * dy + dz * dz)
        if error < tol and angle < 0.055:
            stable += 1
            if stable >= 2:
                return True
        else:
            stable = 0

        sdk.step_raw(
            [
                clamp(16.0 * dx, 0.75),
                clamp(16.0 * dy, 0.75),
                clamp(16.0 * dz, 0.65),
                clamp(scale * rx, 0.65),
                clamp(scale * ry, 0.65),
                clamp(scale * rz, 0.65),
                1.0 if action == "close" else -1.0,
            ],
            steps=1,
        )
    return False


def xy_error(state):
    pos = state["objects"]["nut"]["pos"]
    goal = state["goal_pos"]
    return hypot(pos[0] - goal[0], pos[1] - goal[1])


def move_nut(sdk, target, steps, reserve=24):
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    pos = nut["pos"]
    eef = state["eef_pos"]
    angle = -square_error(nut["yaw"])
    dx = eef[0] - pos[0]
    dy = eef[1] - pos[1]
    return move(
        sdk,
        [
            target[0] + cos(angle) * dx - sin(angle) * dy,
            target[1] + sin(angle) * dx + cos(angle) * dy,
            target[2] + eef[2] - pos[2],
        ],
        wrap_angle(state["eef_yaw"] + angle),
        "close",
        steps,
        0.003,
        reserve,
    )


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    resting_height = state["objects"]["nut"]["pos"][2]
    grasped = False

    for offset in [0.055, 0.045]:
        if budget(sdk) < 180:
            break
        grip(sdk, "open", 8, 160)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        pos = nut["pos"]
        handle_yaw = nut["yaw"]
        yaw = state["eef_yaw"] + 0.5 * atan2(
            sin(2.0 * (handle_yaw - state["eef_yaw"])),
            cos(2.0 * (handle_yaw - state["eef_yaw"])),
        )

        move(
            sdk,
            [
                pos[0] + offset * cos(handle_yaw),
                pos[1] + offset * sin(handle_yaw),
                pos[2] + 0.10,
            ],
            yaw,
            "open",
            45,
            0.005,
            145,
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        pos = nut["pos"]
        handle_yaw = nut["yaw"]
        target = [
            pos[0] + offset * cos(handle_yaw),
            pos[1] + offset * sin(handle_yaw),
            pos[2] - 0.008,
        ]
        reached = move(sdk, target, yaw, "open", 35, 0.004, 125)
        state = sdk.get_state()
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - target[0], eef[1] - target[1]) > 0.009
            or abs(eef[2] - target[2]) > 0.010
        ):
            continue

        baseline = state["objects"]["nut"]["pos"][2]
        grip(sdk, "close", 14, 110)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move(
            sdk,
            [eef[0], eef[1], eef[2] + 0.11],
            yaw,
            "close",
            32,
            0.005,
            95,
        )
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        grasped = (
            nut["pos"][2] > baseline + 0.065
            and 0.012 < state["gripper_width"] < 0.070
        )
        if grasped:
            break
        grip(sdk, "open", 8, 95)

    if not grasped:
        return {
            "status": "failed",
            "reason": "Lifted grasp not verified",
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    goal = state["goal_pos"]
    clearance = max(resting_height + 0.15, goal[2] + 0.13)
    pos = state["objects"]["nut"]["pos"]
    move_nut(sdk, [pos[0], pos[1], clearance], 24, 110)

    for attempt in range(3):
        state = sdk.get_state()
        goal = state["goal_pos"]
        move_nut(
            sdk,
            [goal[0], goal[1], clearance],
            50 if attempt == 0 else 15,
            80,
        )
        state = sdk.get_state()
        if state["gripper_width"] < 0.010:
            return {"status": "failed", "reason": "Grasp lost during transport"}
        if (
            xy_error(state) < 0.003
            and abs(square_error(state["objects"]["nut"]["yaw"])) < 0.045
        ):
            break

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    quat = nut["quat"]
    upright = 1.0 - 2.0 * (quat[0] * quat[0] + quat[1] * quat[1])
    if (
        xy_error(state) > 0.006
        or abs(square_error(nut["yaw"])) > 0.08
        or upright < 0.94
    ):
        return {
            "status": "failed",
            "reason": "Nut pose unsuitable for clean insertion",
            "xy_error_m": xy_error(state),
        }

    stalls = 0
    for attempt in range(18):
        if budget(sdk) <= 27:
            break
        state = sdk.get_state()
        pos = state["objects"]["nut"]["pos"]
        goal = state["goal_pos"]
        if xy_error(state) > 0.012 or state["gripper_width"] < 0.010:
            return {"status": "failed", "reason": "Grasp or alignment lost"}
        if pos[2] <= resting_height + 0.008 and xy_error(state) < 0.006:
            break

        previous_z = pos[2]
        descent = 0.020 if pos[2] > resting_height + 0.08 else 0.010
        move_nut(
            sdk,
            [goal[0], goal[1], max(resting_height, pos[2] - descent)],
            12,
            27,
        )
        state = sdk.get_state()
        progress = previous_z - state["objects"]["nut"]["pos"][2]
        stalls = stalls + 1 if progress < 0.0015 else 0
        if stalls >= 2 and budget(sdk) > 65:
            pos = state["objects"]["nut"]["pos"]
            goal = state["goal_pos"]
            recovery_z = pos[2] + 0.018
            move_nut(sdk, [pos[0], pos[1], recovery_z], 12, 43)
            state = sdk.get_state()
            goal = state["goal_pos"]
            move_nut(sdk, [goal[0], goal[1], recovery_z], 12, 32)
            stalls = 0

    state = sdk.get_state()
    if (
        xy_error(state) > 0.007
        or state["objects"]["nut"]["pos"][2] > resting_height + 0.012
    ):
        return {
            "status": "failed",
            "reason": "Seating height not reached",
            "xy_error_m": xy_error(state),
            "steps_used": state["steps_used"],
        }

    grip(sdk, "open", 10, 16)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        sdk,
        [eef[0], eef[1], eef[2] + 0.08],
        state["eef_yaw"],
        "open",
        24,
        0.008,
        2,
    )
    grip(sdk, "open", 2)
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    quat = nut["quat"]
    verified = (
        xy_error(state) < 0.008
        and abs(nut["pos"][2] - resting_height) < 0.012
        and abs(square_error(nut["yaw"])) < 0.10
        and 1.0 - 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]) > 0.96
        and state["eef_pos"][2] > nut["pos"][2] + 0.045
        and state["gripper_width"] > 0.05
    )
    return {
        "status": "done" if verified else "failed",
        "reason": "Released and seated" if verified else "Final verification failed",
        "xy_error_m": xy_error(state),
        "steps_used": state["steps_used"],
    }
