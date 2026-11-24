def angle_difference(target, current):
    return atan2(sin(target - current), cos(target - current))


def square_alignment(yaw):
    return min(
        [angle_difference(k * pi / 2.0, yaw) for k in range(-2, 3)],
        key=abs,
    )


def clamp(value, limit):
    return max(-limit, min(limit, value))


def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def bounded_move(sdk, position, yaw, gripper, steps, tolerance=0.003):
    budget = min(steps, remaining_steps(sdk))
    if budget > 0:
        return sdk.move_to(
            position,
            yaw=yaw,
            gripper=gripper,
            max_steps=budget,
            tol=tolerance,
        )
    return {"reached": False}


def actuate(sdk, action, steps):
    budget = min(steps, remaining_steps(sdk))
    if budget > 0:
        return sdk.set_gripper(action, steps=budget)
    return {}


def nut_normal(nut):
    q = nut["quat"]
    return [
        2.0 * (q[0] * q[2] + q[1] * q[3]),
        2.0 * (q[1] * q[2] - q[0] * q[3]),
        1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1]),
    ]


def seated(state, resting_z):
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    return (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.009
        and abs(nut["pos"][2] - resting_z) < 0.01
        and abs(square_alignment(nut["yaw"])) < 0.1
        and nut_normal(nut)[2] > 0.97
    )


def servo_tick(sdk, target, ticks=2, speed=0.65):
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    normal = nut_normal(nut)
    error = [target[i] - nut["pos"][i] for i in range(3)]

    rotation = [
        clamp(2.4 * normal[1], 0.24),
        clamp(-2.4 * normal[0], 0.24),
        clamp(2.2 * square_alignment(nut["yaw"]), 0.32),
    ]

    # Compensate the motion of the nut center about the handle grasp.
    lever = [
        nut["pos"][i] - state["eef_pos"][i]
        for i in range(3)
    ]
    rotational_motion = [
        rotation[1] * lever[2] - rotation[2] * lever[1],
        rotation[2] * lever[0] - rotation[0] * lever[2],
        rotation[0] * lever[1] - rotation[1] * lever[0],
    ]
    translation = [
        clamp(38.0 * error[i] - 10.0 * rotational_motion[i], speed)
        for i in range(3)
    ]
    budget = min(ticks, remaining_steps(sdk))
    if budget > 0:
        sdk.step_raw(
            translation + rotation + [1.0],
            steps=budget,
        )


def transport_nut(sdk, target, steps, reserve):
    start = sdk.get_state()["steps_used"]
    while remaining_steps(sdk) > reserve:
        state = sdk.get_state()
        elapsed = state["steps_used"] - start
        if elapsed >= steps or state["gripper_width"] < 0.01:
            return False
        nut = state["objects"]["nut"]
        error = [target[i] - nut["pos"][i] for i in range(3)]
        normal = nut_normal(nut)
        if (
            max(abs(value) for value in error) < 0.004
            and abs(square_alignment(nut["yaw"])) < 0.045
            and hypot(normal[0], normal[1]) < 0.075
        ):
            return True
        servo_tick(
            sdk,
            target,
            min(2, steps - elapsed, remaining_steps(sdk) - reserve),
            0.85,
        )
    return False


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    resting_z = state["objects"]["nut"]["pos"][2]
    grasped = False

    for handle_offset, depth in [(0.060, -0.006), (0.052, -0.009)]:
        if remaining_steps(sdk) < 165:
            break

        actuate(sdk, "open", 5)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        approach_z = max(nut["pos"][2] + 0.085, state["eef_pos"][2])

        if state["eef_pos"][2] < approach_z - 0.008:
            bounded_move(
                sdk,
                [state["eef_pos"][0], state["eef_pos"][1], approach_z],
                state["eef_yaw"],
                "open",
                12,
                0.006,
            )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        grasp_yaw = state["eef_yaw"] + angle_difference(
            yaw + pi / 2.0, state["eef_yaw"]
        )
        bounded_move(
            sdk,
            [
                nut["pos"][0] + handle_offset * cos(yaw),
                nut["pos"][1] + handle_offset * sin(yaw),
                approach_z,
            ],
            grasp_yaw,
            "open",
            30,
            0.003,
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        grasp_yaw = state["eef_yaw"] + angle_difference(
            yaw + pi / 2.0, state["eef_yaw"]
        )
        bounded_move(
            sdk,
            [
                nut["pos"][0] + handle_offset * cos(yaw),
                nut["pos"][1] + handle_offset * sin(yaw),
                nut["pos"][2] + depth,
            ],
            grasp_yaw,
            "open",
            25,
            0.002,
        )

        baseline_z = sdk.get_state()["objects"]["nut"]["pos"][2]
        actuate(sdk, "close", 14)
        state = sdk.get_state()
        bounded_move(
            sdk,
            [
                state["eef_pos"][0],
                state["eef_pos"][1],
                state["eef_pos"][2] + 0.075,
            ],
            state["eef_yaw"],
            "close",
            22,
            0.004,
        )
        state = sdk.get_state()
        if (
            state["objects"]["nut"]["pos"][2] > baseline_z + 0.035
            and state["gripper_width"] > 0.012
        ):
            grasped = True
            break
        actuate(sdk, "open", 5)

    if not grasped:
        return {
            "status": "failed",
            "reason": "Handle grasp did not lift nut",
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    goal = state["goal_pos"]
    clearance_z = max(resting_z + 0.13, goal[2] + 0.12)
    nut = state["objects"]["nut"]
    transport_nut(
        sdk,
        [nut["pos"][0], nut["pos"][1], clearance_z],
        32,
        130,
    )

    state = sdk.get_state()
    if state["objects"]["nut"]["pos"][2] < clearance_z - 0.015:
        transport_nut(
            sdk,
            [
                state["objects"]["nut"]["pos"][0],
                state["objects"]["nut"]["pos"][1],
                clearance_z,
            ],
            18,
            112,
        )

    state = sdk.get_state()
    goal = state["goal_pos"]
    transport_nut(sdk, [goal[0], goal[1], clearance_z], 58, 90)

    phase = "align"
    released = False
    recovery_height = clearance_z
    recoveries = 0
    checkpoint = sdk.get_state()["steps_used"]
    previous_z = sdk.get_state()["objects"]["nut"]["pos"][2]
    stalled = 0

    while remaining_steps(sdk) > 32:
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        position = nut["pos"]
        normal = nut_normal(nut)
        lateral_error = hypot(goal[0] - position[0], goal[1] - position[1])
        yaw_error = abs(square_alignment(nut["yaw"]))
        tilt_error = hypot(normal[0], normal[1])

        if seated(state, resting_z):
            break
        if state["gripper_width"] < 0.01:
            break

        if phase == "recover":
            target_z = recovery_height
            if position[2] >= recovery_height - 0.005:
                phase = "align"

        if phase == "align":
            target_z = max(position[2], clearance_z)
            if lateral_error < 0.004 and yaw_error < 0.055 and tilt_error < 0.085:
                phase = "insert"
                checkpoint = state["steps_used"]
                previous_z = position[2]
                stalled = 0

        if phase == "insert":
            if (
                position[2] < resting_z + 0.030
                and lateral_error < 0.005
                and yaw_error < 0.065
                and tilt_error < 0.10
            ):
                actuate(sdk, "open", 9)
                released = True
                break

            if lateral_error < 0.0045 and yaw_error < 0.06 and tilt_error < 0.09:
                target_z = max(resting_z + 0.002, position[2] - 0.025)
            elif lateral_error < 0.009 and yaw_error < 0.10 and tilt_error < 0.15:
                target_z = position[2]
            else:
                target_z = position[2] + 0.008

            if state["steps_used"] - checkpoint >= 16:
                if previous_z - position[2] < 0.002:
                    stalled += 1
                else:
                    stalled = 0
                previous_z = position[2]
                checkpoint = state["steps_used"]

            if stalled >= 2 and recoveries < 2 and remaining_steps(sdk) > 70:
                recoveries += 1
                phase = "recover"
                recovery_height = position[2] + 0.022
                target_z = recovery_height
                stalled = 0

        servo_tick(
            sdk,
            [goal[0], goal[1], target_z],
            min(2, remaining_steps(sdk) - 32),
            0.65 if phase != "insert" else 0.45,
        )

    state = sdk.get_state()
    if not released:
        if not seated(state, resting_z):
            return {
                "status": "failed",
                "reason": "Nut did not reach verified seating depth",
                "steps_used": state["steps_used"],
            }
        actuate(sdk, "open", 9)

    state = sdk.get_state()
    bounded_move(
        sdk,
        [
            state["eef_pos"][0],
            state["eef_pos"][1],
            state["eef_pos"][2] + 0.09,
        ],
        state["eef_yaw"],
        "open",
        17,
        0.006,
    )
    actuate(sdk, "open", min(12, remaining_steps(sdk)))
    state = sdk.get_state()
    success = seated(state, resting_z) and state["gripper_width"] > 0.055
    return {
        "status": "done" if success else "failed",
        "reason": "Verified released seated nut" if success else "Final seating check failed",
        "steps_used": state["steps_used"],
    }
