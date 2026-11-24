def angle_wrap(angle):
    return (angle + pi) % (2.0 * pi) - pi


def square_yaw_error(angle):
    return (angle + pi / 4.0) % (pi / 2.0) - pi / 4.0


def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def clamp(value, limit):
    return max(-limit, min(limit, value))


def move_budgeted(sdk, position, yaw, gripper, steps, tolerance):
    budget = min(steps, remaining_steps(sdk))
    if budget <= 0:
        return False
    result = sdk.move_to(
        position, yaw=yaw, gripper=gripper,
        max_steps=budget, tol=tolerance
    )
    return result["reached"]


def grip_budgeted(sdk, action, steps):
    budget = min(steps, remaining_steps(sdk))
    if budget > 0:
        sdk.set_gripper(action, steps=budget)


def rotate_gripper(sdk, target_yaw, gripper, steps):
    state = sdk.get_state()
    anchor = list(state["eef_pos"])
    grip = 1.0 if gripper == "close" else -1.0
    for tick in range(min(steps, remaining_steps(sdk))):
        state = sdk.get_state()
        error = angle_wrap(target_yaw - state["eef_yaw"])
        if abs(error) < 0.025:
            break
        position = state["eef_pos"]
        sdk.step_raw(
            [
                clamp(12.0 * (anchor[0] - position[0]), 0.3),
                clamp(12.0 * (anchor[1] - position[1]), 0.3),
                clamp(12.0 * (anchor[2] - position[2]), 0.3),
                0.0, 0.0, clamp(1.5 * error, 0.35), grip,
            ],
            steps=1,
        )


def servo_nut(sdk, target, steps, reserve, tolerance):
    stable = 0
    for tick in range(steps):
        if remaining_steps(sdk) <= reserve:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        position = nut["pos"]
        error = [target[i] - position[i] for i in range(3)]
        yaw_error = square_yaw_error(nut["yaw"])
        if (
            hypot(error[0], error[1]) < tolerance
            and abs(error[2]) < tolerance
            and abs(yaw_error) < 0.035
        ):
            stable += 1
            if stable >= 3:
                return True
        else:
            stable = 0

        # Rotate about the live hole center, compensating for the handle offset.
        eef = state["eef_pos"]
        offset_x = eef[0] - position[0]
        offset_y = eef[1] - position[1]
        rotation = clamp(-0.45 * yaw_error, 0.06)
        correction_x = (
            cos(rotation) * offset_x - sin(rotation) * offset_y - offset_x
        )
        correction_y = (
            sin(rotation) * offset_x + cos(rotation) * offset_y - offset_y
        )
        sdk.step_raw(
            [
                clamp(10.0 * error[0] + correction_x / 0.05, 0.45),
                clamp(10.0 * error[1] + correction_y / 0.05, 0.45),
                clamp(10.0 * error[2], 0.35),
                0.0, 0.0, rotation / 0.5, 1.0,
            ],
            steps=1,
        )
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    return (
        hypot(nut["pos"][0] - target[0], nut["pos"][1] - target[1])
        < tolerance
        and abs(nut["pos"][2] - target[2]) < tolerance
        and abs(square_yaw_error(nut["yaw"])) < 0.05
    )


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    rest_z = state["objects"]["nut"]["pos"][2]
    held = False

    for attempt in range(2):
        if remaining_steps(sdk) < 180:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        desired_yaw = nut["yaw"] + (pi / 2.0 if attempt == 0 else 0.0)
        grasp_yaw = angle_wrap(
            state["eef_yaw"]
            + (desired_yaw - state["eef_yaw"] + pi / 2.0) % pi
            - pi / 2.0
        )
        clearance = max(state["eef_pos"][2], nut["pos"][2] + 0.10)
        move_budgeted(
            sdk,
            [state["eef_pos"][0], state["eef_pos"][1], clearance],
            state["eef_yaw"], "open", 16, 0.008,
        )
        grip_budgeted(sdk, "open", 6)
        rotate_gripper(sdk, grasp_yaw, "open", 24)

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        handle = [
            nut["pos"][0] + 0.055 * cos(nut["yaw"]),
            nut["pos"][1] + 0.055 * sin(nut["yaw"]),
            nut["pos"][2],
        ]
        move_budgeted(
            sdk, [handle[0], handle[1], clearance],
            grasp_yaw, "open", 28, 0.004,
        )
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        handle = [
            nut["pos"][0] + 0.055 * cos(nut["yaw"]),
            nut["pos"][1] + 0.055 * sin(nut["yaw"]),
            nut["pos"][2],
        ]
        move_budgeted(sdk, handle, grasp_yaw, "open", 28, 0.003)
        state = sdk.get_state()
        if (
            hypot(
                state["eef_pos"][0] - handle[0],
                state["eef_pos"][1] - handle[1],
            ) > 0.012
            or abs(state["eef_pos"][2] - handle[2]) > 0.018
        ):
            continue

        before_z = state["objects"]["nut"]["pos"][2]
        grip_budgeted(sdk, "close", 12)
        state = sdk.get_state()
        lift = list(state["eef_pos"])
        lift[2] += 0.075
        move_budgeted(sdk, lift, grasp_yaw, "close", 24, 0.005)
        state = sdk.get_state()
        held = (
            state["objects"]["nut"]["pos"][2] > before_z + 0.04
            and state["gripper_width"] > 0.012
        )
        if held:
            break

    if not held:
        return {"status": "failed", "reason": "Could not lift the nut"}

    state = sdk.get_state()
    goal = state["goal_pos"]
    travel_z = max(rest_z + 0.14, goal[2] + 0.10)
    nut_position = state["objects"]["nut"]["pos"]
    eef = state["eef_pos"]
    move_budgeted(
        sdk, [eef[0], eef[1], eef[2] + travel_z - nut_position[2]],
        state["eef_yaw"], "close", 22, 0.004,
    )

    # Explicitly control yaw; position-only convergence must not end rotation.
    state = sdk.get_state()
    nut_position = state["objects"]["nut"]["pos"]
    servo_nut(
        sdk, [nut_position[0], nut_position[1], travel_z],
        32, 115, 0.004,
    )

    state = sdk.get_state()
    nut_position = state["objects"]["nut"]["pos"]
    eef = state["eef_pos"]
    goal = state["goal_pos"]
    move_budgeted(
        sdk,
        [
            eef[0] + goal[0] - nut_position[0],
            eef[1] + goal[1] - nut_position[1],
            eef[2] + travel_z - nut_position[2],
        ],
        state["eef_yaw"], "close", 32, 0.004,
    )
    servo_nut(sdk, [goal[0], goal[1], travel_z], 32, 80, 0.0025)

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    if (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) > 0.005
        or abs(square_yaw_error(nut["yaw"])) > 0.06
    ):
        return {
            "status": "failed",
            "reason": "Could not align above peg",
            "steps_used": state["steps_used"],
        }

    # Descend in small increments, correcting the actual hole pose each tick.
    target_z = nut["pos"][2]
    for tick in range(150):
        if remaining_steps(sdk) <= 28:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        lateral_error = hypot(
            nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]
        )
        yaw_error = abs(square_yaw_error(nut["yaw"]))
        if lateral_error < 0.0035 and yaw_error < 0.045:
            target_z = max(
                rest_z + 0.001,
                min(target_z, nut["pos"][2]) - 0.0025,
            )
        else:
            target_z = max(target_z, nut["pos"][2])
        servo_nut(sdk, [goal[0], goal[1], target_z], 1, 28, 0.002)
        if (
            nut["pos"][2] < rest_z + 0.012
            and lateral_error < 0.005
            and yaw_error < 0.06
        ):
            servo_nut(
                sdk, [goal[0], goal[1], rest_z + 0.001],
                8, 24, 0.002,
            )
            break

    state = sdk.get_state()
    nut = state["objects"]["nut"]["pos"]
    goal = state["goal_pos"]
    if (
        hypot(nut[0] - goal[0], nut[1] - goal[1]) > 0.007
        or abs(nut[2] - rest_z) > 0.018
        or remaining_steps(sdk) < 20
    ):
        return {
            "status": "failed",
            "reason": "Nut did not seat on peg",
            "steps_used": state["steps_used"],
        }

    grip_budgeted(sdk, "open", 10)
    state = sdk.get_state()
    retreat = list(state["eef_pos"])
    retreat[2] += 0.09
    move_budgeted(sdk, retreat, state["eef_yaw"], "open", 18, 0.008)

    state = sdk.get_state()
    nut = state["objects"]["nut"]["pos"]
    goal = state["goal_pos"]
    error = hypot(nut[0] - goal[0], nut[1] - goal[1])
    seated = error < 0.012 and abs(nut[2] - rest_z) < 0.018
    return {
        "status": "done" if seated else "failed",
        "reason": "Released and seated" if seated else "Seating verification failed",
        "center_error_m": error,
        "steps_used": state["steps_used"],
    }
