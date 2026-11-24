def wrap_angle(angle):
    return atan2(sin(angle), cos(angle))


def square_yaw_error(yaw):
    return 0.25 * atan2(sin(-4.0 * yaw), cos(-4.0 * yaw))


def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def bounded_move(sdk, position, yaw, gripper, steps, tolerance):
    budget = remaining_steps(sdk)
    if budget <= 0:
        return {"reached": False}
    return sdk.move_to(
        position,
        yaw=wrap_angle(yaw),
        gripper=gripper,
        max_steps=min(steps, budget),
        tol=tolerance,
    )


def actuate_gripper(sdk, action, steps):
    budget = remaining_steps(sdk)
    if budget > 0:
        sdk.set_gripper(action, steps=min(steps, budget))


def handle_position(nut, radius):
    return [
        nut["pos"][0] + radius * cos(nut["yaw"]),
        nut["pos"][1] + radius * sin(nut["yaw"]),
        nut["pos"][2],
    ]


def nearest_grasp_yaw(current, target):
    return current + 0.5 * atan2(
        sin(2.0 * (target - current)), cos(2.0 * (target - current))
    )


def move_nut(sdk, target, steps, tolerance, yaw_limit):
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    offset = [nut["pos"][i] - state["eef_pos"][i] for i in range(3)]
    turn = max(-yaw_limit, min(yaw_limit, square_yaw_error(nut["yaw"])))
    rotated = [
        cos(turn) * offset[0] - sin(turn) * offset[1],
        sin(turn) * offset[0] + cos(turn) * offset[1],
        offset[2],
    ]
    return bounded_move(
        sdk,
        [target[i] - rotated[i] for i in range(3)],
        state["eef_yaw"] + turn,
        "close",
        steps,
        tolerance,
    )


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    resting_height = state["objects"]["nut"]["pos"][2]
    clearance = max(resting_height + 0.14, state["goal_pos"][2] + 0.10)
    grasped = False

    # Grasp farther out on the handle, away from the square body.
    configurations = [
        (0.060, 0.0, 0.0),
        (0.060, 0.5 * pi, 0.002),
        (0.055, 0.0, -0.003),
    ]

    for radius, yaw_offset, height_offset in configurations:
        if remaining_steps(sdk) < 150:
            break

        actuate_gripper(sdk, "open", 8)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        if state["eef_pos"][2] < nut["pos"][2] + 0.050:
            bounded_move(
                sdk,
                [
                    state["eef_pos"][0],
                    state["eef_pos"][1],
                    nut["pos"][2] + 0.065,
                ],
                state["eef_yaw"],
                "open",
                12,
                0.005,
            )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        grasp_yaw = nearest_grasp_yaw(
            state["eef_yaw"], nut["yaw"] + yaw_offset
        )
        handle = handle_position(nut, radius)
        bounded_move(
            sdk,
            [handle[0], handle[1], nut["pos"][2] + 0.060],
            grasp_yaw,
            "open",
            24,
            0.004,
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        handle = handle_position(nut, radius)
        handle[2] += height_offset
        grasp_yaw = nearest_grasp_yaw(
            state["eef_yaw"], nut["yaw"] + yaw_offset
        )
        bounded_move(sdk, handle, grasp_yaw, "open", 20, 0.002)

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        handle = handle_position(nut, radius)
        lateral = hypot(
            state["eef_pos"][0] - handle[0],
            state["eef_pos"][1] - handle[1],
        )
        if lateral > 0.006:
            handle[2] += height_offset
            bounded_move(sdk, handle, grasp_yaw, "open", 8, 0.002)
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            handle = handle_position(nut, radius)

        lateral = hypot(
            state["eef_pos"][0] - handle[0],
            state["eef_pos"][1] - handle[1],
        )
        height = state["eef_pos"][2] - nut["pos"][2]
        if lateral > 0.010 or height > 0.016 or height < -0.016:
            continue

        before_height = nut["pos"][2]
        actuate_gripper(sdk, "close", 12)
        state = sdk.get_state()
        lift_start = list(state["eef_pos"])
        bounded_move(
            sdk,
            [lift_start[0], lift_start[1], lift_start[2] + 0.060],
            state["eef_yaw"],
            "close",
            20,
            0.004,
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        if (
            nut["pos"][2] > before_height + 0.035
            and state["gripper_width"] > 0.012
        ):
            offset_before = [
                nut["pos"][i] - state["eef_pos"][i] for i in range(3)
            ]
            lift = list(state["eef_pos"])
            lift[2] += 0.025
            bounded_move(sdk, lift, state["eef_yaw"], "close", 10, 0.004)
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            offset_after = [
                nut["pos"][i] - state["eef_pos"][i] for i in range(3)
            ]
            slip = sqrt(sum(
                (offset_after[i] - offset_before[i]) ** 2 for i in range(3)
            ))
            if slip < 0.012 and nut["pos"][2] > before_height + 0.050:
                grasped = True
                break

        actuate_gripper(sdk, "open", 6)

    if not grasped:
        return {
            "status": "failed",
            "reason": "Could not verify a stable handle grasp",
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    move_nut(
        sdk, [nut["pos"][0], nut["pos"][1], clearance],
        18, 0.004, 0.35
    )

    aligned = False
    for iteration in range(6):
        if remaining_steps(sdk) < 60:
            break
        state = sdk.get_state()
        goal = state["goal_pos"]
        move_nut(
            sdk, [goal[0], goal[1], clearance],
            24 if iteration == 0 else 8, 0.0015, 0.35
        )
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        if (
            hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.003
            and abs(square_yaw_error(nut["yaw"])) < 0.035
            and abs(nut["pos"][2] - clearance) < 0.010
        ):
            aligned = True
            break

    if not aligned:
        return {
            "status": "failed",
            "reason": "Could not align above peg",
            "steps_used": sdk.get_state()["steps_used"],
        }

    seated_height = resting_height + 0.001
    stalled = 0
    for iteration in range(30):
        if remaining_steps(sdk) <= 22:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        position = list(nut["pos"])
        lateral = hypot(position[0] - goal[0], position[1] - goal[1])
        yaw_error = abs(square_yaw_error(nut["yaw"]))

        if (
            position[2] <= seated_height + 0.007
            and lateral < 0.006
            and yaw_error < 0.08
        ):
            break

        descent = 0.030 if position[2] > resting_height + 0.080 else 0.012
        target_height = max(seated_height, position[2] - descent)
        if lateral > 0.0035 or yaw_error > 0.040:
            target_height = position[2]

        move_nut(
            sdk,
            [goal[0], goal[1], target_height],
            min(8, remaining_steps(sdk) - 22),
            0.0015,
            0.10,
        )
        after = sdk.get_state()["objects"]["nut"]["pos"]
        if target_height < position[2] - 0.004 and position[2] - after[2] < 0.002:
            stalled += 1
        else:
            stalled = 0

        if stalled >= 2 and remaining_steps(sdk) > 54:
            state = sdk.get_state()
            goal = state["goal_pos"]
            recovery_height = state["objects"]["nut"]["pos"][2] + 0.018
            move_nut(
                sdk, [goal[0], goal[1], recovery_height],
                8, 0.002, 0.10
            )
            state = sdk.get_state()
            goal = state["goal_pos"]
            move_nut(
                sdk,
                [goal[0], goal[1], state["objects"]["nut"]["pos"][2]],
                8, 0.001, 0.10,
            )
            stalled = 0

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    if (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) > 0.008
        or nut["pos"][2] > resting_height + 0.016
        or abs(square_yaw_error(nut["yaw"])) > 0.10
    ):
        return {
            "status": "failed",
            "reason": "Insertion did not reach seated height",
            "steps_used": state["steps_used"],
        }

    actuate_gripper(sdk, "open", 10)
    state = sdk.get_state()
    retreat = list(state["eef_pos"])
    retreat[2] += 0.080
    bounded_move(sdk, retreat, state["eef_yaw"], "open", 12, 0.008)

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    lateral = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    seated = (
        lateral < 0.010
        and abs(nut["pos"][2] - resting_height) < 0.018
        and abs(square_yaw_error(nut["yaw"])) < 0.12
    )
    return {
        "status": "done" if seated else "failed",
        "lateral_error_m": lateral,
        "steps_used": state["steps_used"],
    }
