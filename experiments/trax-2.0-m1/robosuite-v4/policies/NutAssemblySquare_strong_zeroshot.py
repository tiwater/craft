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


def move_nut(sdk, target, steps, tolerance, yaw_limit):
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    eef = state["eef_pos"]
    offset = [nut["pos"][i] - eef[i] for i in range(3)]
    turn = max(-yaw_limit, min(yaw_limit, square_yaw_error(nut["yaw"])))

    # Rotate the measured grasp offset about the wrist, preserving nut center.
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
    goal = initial["goal_pos"]
    clearance = max(resting_height + 0.17, goal[2] + 0.13)
    grasped = False

    # The SquareNut handle projects along its local positive X axis.
    # All handle targets are transformed from the current object pose.
    for attempt in range(3):
        if remaining_steps(sdk) < 190:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        radius = [0.050, 0.043, 0.057][attempt]
        handle = [
            nut["pos"][0] + radius * cos(yaw),
            nut["pos"][1] + radius * sin(yaw),
            nut["pos"][2],
        ]
        grasp_yaw = state["eef_yaw"] + (
            0.5 * atan2(
                sin(2.0 * (yaw - state["eef_yaw"])),
                cos(2.0 * (yaw - state["eef_yaw"])),
            )
        )

        actuate_gripper(sdk, "open", 8)
        bounded_move(
            sdk, [handle[0], handle[1], clearance],
            grasp_yaw, "open", 28, 0.008,
        )

        # Refresh the handle location after the approach.
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        handle = [
            nut["pos"][0] + radius * cos(nut["yaw"]),
            nut["pos"][1] + radius * sin(nut["yaw"]),
            nut["pos"][2],
        ]
        before_height = nut["pos"][2]
        bounded_move(sdk, handle, grasp_yaw, "open", 23, 0.004)
        actuate_gripper(sdk, "close", 12)

        state = sdk.get_state()
        lift = list(state["eef_pos"])
        lift[2] += 0.065
        bounded_move(sdk, lift, state["eef_yaw"], "close", 22, 0.006)

        state = sdk.get_state()
        rise = state["objects"]["nut"]["pos"][2] - before_height
        if rise > 0.035 and state["gripper_width"] > 0.012:
            grasped = True
            break

        actuate_gripper(sdk, "open", 6)

    if not grasped:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    move_nut(
        sdk, [nut["pos"][0], nut["pos"][1], clearance],
        24, 0.006, 0.30,
    )

    # Align above the peg before descending. The square hole has four
    # equivalent orientations; use the nearest world-axis alignment.
    for iteration in range(5):
        if remaining_steps(sdk) < 85:
            break
        state = sdk.get_state()
        goal = state["goal_pos"]
        move_nut(sdk, [goal[0], goal[1], clearance], 17, 0.004, 0.30)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        if (
            hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.004
            and abs(square_yaw_error(nut["yaw"])) < 0.045
            and abs(nut["pos"][2] - clearance) < 0.012
        ):
            break

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    if (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) > 0.012
        or abs(square_yaw_error(nut["yaw"])) > 0.13
    ):
        return {"status": "failed", "reason": "Could not align above peg"}

    # Use the observed starting nut height as the seated-height reference.
    seated_height = resting_height + 0.002
    stalled = 0
    for iteration in range(18):
        if remaining_steps(sdk) < 32:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        position = nut["pos"]
        lateral_error = hypot(position[0] - goal[0], position[1] - goal[1])

        if position[2] < seated_height + 0.009 and lateral_error < 0.008:
            break

        # Correct lateral drift before applying another downward increment.
        descent = 0.018 if position[2] > resting_height + 0.10 else 0.008
        target_height = max(seated_height, position[2] - descent)
        if lateral_error > 0.006 or abs(square_yaw_error(nut["yaw"])) > 0.07:
            target_height = position[2]

        before_height = position[2]
        move_nut(
            sdk, [goal[0], goal[1], target_height],
            9, 0.0025, 0.07,
        )
        after = sdk.get_state()["objects"]["nut"]["pos"]
        if target_height < before_height - 0.004 and before_height - after[2] < 0.002:
            stalled += 1
        else:
            stalled = 0

        if stalled >= 2 and remaining_steps(sdk) > 55:
            # Unload contact, then re-center using the newly measured pose.
            state = sdk.get_state()
            goal = state["goal_pos"]
            height = state["objects"]["nut"]["pos"][2] + 0.015
            move_nut(sdk, [goal[0], goal[1], height], 10, 0.003, 0.12)
            stalled = 0

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    if (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) > 0.012
        or nut["pos"][2] > resting_height + 0.025
    ):
        return {"status": "failed", "reason": "Insertion did not reach seated height"}

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
    )
    return {
        "status": "done" if seated else "failed",
        "verification": "Released nut pose relative to peg and initial resting height",
        "lateral_error_m": lateral_error,
        "steps_used": final["steps_used"],
    }
