def angle_wrap(angle):
    return (angle + pi) % (2.0 * pi) - pi


def policy_move(sdk, position, yaw, grip, ticks, tolerance=0.004):
    state = sdk.get_state()
    remaining = min(400, state["max_steps"]) - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        position,
        yaw=yaw,
        gripper=grip,
        max_steps=min(ticks, remaining),
        tol=tolerance,
    )
    return result["reached"]


def policy_gripper(sdk, action, ticks):
    state = sdk.get_state()
    remaining = min(400, state["max_steps"]) - state["steps_used"]
    if remaining > 0:
        sdk.set_gripper(action, steps=min(ticks, remaining))


def center_nut(sdk, target, ticks, tolerance=0.003, align_yaw=True):
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    eef = state["eef_pos"]
    position = [
        eef[0] + target[0] - nut["pos"][0],
        eef[1] + target[1] - nut["pos"][1],
        eef[2] + target[2] - nut["pos"][2],
    ]
    yaw = state["eef_yaw"]
    if align_yaw:
        correction = (nut["yaw"] + pi / 4.0) % (pi / 2.0) - pi / 4.0
        yaw = angle_wrap(yaw - correction)
    return policy_move(sdk, position, yaw, "close", ticks, tolerance)


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    nut = state["objects"]["nut"]
    resting_height = nut["pos"][2]
    initial_height = resting_height
    goal = state["goal_pos"]
    clearance = max(resting_height + 0.17, goal[2] + 0.12)

    # The standard square nut has a graspable handle along its local +X axis.
    grasped = False
    for attempt in range(2):
        state = sdk.get_state()
        remaining = min(400, state["max_steps"]) - state["steps_used"]
        if remaining < 200:
            break

        nut = state["objects"]["nut"]
        initial_height = nut["pos"][2]
        yaw = nut["yaw"]
        handle_offset = 0.055 if attempt == 0 else 0.060
        handle = [
            nut["pos"][0] + handle_offset * cos(yaw),
            nut["pos"][1] + handle_offset * sin(yaw),
            nut["pos"][2],
        ]

        policy_gripper(sdk, "open", 8)
        policy_move(
            sdk, [handle[0], handle[1], clearance],
            yaw, "open", 35, 0.006
        )

        # Refresh the grasp point after the approach.
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        handle = [
            nut["pos"][0] + handle_offset * cos(yaw),
            nut["pos"][1] + handle_offset * sin(yaw),
            nut["pos"][2],
        ]
        reached = policy_move(sdk, handle, yaw, "open", 25, 0.003)
        if not reached:
            continue

        policy_gripper(sdk, "close", 14)
        state = sdk.get_state()
        eef = state["eef_pos"]
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.09],
            state["eef_yaw"], "close", 25, 0.004
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        if (
            nut["pos"][2] > initial_height + 0.045
            and state["gripper_width"] > 0.012
        ):
            grasped = True
            break

        policy_gripper(sdk, "open", 6)

    if not grasped:
        return {"status": "failed", "reason": "Nut did not follow the lift"}

    # Rotate above the table, then remeasure the grasp offset.
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    center_nut(
        sdk, [nut["pos"][0], nut["pos"][1], clearance], 28
    )

    for iteration in range(3):
        state = sdk.get_state()
        goal = state["goal_pos"]
        center_nut(sdk, [goal[0], goal[1], clearance], 24)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        yaw_error = abs(
            (nut["yaw"] + pi / 4.0) % (pi / 2.0) - pi / 4.0
        )
        if (
            hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.004
            and yaw_error < 0.045
        ):
            break

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    if hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) > 0.012:
        return {"status": "failed", "reason": "Could not center nut above peg"}

    # The initial resting height measures the seated height on this table.
    # Descend in short stages, correcting from the nut pose each time.
    seated = False
    stalled = 0
    for iteration in range(16):
        state = sdk.get_state()
        remaining = min(400, state["max_steps"]) - state["steps_used"]
        if remaining < 34:
            break

        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        position = nut["pos"]
        lateral_error = hypot(position[0] - goal[0], position[1] - goal[1])

        if lateral_error > 0.025 or state["gripper_width"] < 0.010:
            return {"status": "failed", "reason": "Grasp lost during insertion"}

        if lateral_error < 0.008 and abs(position[2] - resting_height) < 0.010:
            seated = True
            break

        previous_height = position[2]
        target_height = max(resting_height, previous_height - 0.018)
        if lateral_error > 0.005:
            target_height = previous_height

        center_nut(sdk, [goal[0], goal[1], target_height], 12, 0.002)
        state = sdk.get_state()
        current_height = state["objects"]["nut"]["pos"][2]

        if target_height < previous_height - 0.005:
            if previous_height - current_height < 0.003:
                stalled += 1
            else:
                stalled = 0

        if stalled >= 2:
            # Unload rim contact before correcting position and square symmetry.
            nut = state["objects"]["nut"]
            goal = state["goal_pos"]
            center_nut(
                sdk,
                [nut["pos"][0], nut["pos"][1], nut["pos"][2] + 0.025],
                12, 0.003, False
            )
            state = sdk.get_state()
            goal = state["goal_pos"]
            center_nut(
                sdk,
                [goal[0], goal[1], state["objects"]["nut"]["pos"][2]],
                14, 0.002
            )
            stalled = 0

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    seated = seated or (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.008
        and abs(nut["pos"][2] - resting_height) < 0.010
    )
    if not seated:
        return {"status": "failed", "reason": "Insertion did not reach seated height"}

    policy_gripper(sdk, "open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.09],
        state["eef_yaw"], "open", 20, 0.006
    )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    settled = error < 0.012 and abs(nut["pos"][2] - resting_height) < 0.015
    return {
        "status": "done" if settled else "failed",
        "reason": "Released nut is seated" if settled else "Nut moved after release",
        "center_error_m": error,
        "steps_used": state["steps_used"],
    }
