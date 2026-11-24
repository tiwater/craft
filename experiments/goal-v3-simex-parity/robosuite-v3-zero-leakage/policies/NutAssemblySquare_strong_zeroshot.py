def wrap_angle(angle):
    return atan2(sin(angle), cos(angle))


def square_yaw_error(yaw):
    return 0.25 * atan2(sin(4.0 * yaw), cos(4.0 * yaw))


def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def bounded_move(sdk, position, yaw, gripper, steps, tolerance):
    budget = min(steps, remaining_steps(sdk))
    if budget <= 0:
        return False
    result = sdk.move_to(
        position,
        yaw=yaw,
        gripper=gripper,
        max_steps=budget,
        tol=tolerance,
    )
    return result["reached"]


def hold_gripper(sdk, action, steps):
    budget = min(steps, remaining_steps(sdk))
    if budget > 0:
        sdk.set_gripper(action, steps=budget)


def move_nut_center(sdk, target, steps, tolerance):
    state = sdk.get_state()
    nut = state["objects"]["nut"]["pos"]
    eef = state["eef_pos"]
    position = [
        eef[0] + target[0] - nut[0],
        eef[1] + target[1] - nut[1],
        eef[2] + target[2] - nut[2],
    ]
    return bounded_move(
        sdk, position, state["eef_yaw"], "close", steps, tolerance
    )


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    rest_height = state["objects"]["nut"]["pos"][2]
    goal = state["goal_pos"]
    clearance = max(rest_height + 0.16, goal[2] + 0.12)
    grasped = False

    # The square nut's grasping handle extends along its local positive X.
    # These offsets describe the object's geometry, not a sampled world pose.
    for attempt in range(2):
        if remaining_steps(sdk) < 160:
            break

        hold_gripper(sdk, "open", 7)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        handle_offset = 0.055 if attempt == 0 else 0.049
        handle = [
            nut["pos"][0] + handle_offset * cos(yaw),
            nut["pos"][1] + handle_offset * sin(yaw),
            nut["pos"][2] + 0.004,
        ]

        bounded_move(
            sdk,
            [handle[0], handle[1], clearance],
            yaw,
            "open",
            30,
            0.008,
        )

        # Refresh the handle position immediately before descending.
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        handle = [
            nut["pos"][0] + handle_offset * cos(yaw),
            nut["pos"][1] + handle_offset * sin(yaw),
            nut["pos"][2] + 0.004,
        ]
        bounded_move(sdk, handle, yaw, "open", 25, 0.003)

        state = sdk.get_state()
        if hypot(
            state["eef_pos"][0] - handle[0],
            state["eef_pos"][1] - handle[1],
        ) > 0.012:
            continue

        before_lift = state["objects"]["nut"]["pos"][2]
        hold_gripper(sdk, "close", 13)
        state = sdk.get_state()
        eef = state["eef_pos"]
        bounded_move(
            sdk,
            [eef[0], eef[1], eef[2] + 0.09],
            state["eef_yaw"],
            "close",
            25,
            0.006,
        )
        state = sdk.get_state()
        grasped = (
            state["objects"]["nut"]["pos"][2] > before_lift + 0.045
            and state["gripper_width"] > 0.012
        )
        if grasped:
            break

    if not grasped:
        return {"status": "failed", "reason": "Nut did not follow the lift"}

    state = sdk.get_state()
    nut = state["objects"]["nut"]["pos"]
    move_nut_center(sdk, [nut[0], nut[1], clearance], 22, 0.006)

    # The task's fixed square peg is world aligned; its symmetry permits
    # choosing the nearest multiple of pi/2 instead of a large rotation.
    for iteration in range(3):
        state = sdk.get_state()
        error = square_yaw_error(state["objects"]["nut"]["yaw"])
        if abs(error) < 0.025:
            break
        bounded_move(
            sdk,
            list(state["eef_pos"]),
            wrap_angle(state["eef_yaw"] - error),
            "close",
            15,
            0.003,
        )

    state = sdk.get_state()
    goal = state["goal_pos"]
    move_nut_center(sdk, [goal[0], goal[1], clearance], 35, 0.004)

    # Re-estimate the actual grasp offset after transport and rotation.
    for iteration in range(2):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        error = square_yaw_error(nut["yaw"])
        if abs(error) > 0.025:
            bounded_move(
                sdk,
                list(state["eef_pos"]),
                wrap_angle(state["eef_yaw"] - error),
                "close",
                10,
                0.002,
            )
        state = sdk.get_state()
        goal = state["goal_pos"]
        move_nut_center(sdk, [goal[0], goal[1], clearance], 10, 0.002)

    state = sdk.get_state()
    if abs(square_yaw_error(state["objects"]["nut"]["yaw"])) > 0.12:
        return {"status": "failed", "reason": "Unable to align square hole"}

    # Approach from above the peg, then descend in short, measured stages.
    approach_height = max(rest_height + 0.075, state["goal_pos"][2] + 0.065)
    approach_height = min(clearance, approach_height)
    goal = state["goal_pos"]
    move_nut_center(sdk, [goal[0], goal[1], approach_height], 20, 0.003)

    for iteration in range(10):
        if remaining_steps(sdk) < 22:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        center = nut["pos"]
        lateral_error = hypot(center[0] - goal[0], center[1] - goal[1])
        yaw_error = square_yaw_error(nut["yaw"])

        if abs(yaw_error) > 0.10 or lateral_error > 0.012:
            # Clear the peg before correcting a substantial misalignment.
            move_nut_center(
                sdk, [center[0], center[1], approach_height], 12, 0.003
            )
            state = sdk.get_state()
            bounded_move(
                sdk,
                list(state["eef_pos"]),
                wrap_angle(
                    state["eef_yaw"]
                    - square_yaw_error(state["objects"]["nut"]["yaw"])
                ),
                "close",
                10,
                0.002,
            )
            move_nut_center(
                sdk, [goal[0], goal[1], approach_height], 12, 0.002
            )
            continue

        if lateral_error > 0.0025:
            move_nut_center(
                sdk, [goal[0], goal[1], center[2]], 7, 0.0015
            )

        state = sdk.get_state()
        center = state["objects"]["nut"]["pos"]
        target_height = max(rest_height + 0.002, center[2] - 0.014)
        move_nut_center(
            sdk, [goal[0], goal[1], target_height], 9, 0.002
        )

        state = sdk.get_state()
        center = state["objects"]["nut"]["pos"]
        if (
            center[2] <= rest_height + 0.012
            and hypot(center[0] - goal[0], center[1] - goal[1]) < 0.006
        ):
            break

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    seated = (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.008
        and abs(nut["pos"][2] - rest_height) < 0.015
        and abs(square_yaw_error(nut["yaw"])) < 0.12
    )
    if not seated:
        return {"status": "failed", "reason": "Insertion could not be verified"}

    if remaining_steps(sdk) < 16:
        return {"status": "failed", "reason": "Insufficient budget to release"}

    hold_gripper(sdk, "open", 8)
    state = sdk.get_state()
    eef = state["eef_pos"]
    bounded_move(
        sdk,
        [eef[0], eef[1], eef[2] + 0.09],
        state["eef_yaw"],
        "open",
        18,
        0.008,
    )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    center_error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    seated = (
        center_error < 0.008
        and abs(nut["pos"][2] - rest_height) < 0.015
        and abs(square_yaw_error(nut["yaw"])) < 0.12
        and state["gripper_width"] > 0.05
    )
    return {
        "status": "done" if seated else "failed",
        "reason": "Released nut pose verified" if seated else "Release verification failed",
        "center_error_m": center_error,
        "steps_used": state["steps_used"],
    }
