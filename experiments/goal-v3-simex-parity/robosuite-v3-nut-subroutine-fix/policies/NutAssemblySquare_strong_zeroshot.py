def angle_difference(target, current):
    return atan2(sin(target - current), cos(target - current))


def square_alignment(yaw):
    candidates = [
        angle_difference(k * pi / 2.0, yaw)
        for k in range(-2, 3)
    ]
    return min(candidates, key=abs)


def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def bounded_move(sdk, position, yaw, gripper, steps, tolerance):
    budget = min(steps, remaining_steps(sdk))
    if budget <= 0:
        return {"reached": False}
    return sdk.move_to(
        position,
        yaw=yaw,
        gripper=gripper,
        max_steps=budget,
        tol=tolerance,
    )


def actuate(sdk, action, steps):
    budget = min(steps, remaining_steps(sdk))
    if budget > 0:
        sdk.set_gripper(action, steps=budget)


def move_nut_center(sdk, target, steps, tolerance=0.003):
    state = sdk.get_state()
    nut = state["objects"]["nut"]["pos"]
    eef = state["eef_pos"]
    target_eef = [eef[i] + target[i] - nut[i] for i in range(3)]
    return bounded_move(
        sdk, target_eef, state["eef_yaw"], "close", steps, tolerance
    )


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    grasped = False
    lift_height = None

    # The nut handle extends along its local positive x axis.
    # These are object-relative grasp offsets, not world coordinates.
    for handle_offset in [0.045, 0.038]:
        if remaining_steps(sdk) < 185:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        position = nut["pos"]
        yaw = nut["yaw"]
        grasp_yaw = state["eef_yaw"] + angle_difference(
            yaw, state["eef_yaw"]
        )
        grasp_x = position[0] + handle_offset * cos(yaw)
        grasp_y = position[1] + handle_offset * sin(yaw)
        approach_z = max(position[2] + 0.10, state["eef_pos"][2])

        actuate(sdk, "open", 7)
        bounded_move(
            sdk,
            [state["eef_pos"][0], state["eef_pos"][1], approach_z],
            state["eef_yaw"],
            "open",
            16,
            0.008,
        )
        bounded_move(
            sdk,
            [grasp_x, grasp_y, approach_z],
            grasp_yaw,
            "open",
            28,
            0.005,
        )

        # Refresh the handle location after approaching.
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        position = nut["pos"]
        yaw = nut["yaw"]
        grasp_yaw = state["eef_yaw"] + angle_difference(
            yaw, state["eef_yaw"]
        )
        grasp_point = [
            position[0] + handle_offset * cos(yaw),
            position[1] + handle_offset * sin(yaw),
            position[2],
        ]
        bounded_move(
            sdk, grasp_point, grasp_yaw, "open", 25, 0.003
        )
        before = sdk.get_state()
        baseline_z = before["objects"]["nut"]["pos"][2]
        actuate(sdk, "close", 12)

        state = sdk.get_state()
        eef = state["eef_pos"]
        bounded_move(
            sdk,
            [eef[0], eef[1], eef[2] + 0.105],
            state["eef_yaw"],
            "close",
            28,
            0.006,
        )
        state = sdk.get_state()
        rise = state["objects"]["nut"]["pos"][2] - baseline_z
        if rise > 0.055 and state["gripper_width"] > 0.012:
            grasped = True
            lift_height = state["objects"]["nut"]["pos"][2]
            break

        actuate(sdk, "open", 6)

    if not grasped:
        return {"status": "failed", "reason": "Nut did not follow the lift"}

    # Align a square-hole symmetry axis with the fixed square peg.
    # Measure again after rotation because the nut is grasped off center.
    for attempt in range(2):
        state = sdk.get_state()
        correction = square_alignment(state["objects"]["nut"]["yaw"])
        if abs(correction) < 0.025:
            break
        bounded_move(
            sdk,
            list(state["eef_pos"]),
            state["eef_yaw"] + correction,
            "close",
            22,
            0.004,
        )

    state = sdk.get_state()
    goal = state["goal_pos"]
    clearance_z = max(
        state["objects"]["nut"]["pos"][2], goal[2] + 0.12
    )
    nut = state["objects"]["nut"]["pos"]
    move_nut_center(sdk, [nut[0], nut[1], clearance_z], 18)
    move_nut_center(sdk, [goal[0], goal[1], clearance_z], 32)

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    if hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) > 0.018:
        return {"status": "failed", "reason": "Could not center nut above peg"}

    correction = square_alignment(nut["yaw"])
    if abs(correction) > 0.025:
        bounded_move(
            sdk,
            list(state["eef_pos"]),
            state["eef_yaw"] + correction,
            "close",
            16,
            0.003,
        )
        move_nut_center(sdk, [goal[0], goal[1], clearance_z], 14)

    # Descend incrementally, correcting the measured nut center each time.
    # Reserve enough time to release, withdraw, and inspect the result.
    seated = False
    stalled = 0
    offsets = [(0.002, 0.0), (-0.002, 0.0),
               (0.0, 0.002), (0.0, -0.002)]

    for iteration in range(16):
        if remaining_steps(sdk) < 34:
            break

        state = sdk.get_state()
        goal = state["goal_pos"]
        nut = state["objects"]["nut"]["pos"]
        error_xy = hypot(nut[0] - goal[0], nut[1] - goal[1])
        if error_xy > 0.025:
            return {"status": "failed", "reason": "Nut slipped during insertion"}

        target_z = goal[2] + 0.011
        if nut[2] <= target_z + 0.008 and error_xy < 0.009:
            seated = True
            break

        next_z = max(target_z, nut[2] - 0.015)
        previous_z = nut[2]
        move_nut_center(sdk, [goal[0], goal[1], next_z], 9, 0.002)
        state = sdk.get_state()
        actual_z = state["objects"]["nut"]["pos"][2]

        if previous_z - actual_z < 0.002:
            stalled += 1
        else:
            stalled = 0

        if stalled > 0 and remaining_steps(sdk) >= 60:
            # Back off before a small lateral search; do not push harder.
            nut = state["objects"]["nut"]["pos"]
            move_nut_center(
                sdk, [nut[0], nut[1], nut[2] + 0.008], 6
            )
            state = sdk.get_state()
            correction = square_alignment(state["objects"]["nut"]["yaw"])
            if abs(correction) > 0.02:
                bounded_move(
                    sdk,
                    list(state["eef_pos"]),
                    state["eef_yaw"] + correction,
                    "close",
                    8,
                    0.003,
                )
            offset = offsets[(stalled - 1) % len(offsets)]
            state = sdk.get_state()
            move_nut_center(
                sdk,
                [
                    goal[0] + offset[0],
                    goal[1] + offset[1],
                    state["objects"]["nut"]["pos"][2],
                ],
                6,
            )

    if not seated:
        return {
            "status": "failed",
            "reason": "Insertion depth was not verified",
            "steps_used": sdk.get_state()["steps_used"],
        }

    actuate(sdk, "open", 10)
    state = sdk.get_state()
    eef = state["eef_pos"]
    bounded_move(
        sdk,
        [eef[0], eef[1], eef[2] + 0.08],
        state["eef_yaw"],
        "open",
        18,
        0.008,
    )

    state = sdk.get_state()
    nut = state["objects"]["nut"]["pos"]
    goal = state["goal_pos"]
    centered = hypot(nut[0] - goal[0], nut[1] - goal[1]) < 0.012
    lowered = nut[2] < lift_height - 0.045
    at_seat = goal[2] - 0.015 <= nut[2] <= goal[2] + 0.025
    released = state["gripper_width"] > 0.055
    success = centered and lowered and at_seat and released
    return {
        "status": "done" if success else "failed",
        "reason": "Verified released nut pose" if success else "Final pose check failed",
        "steps_used": state["steps_used"],
    }
