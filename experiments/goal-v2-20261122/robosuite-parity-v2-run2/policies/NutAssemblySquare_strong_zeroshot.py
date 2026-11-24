def wrap_angle(angle):
    return atan2(sin(angle), cos(angle))


def square_error(yaw):
    return (yaw + pi / 4.0) % (pi / 2.0) - pi / 4.0


def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def bounded_move(sdk, position, yaw, gripper, steps, tolerance):
    budget = remaining_steps(sdk)
    if budget <= 0:
        return None
    return sdk.move_to(
        position,
        yaw=wrap_angle(yaw),
        gripper=gripper,
        max_steps=min(steps, budget),
        tol=tolerance,
    )


def actuate(sdk, action, steps):
    budget = remaining_steps(sdk)
    if budget > 0:
        return sdk.set_gripper(action, steps=min(steps, budget))
    return None


def move_nut_center(sdk, target, yaw_bias, steps):
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    position = [
        state["eef_pos"][axis] + target[axis] - nut["pos"][axis]
        for axis in range(3)
    ]
    yaw = state["eef_yaw"] - square_error(nut["yaw"]) + yaw_bias
    return bounded_move(sdk, position, yaw, "close", steps, 0.003)


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    # The initial resting nut height supplies the table-level seating height.
    resting_height = state["objects"]["nut"]["pos"][2]
    held = False

    # SquareNut's handle extends along its local positive x axis.
    for attempt in range(2):
        if remaining_steps(sdk) < 170:
            break

        actuate(sdk, "open", 8)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        handle_offset = 0.055 if attempt == 0 else 0.045
        grasp_x = nut["pos"][0] + handle_offset * cos(yaw)
        grasp_y = nut["pos"][1] + handle_offset * sin(yaw)
        clearance = max(state["eef_pos"][2], nut["pos"][2] + 0.10)

        bounded_move(
            sdk,
            [state["eef_pos"][0], state["eef_pos"][1], clearance],
            yaw,
            "open",
            12,
            0.008,
        )
        bounded_move(
            sdk, [grasp_x, grasp_y, clearance], yaw, "open", 25, 0.005
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        grasp = [
            nut["pos"][0] + handle_offset * cos(yaw),
            nut["pos"][1] + handle_offset * sin(yaw),
            nut["pos"][2],
        ]
        bounded_move(sdk, grasp, yaw, "open", 22, 0.003)
        actuate(sdk, "close", 12)

        state = sdk.get_state()
        before = list(state["objects"]["nut"]["pos"])
        lift = list(state["eef_pos"])
        lift[2] += 0.105
        bounded_move(sdk, lift, state["eef_yaw"], "close", 25, 0.005)

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        held = (
            nut["pos"][2] - before[2] > 0.055
            and state["gripper_width"] > 0.012
            and hypot(
                nut["pos"][0] - state["eef_pos"][0],
                nut["pos"][1] - state["eef_pos"][1],
            ) < 0.11
        )
        if held:
            break

    if not held:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    goal = state["goal_pos"]
    transit_height = max(resting_height + 0.15, goal[2] + 0.10)
    nut = state["objects"]["nut"]

    # Rotate above the table before translating across the peg.
    move_nut_center(
        sdk, [nut["pos"][0], nut["pos"][1], transit_height], 0.0, 22
    )
    state = sdk.get_state()
    goal = state["goal_pos"]
    move_nut_center(sdk, [goal[0], goal[1], transit_height], 0.0, 30)

    seated = False
    stalls = 0
    previous_height = sdk.get_state()["objects"]["nut"]["pos"][2]

    for iteration in range(24):
        if remaining_steps(sdk) < 28:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        lateral_error = hypot(
            nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]
        )
        angle_error = abs(square_error(nut["yaw"]))

        if (
            lateral_error < 0.006
            and angle_error < 0.07
            and nut["pos"][2] <= resting_height + 0.008
        ):
            seated = True
            break

        if nut["pos"][2] < resting_height + 0.025 and lateral_error > 0.035:
            return {"status": "failed", "reason": "Nut slipped from grasp"}

        # Correct actual nut pose, including any change of grasp offset.
        if lateral_error > 0.004 or angle_error > 0.045:
            target_height = nut["pos"][2]
            if lateral_error > 0.015:
                target_height = max(target_height, transit_height)
            move_nut_center(
                sdk, [goal[0], goal[1], target_height], 0.0, 9
            )
            continue

        target_height = max(resting_height + 0.001, nut["pos"][2] - 0.012)
        move_nut_center(
            sdk, [goal[0], goal[1], target_height], 0.0, 8
        )

        current = sdk.get_state()["objects"]["nut"]["pos"]
        if previous_height - current[2] < 0.002:
            stalls += 1
        else:
            stalls = 0
        previous_height = current[2]

        if stalls >= 2 and remaining_steps(sdk) >= 55:
            # Unload contact before a small angular correction and retry.
            state = sdk.get_state()
            goal = state["goal_pos"]
            height = state["objects"]["nut"]["pos"][2] + 0.025
            bias = 0.025 if iteration % 2 == 0 else -0.025
            move_nut_center(sdk, [goal[0], goal[1], height], bias, 10)
            move_nut_center(sdk, [goal[0], goal[1], height], 0.0, 8)
            stalls = 0
            previous_height = sdk.get_state()["objects"]["nut"]["pos"][2]

    if not seated:
        return {
            "status": "failed",
            "reason": "Insertion could not be verified before the step limit",
            "steps_used": sdk.get_state()["steps_used"],
        }

    actuate(sdk, "open", 10)
    state = sdk.get_state()
    retreat = list(state["eef_pos"])
    retreat[2] += 0.09
    bounded_move(sdk, retreat, state["eef_yaw"], "open", 16, 0.008)

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    lateral_error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    verified = (
        lateral_error < 0.008
        and abs(square_error(nut["yaw"])) < 0.10
        and abs(nut["pos"][2] - resting_height) < 0.012
    )
    return {
        "status": "done" if verified else "failed",
        "reason": "Released seating verified from live pose"
        if verified
        else "Nut moved out of its seated pose after release",
        "center_error_m": lateral_error,
        "steps_used": state["steps_used"],
    }
