def wrap_angle(angle):
    return atan2(sin(angle), cos(angle))


def square_error(yaw):
    return (yaw + pi / 4.0) % (pi / 2.0) - pi / 4.0


def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def move_bounded(sdk, position, yaw, gripper, steps, tolerance):
    available = remaining_steps(sdk)
    if available <= 0:
        return False
    result = sdk.move_to(
        position,
        yaw=yaw,
        gripper=gripper,
        max_steps=min(steps, available),
        tol=tolerance,
    )
    return result["reached"]


def grip_bounded(sdk, action, steps):
    available = remaining_steps(sdk)
    if available > 0:
        sdk.set_gripper(action, steps=min(steps, available))


def rotate_bounded(sdk, rotation, gripper, steps):
    available = remaining_steps(sdk)
    if available > 0:
        sdk.move_delta(
            [0.0, 0.0, 0.0],
            dyaw=rotation,
            gripper=gripper,
            steps=min(steps, available),
        )


def translate_nut(sdk, target, steps, tolerance):
    state = sdk.get_state()
    nut = state["objects"]["nut"]["pos"]
    eef = state["eef_pos"]
    return move_bounded(
        sdk,
        [
            eef[0] + target[0] - nut[0],
            eef[1] + target[1] - nut[1],
            eef[2] + target[2] - nut[2],
        ],
        state["eef_yaw"],
        "close",
        steps,
        tolerance,
    )


def run_policy(sdk):
    state = sdk.get_state()
    if state["goal_pos"] is None or "nut" not in state["objects"]:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    rest_height = state["objects"]["nut"]["pos"][2]
    held = False

    # Keep the fingertips above the table while enclosing the handle.
    # Contact-limited position tracking is not itself a failed grasp.
    for attempt in range(3):
        if remaining_steps(sdk) < 145:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        handle_offset = 0.055 if attempt != 1 else 0.060
        grasp_height = 0.014 if attempt != 2 else 0.007
        desired_yaw = nut["yaw"] + pi / 2.0
        grasp_yaw = wrap_angle(
            state["eef_yaw"]
            + (desired_yaw - state["eef_yaw"] + pi / 2.0) % pi
            - pi / 2.0
        )
        approach = [
            nut["pos"][0] + handle_offset * cos(nut["yaw"]),
            nut["pos"][1] + handle_offset * sin(nut["yaw"]),
            nut["pos"][2] + 0.085,
        ]
        move_bounded(sdk, approach, grasp_yaw, "open", 26, 0.005)

        state = sdk.get_state()
        yaw_error = (
            grasp_yaw - state["eef_yaw"] + pi / 2.0
        ) % pi - pi / 2.0
        if abs(yaw_error) > 0.05:
            rotate_bounded(sdk, yaw_error, "open", 7)

        for correction in range(2):
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            desired_yaw = nut["yaw"] + pi / 2.0
            grasp_yaw = wrap_angle(
                state["eef_yaw"]
                + (desired_yaw - state["eef_yaw"] + pi / 2.0) % pi
                - pi / 2.0
            )
            grasp = [
                nut["pos"][0] + handle_offset * cos(nut["yaw"]),
                nut["pos"][1] + handle_offset * sin(nut["yaw"]),
                nut["pos"][2] + grasp_height,
            ]
            move_bounded(
                sdk, grasp, grasp_yaw, "open",
                24 if correction == 0 else 9, 0.003
            )
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            handle_x = nut["pos"][0] + handle_offset * cos(nut["yaw"])
            handle_y = nut["pos"][1] + handle_offset * sin(nut["yaw"])
            if hypot(
                state["eef_pos"][0] - handle_x,
                state["eef_pos"][1] - handle_y,
            ) < 0.006:
                break

        baseline = sdk.get_state()["objects"]["nut"]["pos"][2]
        grip_bounded(sdk, "close", 14)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move_bounded(
            sdk, [eef[0], eef[1], eef[2] + 0.075],
            state["eef_yaw"], "close", 23, 0.004
        )

        state = sdk.get_state()
        held = (
            state["objects"]["nut"]["pos"][2] > baseline + 0.035
            and state["gripper_width"] > 0.008
        )
        if held:
            break
        grip_bounded(sdk, "open", 5)

    if not held:
        return {
            "status": "failed",
            "reason": "Could not verify lifted grasp",
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    goal = state["goal_pos"]
    clearance = max(rest_height + 0.13, goal[2] + 0.10)
    nut = state["objects"]["nut"]
    translate_nut(
        sdk, [nut["pos"][0], nut["pos"][1], clearance], 18, 0.004
    )

    for correction in range(3):
        state = sdk.get_state()
        error = square_error(state["objects"]["nut"]["yaw"])
        if abs(error) < 0.025 or remaining_steps(sdk) < 65:
            break
        rotate_bounded(sdk, -error, "close", 8)

    state = sdk.get_state()
    goal = state["goal_pos"]
    translate_nut(sdk, [goal[0], goal[1], clearance], 28, 0.002)

    previous_height = None
    stalled = 0
    for stage in range(24):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        available = remaining_steps(sdk)
        if available <= 24:
            break

        center_error = hypot(
            nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]
        )
        yaw_error = square_error(nut["yaw"])
        height = nut["pos"][2]

        if (
            height <= rest_height + 0.010
            and center_error < 0.006
            and abs(yaw_error) < 0.06
        ):
            break

        if abs(yaw_error) > 0.045:
            if height < clearance - 0.015:
                translate_nut(
                    sdk, [nut["pos"][0], nut["pos"][1], clearance],
                    min(16, available - 24), 0.003
                )
            if remaining_steps(sdk) > 32:
                rotate_bounded(sdk, -yaw_error, "close", 8)
            previous_height = None
            continue

        if previous_height is not None:
            stalled = stalled + 1 if previous_height - height < 0.002 else 0

        if stalled >= 2:
            translate_nut(
                sdk, [nut["pos"][0], nut["pos"][1], height + 0.025],
                min(12, available - 24), 0.002
            )
            state = sdk.get_state()
            goal = state["goal_pos"]
            height = state["objects"]["nut"]["pos"][2]
            if remaining_steps(sdk) > 24:
                translate_nut(
                    sdk, [goal[0], goal[1], height],
                    min(10, remaining_steps(sdk) - 24), 0.001
                )
            stalled = 0
            previous_height = None
            continue

        if center_error > 0.004:
            translate_nut(
                sdk, [goal[0], goal[1], height],
                min(10, available - 24), 0.0015
            )
            previous_height = height
            continue

        previous_height = height
        translate_nut(
            sdk,
            [
                goal[0],
                goal[1],
                max(rest_height + 0.001, height - 0.025),
            ],
            min(12, available - 24),
            0.002,
        )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    center_error = hypot(
        nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]
    )
    if (
        nut["pos"][2] > rest_height + 0.020
        or center_error > 0.008
        or abs(square_error(nut["yaw"])) > 0.08
    ):
        return {
            "status": "failed",
            "reason": "Insertion not verified",
            "center_error_m": center_error,
            "height_above_rest_m": nut["pos"][2] - rest_height,
            "steps_used": state["steps_used"],
        }

    grip_bounded(sdk, "open", 9)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move_bounded(
        sdk, [eef[0], eef[1], eef[2] + 0.09],
        state["eef_yaw"], "open", 15, 0.008
    )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    center_error = hypot(
        nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]
    )
    seated = (
        center_error < 0.010
        and abs(nut["pos"][2] - rest_height) < 0.020
        and abs(square_error(nut["yaw"])) < 0.10
    )
    return {
        "status": "done" if seated else "failed",
        "reason": "Released and seated" if seated else "Final pose check failed",
        "center_error_m": center_error,
        "steps_used": state["steps_used"],
    }
