def angle_wrap(angle):
    return atan2(sin(angle), cos(angle))


def square_error(target, current):
    return atan2(sin(4.0 * (target - current)),
                 cos(4.0 * (target - current))) / 4.0


def move_budget(sdk, target, yaw, gripper, ticks, tolerance=0.006):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining > 0:
        sdk.move_to(
            target,
            yaw=yaw,
            gripper=gripper,
            max_steps=min(ticks, remaining),
            tol=tolerance,
        )


def grip_budget(sdk, action, ticks):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining > 0:
        sdk.set_gripper(action, steps=min(ticks, remaining))


def run_policy(sdk):
    # The logs show unchanged nuts and elevated hands after only 70 steps:
    # approach timeouts must not discard an otherwise viable grasp attempt.
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or goal"}

    resting_height = state["objects"]["nut"]["pos"][2]
    held = False

    for attempt in range(2):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 175:
            break

        nut = state["objects"]["nut"]
        grasp_yaw = state["eef_yaw"] + (
            angle_wrap(2.0 * (nut["yaw"] - state["eef_yaw"])) / 2.0
        )
        if attempt == 1:
            grasp_yaw += pi / 2.0

        radius = 0.055
        handle_x = nut["pos"][0] + radius * cos(nut["yaw"])
        handle_y = nut["pos"][1] + radius * sin(nut["yaw"])
        approach_z = nut["pos"][2] + 0.105

        grip_budget(sdk, "open", 5)
        move_budget(
            sdk,
            [handle_x, handle_y, approach_z],
            grasp_yaw,
            "open",
            60,
            0.010,
        )

        # Inspect actual position, and allow an unfinished approach to converge.
        state = sdk.get_state()
        horizontal_error = hypot(
            state["eef_pos"][0] - handle_x,
            state["eef_pos"][1] - handle_y,
        )
        if horizontal_error > 0.018:
            move_budget(
                sdk,
                [handle_x, handle_y, approach_z],
                grasp_yaw,
                "open",
                25,
                0.008,
            )

        state = sdk.get_state()
        if hypot(
            state["eef_pos"][0] - handle_x,
            state["eef_pos"][1] - handle_y,
        ) > 0.022:
            continue

        nut = state["objects"]["nut"]
        handle = [
            nut["pos"][0] + radius * cos(nut["yaw"]),
            nut["pos"][1] + radius * sin(nut["yaw"]),
            nut["pos"][2] + 0.002,
        ]
        move_budget(sdk, handle, grasp_yaw, "open", 38, 0.004)
        state = sdk.get_state()
        if (
            hypot(state["eef_pos"][0] - handle[0],
                  state["eef_pos"][1] - handle[1]) > 0.012
            or abs(state["eef_pos"][2] - handle[2]) > 0.012
        ):
            move_budget(sdk, handle, grasp_yaw, "open", 16, 0.004)

        state = sdk.get_state()
        if (
            hypot(state["eef_pos"][0] - handle[0],
                  state["eef_pos"][1] - handle[1]) > 0.015
            or abs(state["eef_pos"][2] - handle[2]) > 0.015
        ):
            continue

        before_lift = state["objects"]["nut"]["pos"][2]
        grip_budget(sdk, "close", 12)
        state = sdk.get_state()
        move_budget(
            sdk,
            [state["eef_pos"][0],
             state["eef_pos"][1],
             state["eef_pos"][2] + 0.115],
            grasp_yaw,
            "close",
            30,
            0.008,
        )
        state = sdk.get_state()
        held = (
            0.012 < state["gripper_width"] < 0.070
            and state["objects"]["nut"]["pos"][2] - before_lift > 0.055
        )
        if held:
            break
        grip_budget(sdk, "open", 6)

    if not held:
        return {"status": "failed", "reason": "Handle grasp not verified"}

    state = sdk.get_state()
    peg_yaw = 0.0
    if "peg" in state["objects"]:
        peg_yaw = state["objects"]["peg"]["yaw"]
    clearance = max(resting_height + 0.115, state["goal_pos"][2] + 0.085)

    # Rotate the measured handle-to-hole vector with the held nut.
    nut = state["objects"]["nut"]
    rotation = square_error(peg_yaw, nut["yaw"])
    offset_x = state["eef_pos"][0] - nut["pos"][0]
    offset_y = state["eef_pos"][1] - nut["pos"][1]
    offset_z = state["eef_pos"][2] - nut["pos"][2]
    move_budget(
        sdk,
        [
            nut["pos"][0] + cos(rotation) * offset_x - sin(rotation) * offset_y,
            nut["pos"][1] + sin(rotation) * offset_x + cos(rotation) * offset_y,
            clearance + offset_z,
        ],
        state["eef_yaw"] + rotation,
        "close",
        26,
        0.006,
    )

    aligned = False
    for correction in range(5):
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining < 65:
            break
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        rotation = square_error(peg_yaw, nut["yaw"])
        offset_x = state["eef_pos"][0] - nut["pos"][0]
        offset_y = state["eef_pos"][1] - nut["pos"][1]
        target = [
            goal[0] + cos(rotation) * offset_x - sin(rotation) * offset_y,
            goal[1] + sin(rotation) * offset_x + cos(rotation) * offset_y,
            state["eef_pos"][2] + clearance - nut["pos"][2],
        ]
        move_budget(
            sdk, target, state["eef_yaw"] + rotation, "close",
            min(55 if correction == 0 else 14, remaining - 60),
            0.002,
        )
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        aligned = (
            hypot(nut["pos"][0] - state["goal_pos"][0],
                  nut["pos"][1] - state["goal_pos"][1]) < 0.004
            and abs(square_error(peg_yaw, nut["yaw"])) < 0.045
            and nut["pos"][2] >= clearance - 0.015
        )
        if aligned:
            break

    if not aligned:
        return {"status": "failed", "reason": "Hole alignment not verified"}

    state = sdk.get_state()
    hand_x = state["eef_pos"][0]
    hand_y = state["eef_pos"][1]
    hand_yaw = state["eef_yaw"]
    offset_z = state["eef_pos"][2] - state["objects"]["nut"]["pos"][2]
    seated = False

    for descent in range(12):
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining <= 22:
            break
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        if hypot(nut["pos"][0] - goal[0],
                 nut["pos"][1] - goal[1]) > 0.010:
            return {"status": "failed", "reason": "Hole shifted off peg"}

        if nut["pos"][2] <= resting_height + 0.008:
            seated = True
            break

        target_height = max(resting_height, nut["pos"][2] - 0.025)
        move_budget(
            sdk,
            [hand_x, hand_y, target_height + offset_z],
            hand_yaw,
            "close",
            min(9, remaining - 22),
            0.003,
        )

    state = sdk.get_state()
    seated = seated or state["objects"]["nut"]["pos"][2] <= resting_height + 0.008
    if not seated:
        return {"status": "failed", "reason": "Insertion did not reach seating height"}

    grip_budget(sdk, "open", 10)
    state = sdk.get_state()
    move_budget(
        sdk,
        [state["eef_pos"][0], state["eef_pos"][1], state["eef_pos"][2] + 0.085],
        state["eef_yaw"],
        "open",
        12,
        0.010,
    )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    success = (
        error < 0.012
        and abs(nut["pos"][2] - resting_height) < 0.012
        and abs(square_error(peg_yaw, nut["yaw"])) < 0.10
        and state["gripper_width"] > 0.050
    )
    return {
        "status": "done" if success else "failed",
        "reason": "Released and seated" if success else "Release verification failed",
        "hole_position_error_m": error,
        "steps_used": state["steps_used"],
    }
