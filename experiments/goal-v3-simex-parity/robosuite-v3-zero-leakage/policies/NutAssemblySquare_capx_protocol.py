def angle_wrap(angle):
    return atan2(sin(angle), cos(angle))


def square_error(yaw):
    return atan2(sin(4.0 * yaw), cos(4.0 * yaw)) / 4.0


def budget_left(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def move_bounded(sdk, position, yaw, grip, steps, tolerance, reserve=0):
    available = min(steps, budget_left(sdk) - reserve)
    if available <= 0:
        return False
    result = sdk.move_to(
        position, yaw=yaw, gripper=grip,
        max_steps=available, tol=tolerance
    )
    return result["reached"]


def grip_bounded(sdk, action, steps, reserve=0):
    available = min(steps, budget_left(sdk) - reserve)
    if available > 0:
        sdk.set_gripper(action, steps=available)


def center_move(sdk, target, steps, tolerance, reserve=24):
    state = sdk.get_state()
    center = state["objects"]["nut"]["pos"]
    hand = state["eef_pos"]
    return move_bounded(
        sdk,
        [hand[i] + target[i] - center[i] for i in range(3)],
        state["eef_yaw"], "close", steps, tolerance, reserve
    )


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    rest_height = state["objects"]["nut"]["pos"][2]
    clearance = max(rest_height + 0.13, state["goal_pos"][2] + 0.10)
    grasped = False

    for attempt in range(2):
        if budget_left(sdk) < 180:
            break

        grip_bounded(sdk, "open", 14, 160)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        offset = 0.055
        handle = [
            nut["pos"][0] + offset * cos(yaw),
            nut["pos"][1] + offset * sin(yaw),
            nut["pos"][2] + 0.002,
        ]
        move_bounded(
            sdk, [handle[0], handle[1], clearance],
            yaw, "open", 35, 0.006, 145
        )

        # Finish the descent before closing. A timed-out approach is
        # not evidence that the fingers surround the handle.
        positioned = False
        for stage in range(4):
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            yaw = nut["yaw"]
            handle = [
                nut["pos"][0] + offset * cos(yaw),
                nut["pos"][1] + offset * sin(yaw),
                nut["pos"][2] + 0.002,
            ]
            move_bounded(sdk, handle, yaw, "open", 20, 0.004, 130)
            state = sdk.get_state()
            hand = state["eef_pos"]
            nut = state["objects"]["nut"]
            live_handle = [
                nut["pos"][0] + offset * cos(nut["yaw"]),
                nut["pos"][1] + offset * sin(nut["yaw"]),
                nut["pos"][2] + 0.002,
            ]
            positioned = (
                hypot(hand[0] - live_handle[0], hand[1] - live_handle[1]) < 0.008
                and abs(hand[2] - live_handle[2]) < 0.007
                and state["gripper_width"] > 0.045
            )
            if positioned or budget_left(sdk) <= 130:
                break

        if not positioned:
            return {
                "status": "failed",
                "reason": "Grasp approach did not reach handle height",
                "steps_used": sdk.get_state()["steps_used"],
            }

        before_lift = state["objects"]["nut"]["pos"][2]
        grip_bounded(sdk, "close", 16, 110)
        state = sdk.get_state()
        if state["gripper_width"] < 0.012:
            continue

        hand = state["eef_pos"]
        move_bounded(
            sdk, [hand[0], hand[1], hand[2] + 0.09],
            state["eef_yaw"], "close", 30, 0.006, 85
        )
        state = sdk.get_state()
        grasped = (
            state["objects"]["nut"]["pos"][2] > before_lift + 0.045
            and state["gripper_width"] > 0.012
        )
        if grasped:
            break

    if not grasped:
        return {"status": "failed", "reason": "Nut did not follow grasp and lift"}

    state = sdk.get_state()
    center = state["objects"]["nut"]["pos"]
    center_move(sdk, [center[0], center[1], clearance], 20, 0.005, 80)

    for iteration in range(3):
        state = sdk.get_state()
        error = square_error(state["objects"]["nut"]["yaw"])
        if abs(error) < 0.035:
            break
        target_yaw = angle_wrap(state["eef_yaw"] - error)
        # A positional move may terminate before yaw settles.
        ticks = min(10, max(0, budget_left(sdk) - 75))
        if ticks == 0:
            break
        sdk.move_delta(
            [0.0, 0.0, 0.0],
            dyaw=angle_wrap(target_yaw - state["eef_yaw"]),
            gripper="close", steps=ticks
        )

    for iteration in range(3):
        state = sdk.get_state()
        goal = state["goal_pos"]
        center_move(sdk, [goal[0], goal[1], clearance], 22, 0.003, 65)
        state = sdk.get_state()
        center = state["objects"]["nut"]["pos"]
        if hypot(center[0] - goal[0], center[1] - goal[1]) < 0.004:
            break

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    if (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) > 0.009
        or abs(square_error(nut["yaw"])) > 0.10
    ):
        return {"status": "failed", "reason": "Peg alignment not achieved"}

    # Keep measuring the nut itself as the held offset can change.
    for iteration in range(14):
        if budget_left(sdk) <= 26:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        center = nut["pos"]
        goal = state["goal_pos"]
        lateral = hypot(center[0] - goal[0], center[1] - goal[1])

        if center[2] <= rest_height + 0.010 and lateral < 0.007:
            break

        if abs(square_error(nut["yaw"])) > 0.12 or lateral > 0.015:
            return {"status": "failed", "reason": "Alignment lost during insertion"}

        target_z = max(rest_height + 0.001, center[2] - 0.022)
        if lateral > 0.005:
            target_z = center[2]
        center_move(sdk, [goal[0], goal[1], target_z], 12, 0.002, 26)

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    seated = (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.008
        and abs(nut["pos"][2] - rest_height) < 0.014
        and abs(square_error(nut["yaw"])) < 0.12
    )
    if not seated:
        return {"status": "failed", "reason": "Nut seating could not be verified"}

    grip_bounded(sdk, "open", 14)
    state = sdk.get_state()
    hand = state["eef_pos"]
    move_bounded(
        sdk, [hand[0], hand[1], hand[2] + 0.08],
        state["eef_yaw"], "open", 12, 0.008
    )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    seated = (
        error < 0.008
        and abs(nut["pos"][2] - rest_height) < 0.014
        and abs(square_error(nut["yaw"])) < 0.12
        and state["gripper_width"] > 0.045
    )
    return {
        "status": "done" if seated else "failed",
        "reason": "Released nut pose verified" if seated else "Release verification failed",
        "center_error_m": error,
        "steps_used": state["steps_used"],
    }
