def wrap_angle(angle):
    return atan2(sin(angle), cos(angle))


def square_error(yaw):
    return 0.25 * atan2(sin(4.0 * yaw), cos(4.0 * yaw))


def remaining(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def move(sdk, position, yaw, gripper, ticks, tolerance=0.006, reserve=0):
    budget = min(ticks, remaining(sdk) - reserve)
    if budget <= 0:
        return False
    result = sdk.move_to(
        position, yaw=yaw, gripper=gripper,
        max_steps=budget, tol=tolerance
    )
    return result["reached"]


def grip(sdk, action, ticks, reserve=0):
    budget = min(ticks, remaining(sdk) - reserve)
    if budget > 0:
        sdk.set_gripper(action, steps=budget)


def center_move(sdk, target, ticks, tolerance=0.006, reserve=0):
    state = sdk.get_state()
    hand = state["eef_pos"]
    nut = state["objects"]["nut"]["pos"]
    return move(
        sdk,
        [hand[i] + target[i] - nut[i] for i in range(3)],
        state["eef_yaw"], "close", ticks, tolerance, reserve
    )


def align(sdk, reserve):
    for iteration in range(2):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        correction = -square_error(nut["yaw"])
        if abs(correction) < 0.035:
            return True
        hand = state["eef_pos"]
        dx = hand[0] - nut["pos"][0]
        dy = hand[1] - nut["pos"][1]
        move(
            sdk,
            [
                nut["pos"][0] + cos(correction) * dx - sin(correction) * dy,
                nut["pos"][1] + sin(correction) * dx + cos(correction) * dy,
                hand[2]
            ],
            wrap_angle(state["eef_yaw"] + correction),
            "close", 22, 0.005, reserve
        )
    return abs(square_error(sdk.get_state()["objects"]["nut"]["yaw"])) < 0.07


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or goal"}

    resting_height = state["objects"]["nut"]["pos"][2]
    clearance = max(resting_height + 0.095, state["goal_pos"][2] + 0.12)
    grasped = False

    for attempt in range(2):
        if remaining(sdk) < 235:
            break

        state = sdk.get_state()
        hand = state["eef_pos"]
        nut = state["objects"]["nut"]
        approach_height = max(nut["pos"][2] + 0.075, clearance)

        if hand[2] < nut["pos"][2] + 0.055:
            move(
                sdk, [hand[0], hand[1], approach_height],
                state["eef_yaw"], "open", 18, 0.010, 210
            )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        grasp_yaw = wrap_angle(
            state["eef_yaw"] + 0.5 * atan2(
                sin(2.0 * (nut["yaw"] - state["eef_yaw"])),
                cos(2.0 * (nut["yaw"] - state["eef_yaw"]))
            )
        )
        move(
            sdk,
            [
                nut["pos"][0] + 0.05 * cos(nut["yaw"]),
                nut["pos"][1] + 0.05 * sin(nut["yaw"]),
                approach_height
            ],
            grasp_yaw, "open", 40, 0.008, 200
        )

        for refinement in range(2):
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            target = [
                nut["pos"][0] + 0.05 * cos(nut["yaw"]),
                nut["pos"][1] + 0.05 * sin(nut["yaw"]),
                nut["pos"][2] + 0.003
            ]
            move(
                sdk, target, grasp_yaw, "open",
                32 if refinement == 0 else 12, 0.006, 180
            )
            state = sdk.get_state()
            hand = state["eef_pos"]
            if (
                hypot(hand[0] - target[0], hand[1] - target[1]) < 0.009
                and abs(hand[2] - target[2]) < 0.011
            ):
                break

        state = sdk.get_state()
        hand = state["eef_pos"]
        if (
            hypot(hand[0] - target[0], hand[1] - target[1]) > 0.014
            or abs(hand[2] - target[2]) > 0.017
        ):
            continue

        before = state["objects"]["nut"]["pos"][2]
        grip(sdk, "close", 12, 165)
        state = sdk.get_state()
        hand = state["eef_pos"]
        nut = state["objects"]["nut"]["pos"]
        move(
            sdk,
            [hand[0], hand[1], hand[2] + clearance - nut[2]],
            state["eef_yaw"], "close", 38, 0.008, 145
        )
        state = sdk.get_state()
        if (
            state["objects"]["nut"]["pos"][2] > before + 0.035
            and state["gripper_width"] > 0.012
        ):
            grasped = True
            break
        grip(sdk, "open", 8, 150)

    if not grasped:
        return {"status": "failed", "reason": "Nut did not follow grasp lift"}

    state = sdk.get_state()
    nut = state["objects"]["nut"]["pos"]
    if nut[2] < clearance - 0.012:
        center_move(sdk, [nut[0], nut[1], clearance], 18, 0.006, 140)

    align(sdk, 135)

    # Command the complete transport, then correct the measured hole offset.
    for iteration in range(4):
        state = sdk.get_state()
        nut = state["objects"]["nut"]["pos"]
        goal = state["goal_pos"]
        distance = hypot(goal[0] - nut[0], goal[1] - nut[1])
        if distance < 0.0035 and nut[2] >= clearance - 0.012:
            break
        if remaining(sdk) <= 95:
            break
        center_move(
            sdk, [goal[0], goal[1], clearance],
            58 if iteration == 0 else 18, 0.003, 95
        )

    state = sdk.get_state()
    if abs(square_error(state["objects"]["nut"]["yaw"])) > 0.05:
        align(sdk, 75)

    for iteration in range(2):
        state = sdk.get_state()
        nut = state["objects"]["nut"]["pos"]
        goal = state["goal_pos"]
        if hypot(nut[0] - goal[0], nut[1] - goal[1]) < 0.004:
            break
        center_move(sdk, [goal[0], goal[1], clearance], 20, 0.002, 65)

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    if (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) > 0.007
        or abs(square_error(nut["yaw"])) > 0.09
    ):
        return {
            "status": "failed",
            "reason": "Hole alignment incomplete",
            "steps_used": state["steps_used"]
        }

    stalled = 0
    for iteration in range(9):
        if remaining(sdk) <= 22:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        height = nut["pos"][2]
        lateral_error = hypot(
            nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]
        )
        if height <= resting_height + 0.009 and lateral_error < 0.008:
            break

        if lateral_error > 0.005:
            center_move(
                sdk, [goal[0], goal[1], height + 0.003],
                14, 0.002, 22
            )
        else:
            center_move(
                sdk,
                [goal[0], goal[1], max(resting_height + 0.001, height - 0.035)],
                18, 0.003, 22
            )

        state = sdk.get_state()
        new_height = state["objects"]["nut"]["pos"][2]
        stalled = stalled + 1 if height - new_height < 0.002 else 0
        if stalled >= 2 and remaining(sdk) > 60:
            goal = state["goal_pos"]
            center_move(
                sdk, [goal[0], goal[1], new_height + 0.015],
                12, 0.003, 42
            )
            align(sdk, 32)
            stalled = 0

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    seated = (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.009
        and abs(nut["pos"][2] - resting_height) < 0.014
        and abs(square_error(nut["yaw"])) < 0.10
    )
    if not seated:
        return {
            "status": "failed",
            "reason": "Insertion did not seat",
            "steps_used": state["steps_used"]
        }

    grip(sdk, "open", 10, 10)
    state = sdk.get_state()
    hand = state["eef_pos"]
    move(
        sdk, [hand[0], hand[1], hand[2] + 0.065],
        state["eef_yaw"], "open", 20, 0.010
    )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    success = (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.012
        and abs(nut["pos"][2] - resting_height) < 0.016
        and abs(square_error(nut["yaw"])) < 0.10
        and state["gripper_width"] > 0.05
    )
    return {
        "status": "done" if success else "failed",
        "reason": "Released seated nut" if success else "Release verification failed",
        "steps_used": state["steps_used"]
    }
