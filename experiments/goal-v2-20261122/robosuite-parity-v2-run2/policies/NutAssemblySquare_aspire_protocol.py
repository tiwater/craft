def wrap_angle(angle):
    return atan2(sin(angle), cos(angle))


def square_error(yaw):
    return atan2(sin(4.0 * yaw), cos(4.0 * yaw)) / 4.0


def steps_left(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def move(sdk, position, yaw, gripper, steps, tolerance):
    available = steps_left(sdk)
    if available > 0:
        return sdk.move_to(
            position,
            yaw=yaw,
            gripper=gripper,
            max_steps=min(steps, available),
            tol=tolerance,
        )
    return {"reached": False}


def grip(sdk, action, steps):
    available = steps_left(sdk)
    if available > 0:
        sdk.set_gripper(action, steps=min(steps, available))


def grasp_yaw(current, desired):
    return wrap_angle(
        current
        + atan2(sin(2.0 * (desired - current)),
                cos(2.0 * (desired - current))) / 2.0
    )


def move_center(sdk, target, steps, tolerance, align=True):
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    eef = state["eef_pos"]
    rotation = -square_error(nut["yaw"]) if align else 0.0
    offset_x = nut["pos"][0] - eef[0]
    offset_y = nut["pos"][1] - eef[1]
    return move(
        sdk,
        [
            target[0] - cos(rotation) * offset_x + sin(rotation) * offset_y,
            target[1] - sin(rotation) * offset_x - cos(rotation) * offset_y,
            eef[2] + target[2] - nut["pos"][2],
        ],
        wrap_angle(state["eef_yaw"] + rotation),
        "close",
        steps,
        tolerance,
    )


def seated(state, rest_height):
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    quat = nut["quat"]
    return (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.007
        and abs(nut["pos"][2] - rest_height) < 0.012
        and abs(square_error(nut["yaw"])) < 0.10
        and quat[0] * quat[0] + quat[1] * quat[1] < 0.015
    )


def failure(sdk, reason):
    return {
        "status": "failed",
        "reason": reason,
        "steps_used": sdk.get_state()["steps_used"],
    }


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return failure(sdk, "Missing nut or goal pose")

    # Let the free nut settle before measuring its resting height.
    grip(sdk, "open", 8)
    state = sdk.get_state()
    rest_height = state["objects"]["nut"]["pos"][2]
    grasped = False

    candidates = [
        [0.052, pi / 2.0, 0.003],
        [0.047, pi / 2.0, -0.002],
        [0.052, 0.0, 0.001],
    ]

    for candidate in candidates:
        if steps_left(sdk) < 175:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        safe_height = max(state["eef_pos"][2], nut["pos"][2] + 0.085)
        move(
            sdk,
            [state["eef_pos"][0], state["eef_pos"][1], safe_height],
            state["eef_yaw"], "open", 12, 0.007,
        )

        for approach in range(2):
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            yaw = grasp_yaw(state["eef_yaw"], nut["yaw"] + candidate[1])
            handle = [
                nut["pos"][0] + candidate[0] * cos(nut["yaw"]),
                nut["pos"][1] + candidate[0] * sin(nut["yaw"]),
                safe_height,
            ]
            move(sdk, handle, yaw, "open", 16, 0.004)
            state = sdk.get_state()
            if hypot(
                state["eef_pos"][0] - handle[0],
                state["eef_pos"][1] - handle[1],
            ) < 0.007:
                break

        for descent in range(2):
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            target = [
                nut["pos"][0] + candidate[0] * cos(nut["yaw"]),
                nut["pos"][1] + candidate[0] * sin(nut["yaw"]),
                nut["pos"][2] + candidate[2],
            ]
            yaw = grasp_yaw(state["eef_yaw"], nut["yaw"] + candidate[1])
            move(sdk, target, yaw, "open", 16, 0.003)
            state = sdk.get_state()
            if (
                hypot(state["eef_pos"][0] - target[0],
                      state["eef_pos"][1] - target[1]) < 0.008
                and abs(state["eef_pos"][2] - target[2]) < 0.009
            ):
                break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        rest_height = min(rest_height, nut["pos"][2])
        before_height = nut["pos"][2]
        grip(sdk, "close", 12)
        state = sdk.get_state()
        eef = state["eef_pos"]
        lift_target = [eef[0], eef[1], eef[2] + 0.070]
        move(sdk, lift_target, state["eef_yaw"], "close", 22, 0.004)
        state = sdk.get_state()

        if state["objects"]["nut"]["pos"][2] > before_height + 0.035:
            grip(sdk, "close", 4)
            state = sdk.get_state()
            grasped = (
                state["objects"]["nut"]["pos"][2] > before_height + 0.032
                and state["gripper_width"] > 0.010
            )
            if grasped:
                break

        if state["objects"]["nut"]["pos"][2] > before_height + 0.012:
            nut = state["objects"]["nut"]
            move_center(
                sdk,
                [nut["pos"][0], nut["pos"][1], before_height + 0.003],
                16, 0.005, False,
            )
        grip(sdk, "open", 6)

    if not grasped:
        return failure(sdk, "Nut did not follow verification lift")

    # Four logged runs lifted and rotated the nut successfully, then stopped
    # before transport. Allow feedback corrections instead of treating a
    # short, unfinished clearance move as a lost grasp.
    state = sdk.get_state()
    goal = state["goal_pos"]
    clearance = max(rest_height + 0.125, goal[2] + 0.110)
    clear = False
    for correction in range(4):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        if nut["pos"][2] >= clearance - 0.010:
            clear = True
            break
        if steps_left(sdk) < 115:
            break
        move_center(
            sdk,
            [nut["pos"][0], nut["pos"][1], clearance + 0.004],
            14, 0.003, False,
        )

    if not clear:
        state = sdk.get_state()
        clear = state["objects"]["nut"]["pos"][2] >= clearance - 0.010
    if not clear:
        return failure(sdk, "Unable to lift nut above peg")

    state = sdk.get_state()
    goal = state["goal_pos"]
    move_center(sdk, [goal[0], goal[1], clearance], 40, 0.003)

    aligned = False
    for correction in range(7):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        if (
            hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.003
            and abs(square_error(nut["yaw"])) < 0.045
            and nut["pos"][2] >= clearance - 0.012
        ):
            aligned = True
            break
        if steps_left(sdk) < 65:
            break
        move_center(sdk, [goal[0], goal[1], clearance], 9, 0.001)

    if not aligned:
        return failure(sdk, "Alignment above peg not verified")

    stalled = 0
    recoveries = 0
    for segment in range(24):
        if steps_left(sdk) <= 23:
            break
        state = sdk.get_state()
        if seated(state, rest_height):
            break

        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        height = nut["pos"][2]
        lateral = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
        angular = abs(square_error(nut["yaw"]))

        if lateral > 0.007 or angular > 0.10 or stalled >= 3:
            if recoveries >= 1 or steps_left(sdk) < 75:
                break
            move_center(
                sdk, [nut["pos"][0], nut["pos"][1], clearance],
                20, 0.003, False,
            )
            state = sdk.get_state()
            if state["objects"]["nut"]["pos"][2] < clearance - 0.012:
                break
            goal = state["goal_pos"]
            move_center(sdk, [goal[0], goal[1], clearance], 14, 0.001)
            recoveries += 1
            stalled = 0
            continue

        drop = 0.025 if height > rest_height + 0.075 else 0.012
        move_center(
            sdk,
            [goal[0], goal[1], max(rest_height + 0.001, height - drop)],
            7, 0.001,
        )
        after_height = sdk.get_state()["objects"]["nut"]["pos"][2]
        stalled = stalled + 1 if height - after_height < 0.001 else 0

    state = sdk.get_state()
    if not seated(state, rest_height):
        return failure(sdk, "Nut did not reach seated pose")

    grip(sdk, "open", 10)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        sdk, [eef[0], eef[1], eef[2] + 0.080],
        state["eef_yaw"], "open", 12, 0.007,
    )
    state = sdk.get_state()
    success = seated(state, rest_height)
    return {
        "status": "done" if success else "failed",
        "reason": "Seated pose verified after release"
        if success else "Nut shifted after release",
        "steps_used": state["steps_used"],
    }
