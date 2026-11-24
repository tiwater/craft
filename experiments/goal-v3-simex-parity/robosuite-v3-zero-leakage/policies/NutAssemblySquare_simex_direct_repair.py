def run_policy(sdk):
    def wrap(angle):
        return atan2(sin(angle), cos(angle))

    def square_error(angle):
        return atan2(sin(4.0 * angle), cos(4.0 * angle)) / 4.0

    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(position, yaw, action, steps, tolerance=0.003):
        budget = min(max(0, steps), remaining())
        if budget > 0:
            return sdk.move_to(
                position, yaw=yaw, gripper=action,
                max_steps=budget, tol=tolerance
            )
        return {"reached": False}

    def grip(action, steps):
        budget = min(max(0, steps), remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def center_move(target, steps, tolerance=0.002):
        state = sdk.get_state()
        center = state["objects"]["nut"]["pos"]
        eef = state["eef_pos"]
        return move(
            [eef[i] + target[i] - center[i] for i in range(3)],
            state["eef_yaw"], "close", steps, tolerance
        )

    def align_center(target, steps):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        eef = state["eef_pos"]
        correction = -square_error(nut["yaw"])
        correction = max(-0.30, min(0.30, correction))
        dx = eef[0] - nut["pos"][0]
        dy = eef[1] - nut["pos"][1]
        move(
            [
                target[0] + cos(correction) * dx - sin(correction) * dy,
                target[1] + sin(correction) * dx + cos(correction) * dy,
                target[2] + eef[2] - nut["pos"][2],
            ],
            wrap(state["eef_yaw"] + correction),
            "close", steps, 0.002
        )

    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    rest_height = state["objects"]["nut"]["pos"][2]
    clearance = max(rest_height + 0.14, state["goal_pos"][2] + 0.12)
    grasped = False

    for attempt in range(3):
        if remaining() < 145:
            break

        state = sdk.get_state()
        eef = state["eef_pos"]
        move(
            [eef[0], eef[1], clearance],
            state["eef_yaw"], None, 16, 0.006
        )
        grip("open", 7)

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        offset = 0.055 if attempt < 2 else 0.045
        yaw = wrap(nut["yaw"] + (0.0 if attempt == 1 else pi / 2.0))
        move(
            [
                nut["pos"][0] + offset * cos(nut["yaw"]),
                nut["pos"][1] + offset * sin(nut["yaw"]),
                clearance,
            ],
            yaw, "open", 26, 0.003
        )

        for refinement in range(2):
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            yaw = wrap(nut["yaw"] + (0.0 if attempt == 1 else pi / 2.0))
            handle = [
                nut["pos"][0] + offset * cos(nut["yaw"]),
                nut["pos"][1] + offset * sin(nut["yaw"]),
                nut["pos"][2],
            ]
            move(handle, yaw, "open", 24 if refinement == 0 else 9, 0.002)
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            eef = state["eef_pos"]
            hx = nut["pos"][0] + offset * cos(nut["yaw"])
            hy = nut["pos"][1] + offset * sin(nut["yaw"])
            if (
                hypot(eef[0] - hx, eef[1] - hy) < 0.005
                and abs(eef[2] - nut["pos"][2]) < 0.007
            ):
                break

        before = list(sdk.get_state()["objects"]["nut"]["pos"])
        grip("close", 12)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move(
            [eef[0], eef[1], eef[2] + 0.06],
            state["eef_yaw"], "close", 22, 0.003
        )
        state = sdk.get_state()
        if (
            state["objects"]["nut"]["pos"][2] > before[2] + 0.03
            and state["gripper_width"] > 0.012
        ):
            grasped = True
            break

    if not grasped:
        return {
            "status": "failed",
            "reason": "Nut did not follow grasp",
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    center = state["objects"]["nut"]["pos"]
    center_move([center[0], center[1], clearance], 18, 0.004)

    for iteration in range(5):
        state = sdk.get_state()
        if abs(square_error(state["objects"]["nut"]["yaw"])) < 0.025:
            break
        center = state["objects"]["nut"]["pos"]
        align_center([center[0], center[1], clearance], 8)

    state = sdk.get_state()
    goal = state["goal_pos"]
    center_move([goal[0], goal[1], clearance], 30, 0.002)

    # Correct yaw about the nut center, preserving the grasp offset.
    for iteration in range(4):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        if (
            hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.002
            and abs(square_error(nut["yaw"])) < 0.025
        ):
            break
        if remaining() < 65:
            break
        align_center([goal[0], goal[1], clearance], 8)

    stalled = 0
    recoveries = 0
    released = False

    while remaining() > 25:
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        center = list(nut["pos"])
        goal = state["goal_pos"]
        lateral = hypot(center[0] - goal[0], center[1] - goal[1])
        angle = abs(square_error(nut["yaw"]))

        # Once the hole is engaged, let gravity finish the last few millimeters.
        if (
            lateral < 0.005
            and angle < 0.07
            and center[2] <= rest_height + 0.020
        ):
            grip("open", 10)
            released = True
            break

        if stalled >= 2 and recoveries < 2 and remaining() > 65:
            # Clear the contact locally instead of repeatedly returning
            # to the full transport height.
            recovery_height = min(clearance, center[2] + 0.035)
            center_move(
                [center[0], center[1], recovery_height], 10, 0.003
            )
            state = sdk.get_state()
            goal = state["goal_pos"]
            align_center([goal[0], goal[1], recovery_height], 10)
            center_move([goal[0], goal[1], recovery_height], 6, 0.001)
            recoveries += 1
            stalled = 0
            continue

        if angle > 0.045 and center[2] > rest_height + 0.065:
            align_center([goal[0], goal[1], center[2]], 8)
        elif lateral > 0.0015:
            center_move([goal[0], goal[1], center[2]], 6, 0.001)

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        center = list(nut["pos"])
        goal = state["goal_pos"]
        lateral = hypot(center[0] - goal[0], center[1] - goal[1])

        if lateral > 0.008:
            stalled += 1
            continue

        descent = 0.025 if center[2] > rest_height + 0.065 else 0.012
        target_z = max(rest_height + 0.002, center[2] - descent)
        center_move(
            [goal[0], goal[1], target_z],
            min(9, remaining() - 25), 0.0015
        )
        state = sdk.get_state()
        after = state["objects"]["nut"]["pos"]
        stalled = stalled + 1 if center[2] - after[2] < 0.002 else 0

        if stalled >= 4:
            break

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    ready = (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.005
        and nut["pos"][2] <= rest_height + 0.025
        and abs(square_error(nut["yaw"])) < 0.07
    )

    if not released and ready:
        grip("open", 10)
        released = True

    if not released:
        return {
            "status": "failed",
            "reason": "Insertion did not reach a safe release pose",
            "center_error_m": hypot(
                nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]
            ),
            "height_above_rest_m": nut["pos"][2] - rest_height,
            "yaw_error_rad": square_error(nut["yaw"]),
            "steps_used": state["steps_used"],
        }

    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.08],
        state["eef_yaw"], "open", 12, 0.006
    )
    grip("open", min(8, remaining()))

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    center_error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    seated = (
        center_error < 0.008
        and abs(nut["pos"][2] - rest_height) < 0.015
        and abs(square_error(nut["yaw"])) < 0.12
        and state["gripper_width"] > 0.05
    )
    return {
        "status": "done" if seated else "failed",
        "reason": "Released nut pose verified" if seated else "Release verification failed",
        "center_error_m": center_error,
        "height_above_rest_m": nut["pos"][2] - rest_height,
        "steps_used": state["steps_used"],
    }
