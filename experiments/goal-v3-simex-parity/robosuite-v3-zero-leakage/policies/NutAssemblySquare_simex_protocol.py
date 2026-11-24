def wrap_angle(angle):
    return atan2(sin(angle), cos(angle))


def square_yaw_error(angle):
    return 0.25 * atan2(sin(4.0 * angle), cos(4.0 * angle))


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


def nearest_grasp_yaw(state):
    yaw = wrap_angle(state["objects"]["nut"]["yaw"] + pi / 2.0)
    opposite = wrap_angle(yaw + pi)
    if abs(wrap_angle(opposite - state["eef_yaw"])) < abs(
        wrap_angle(yaw - state["eef_yaw"])
    ):
        return opposite
    return yaw


def nut_normal(nut):
    q = nut["quat"]
    normal = [
        2.0 * (q[0] * q[2] + q[3] * q[1]),
        2.0 * (q[1] * q[2] - q[3] * q[0]),
        1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1]),
    ]
    if normal[2] < 0.0:
        return [-normal[0], -normal[1], -normal[2]]
    return normal


def move_center(sdk, target, steps, tolerance, yaw=None):
    state = sdk.get_state()
    center = state["objects"]["nut"]["pos"]
    eef = state["eef_pos"]
    if yaw is None:
        yaw = state["eef_yaw"]
    rotation = wrap_angle(yaw - state["eef_yaw"])
    offset_x = eef[0] - center[0]
    offset_y = eef[1] - center[1]
    return bounded_move(
        sdk,
        [
            target[0] + cos(rotation) * offset_x - sin(rotation) * offset_y,
            target[1] + sin(rotation) * offset_x + cos(rotation) * offset_y,
            target[2] + eef[2] - center[2],
        ],
        yaw,
        "close",
        steps,
        tolerance,
    )


def align_center(sdk, target, steps, reserve, yaw_bias=0.0):
    start = sdk.get_state()["steps_used"]
    while remaining_steps(sdk) > reserve:
        state = sdk.get_state()
        used = state["steps_used"] - start
        if used >= steps:
            break
        nut = state["objects"]["nut"]
        center = nut["pos"]
        error = hypot(center[0] - target[0], center[1] - target[1])
        yaw_error = square_yaw_error(nut["yaw"] - yaw_bias)
        if (
            error < 0.0018
            and abs(center[2] - target[2]) < 0.0025
            and abs(yaw_error) < 0.018
        ):
            return True
        budget = min(6, steps - used, remaining_steps(sdk) - reserve)
        if budget <= 0:
            break
        move_center(
            sdk,
            target,
            budget,
            0.0008,
            wrap_angle(state["eef_yaw"] - yaw_error),
        )
        if sdk.get_state()["steps_used"] == state["steps_used"]:
            sdk.step_raw([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], steps=1)
    return False


def level_nut(sdk, reserve):
    for iteration in range(5):
        if remaining_steps(sdk) <= reserve + 2:
            break
        state = sdk.get_state()
        normal = nut_normal(state["objects"]["nut"])
        tilt = hypot(normal[0], normal[1])
        if tilt < 0.035:
            break
        sdk.step_raw(
            [
                0.0,
                0.0,
                0.0,
                max(-0.16, min(0.16, 0.65 * normal[1])),
                max(-0.16, min(0.16, -0.65 * normal[0])),
                0.0,
                1.0,
            ],
            steps=2,
        )


def is_seated(state, rest_height, tolerance):
    nut = state["objects"]["nut"]
    center = nut["pos"]
    goal = state["goal_pos"]
    normal = nut_normal(nut)
    return (
        hypot(center[0] - goal[0], center[1] - goal[1]) < tolerance
        and abs(center[2] - rest_height) < 0.012
        and abs(square_yaw_error(nut["yaw"])) < 0.10
        and hypot(normal[0], normal[1]) < 0.15
    )


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    rest_height = state["objects"]["nut"]["pos"][2]
    clearance = max(rest_height + 0.145, state["goal_pos"][2] + 0.125)
    grasped = False

    # Pinch the handle near its root and near its thickness midplane.
    # Refresh both handle position and jaw yaw after every approach segment.
    for attempt in range(2):
        if remaining_steps(sdk) < 160:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        offset = 0.050 if attempt == 0 else 0.057
        yaw = nearest_grasp_yaw(state)
        bounded_move(
            sdk,
            [
                nut["pos"][0] + offset * cos(nut["yaw"]),
                nut["pos"][1] + offset * sin(nut["yaw"]),
                max(nut["pos"][2] + 0.090, state["eef_pos"][2]),
            ],
            yaw,
            "open",
            28,
            0.004,
        )

        ready = False
        for descent in range(7):
            if remaining_steps(sdk) < 130:
                break
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            yaw = nearest_grasp_yaw(state)
            handle = [
                nut["pos"][0] + offset * cos(nut["yaw"]),
                nut["pos"][1] + offset * sin(nut["yaw"]),
                nut["pos"][2] + 0.006,
            ]
            lateral = hypot(
                state["eef_pos"][0] - handle[0],
                state["eef_pos"][1] - handle[1],
            )
            yaw_error = abs(wrap_angle(yaw - state["eef_yaw"]))
            target = list(handle)
            if lateral > 0.012 or yaw_error > 0.16:
                target[2] += 0.040
            bounded_move(sdk, target, yaw, "open", 7, 0.002)
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            lateral = hypot(
                state["eef_pos"][0] - nut["pos"][0] - offset * cos(nut["yaw"]),
                state["eef_pos"][1] - nut["pos"][1] - offset * sin(nut["yaw"]),
            )
            if (
                lateral < 0.006
                and -0.004 < state["eef_pos"][2] - nut["pos"][2] < 0.016
                and abs(wrap_angle(nearest_grasp_yaw(state) - state["eef_yaw"])) < 0.10
            ):
                ready = True
                break

        if not ready:
            continue

        before_height = sdk.get_state()["objects"]["nut"]["pos"][2]
        hold_gripper(sdk, "close", 12)
        state = sdk.get_state()
        eef = state["eef_pos"]
        bounded_move(
            sdk,
            [eef[0], eef[1], eef[2] + 0.080],
            state["eef_yaw"],
            "close",
            20,
            0.003,
        )
        state = sdk.get_state()
        grasped = (
            state["objects"]["nut"]["pos"][2] > before_height + 0.040
            and state["gripper_width"] > 0.010
        )
        if grasped:
            break
        hold_gripper(sdk, "open", 5)

    if not grasped:
        return {"status": "failed", "reason": "Handle grasp did not lift nut"}

    state = sdk.get_state()
    center = state["objects"]["nut"]["pos"]
    move_center(sdk, [center[0], center[1], clearance], 16, 0.003)
    level_nut(sdk, 125)
    state = sdk.get_state()
    goal = state["goal_pos"]
    align_center(sdk, [goal[0], goal[1], clearance], 42, 100)

    # Descend in short increments. A plateau triggers a complete unloading
    # of the peg contact before leveling and retrying the square-hole entry.
    stalled = 0
    recoveries = 0
    for iteration in range(48):
        if remaining_steps(sdk) <= 24:
            break
        state = sdk.get_state()
        if is_seated(state, rest_height, 0.006):
            break
        if state["gripper_width"] <= 0.010:
            return {"status": "failed", "reason": "Nut lost during insertion"}

        nut = state["objects"]["nut"]
        center = nut["pos"]
        goal = state["goal_pos"]
        lateral = hypot(center[0] - goal[0], center[1] - goal[1])
        yaw_error = square_yaw_error(nut["yaw"])
        normal = nut_normal(nut)
        tilt = hypot(normal[0], normal[1])

        if stalled >= 2 or lateral > 0.007 or abs(yaw_error) > 0.075 or tilt > 0.12:
            if recoveries >= 3 or remaining_steps(sdk) < 65:
                break
            retry_height = max(clearance, center[2] + 0.028)
            move_center(
                sdk,
                [center[0], center[1], retry_height],
                10,
                0.002,
            )
            level_nut(sdk, 40)
            align_center(
                sdk,
                [goal[0], goal[1], retry_height],
                18,
                34,
            )
            recoveries += 1
            stalled = 0
            continue

        # Correct planar and angular errors without adding downward load.
        if lateral > 0.0025 or abs(yaw_error) > 0.025:
            align_center(
                sdk,
                [goal[0], goal[1], center[2]],
                8,
                24,
            )
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            center = nut["pos"]
            yaw_error = square_yaw_error(nut["yaw"])

        before_height = center[2]
        before_steps = state["steps_used"]
        increment = 0.010 if center[2] > clearance - 0.018 else 0.006
        move_center(
            sdk,
            [goal[0], goal[1], max(rest_height, center[2] - increment)],
            min(6, remaining_steps(sdk) - 24),
            0.0008,
            wrap_angle(state["eef_yaw"] - yaw_error),
        )
        state = sdk.get_state()
        if state["steps_used"] == before_steps and remaining_steps(sdk) > 24:
            sdk.step_raw([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], steps=1)
            state = sdk.get_state()
        progress = before_height - state["objects"]["nut"]["pos"][2]
        stalled = stalled + 1 if progress < 0.0012 else 0

    state = sdk.get_state()
    if not is_seated(state, rest_height, 0.007):
        return {
            "status": "failed",
            "reason": "Insertion not seated",
            "steps_used": state["steps_used"],
        }

    hold_gripper(sdk, "open", 10)
    state = sdk.get_state()
    eef = state["eef_pos"]
    bounded_move(
        sdk,
        [eef[0], eef[1], eef[2] + 0.075],
        state["eef_yaw"],
        "open",
        min(14, remaining_steps(sdk)),
        0.005,
    )
    state = sdk.get_state()
    center = state["objects"]["nut"]["pos"]
    goal = state["goal_pos"]
    seated = is_seated(state, rest_height, 0.008)
    released = state["gripper_width"] > 0.050
    return {
        "status": "done" if seated and released else "failed",
        "reason": "Released nut seated" if seated and released else "Release verification failed",
        "center_error_m": hypot(center[0] - goal[0], center[1] - goal[1]),
        "steps_used": state["steps_used"],
    }
