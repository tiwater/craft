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
        position, yaw=yaw, gripper=gripper,
        max_steps=budget, tol=tolerance
    )
    return result["reached"]


def hold_gripper(sdk, action, steps):
    budget = min(steps, remaining_steps(sdk))
    if budget > 0:
        sdk.set_gripper(action, steps=budget)


def handle_position(nut):
    yaw = nut["yaw"]
    return [
        nut["pos"][0] + 0.055 * cos(yaw),
        nut["pos"][1] + 0.055 * sin(yaw),
        nut["pos"][2] + 0.004,
    ]


def move_nut_center(sdk, target, steps, tolerance):
    state = sdk.get_state()
    nut = state["objects"]["nut"]["pos"]
    eef = state["eef_pos"]
    return bounded_move(
        sdk,
        [
            eef[0] + target[0] - nut[0],
            eef[1] + target[1] - nut[1],
            eef[2] + target[2] - nut[2],
        ],
        state["eef_yaw"], "close", steps, tolerance
    )


def nut_normal(nut):
    q = nut["quat"]
    normal = [
        2.0 * (q[0] * q[2] + q[3] * q[1]),
        2.0 * (q[1] * q[2] - q[3] * q[0]),
        1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1]),
    ]
    if normal[2] < 0.0:
        normal = [-normal[0], -normal[1], -normal[2]]
    return normal


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    hold_gripper(sdk, "open", 16)
    state = sdk.get_state()
    rest_height = state["objects"]["nut"]["pos"][2]
    clearance = max(rest_height + 0.16, state["goal_pos"][2] + 0.12)
    grasped = False

    for attempt in range(2):
        if remaining_steps(sdk) < 190:
            break

        if attempt > 0:
            hold_gripper(sdk, "open", 15)

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        handle = handle_position(nut)
        travel_height = max(nut["pos"][2] + 0.09, state["eef_pos"][2])

        bounded_move(
            sdk, [handle[0], handle[1], travel_height],
            nut["yaw"], "open", 35, 0.005
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        handle = handle_position(nut)
        bounded_move(
            sdk, [handle[0], handle[1], handle[2] + 0.035],
            nut["yaw"], "open", 25, 0.004
        )

        contact_ready = False
        for correction in range(4):
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            handle = handle_position(nut)
            bounded_move(sdk, handle, nut["yaw"], "open", 15, 0.002)
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            handle = handle_position(nut)
            eef = state["eef_pos"]
            yaw_error = wrap_angle(state["eef_yaw"] - nut["yaw"])
            yaw_error = min(abs(yaw_error), abs(pi - abs(yaw_error)))
            contact_ready = (
                hypot(eef[0] - handle[0], eef[1] - handle[1]) < 0.006
                and abs(eef[2] - handle[2]) < 0.006
                and yaw_error < 0.10
                and state["gripper_width"] > 0.045
            )
            if contact_ready:
                break
            if remaining_steps(sdk) < 190:
                break

        if not contact_ready:
            contact_ready = (
                hypot(eef[0] - handle[0], eef[1] - handle[1]) < 0.006
                and abs(eef[2] - handle[2]) < 0.010
                and yaw_error < 0.15
                and state["gripper_width"] > 0.045
            )

        if not contact_ready:
            return {
                "status": "failed",
                "reason": "Grasp approach did not converge before closure",
            }

        before_lift = state["objects"]["nut"]["pos"][2]
        rest_height = before_lift
        hold_gripper(sdk, "close", 16)
        state = sdk.get_state()

        if state["gripper_width"] < 0.012:
            continue

        eef = state["eef_pos"]
        bounded_move(
            sdk, [eef[0], eef[1], eef[2] + 0.085],
            state["eef_yaw"], "close", 30, 0.004
        )
        state = sdk.get_state()
        lift_delta = state["objects"]["nut"]["pos"][2] - before_lift
        grasped = lift_delta > 0.04 and state["gripper_width"] > 0.012
        if grasped:
            break

    if not grasped:
        return {"status": "failed", "reason": "Nut did not follow the lift"}

    state = sdk.get_state()
    center = state["objects"]["nut"]["pos"]
    move_nut_center(sdk, [center[0], center[1], clearance], 25, 0.004)

    for iteration in range(3):
        state = sdk.get_state()
        error = square_yaw_error(state["objects"]["nut"]["yaw"])
        if abs(error) < 0.025:
            break
        bounded_move(
            sdk, list(state["eef_pos"]),
            wrap_angle(state["eef_yaw"] - error), "close", 12, 0.002
        )

    state = sdk.get_state()
    goal = state["goal_pos"]
    move_nut_center(sdk, [goal[0], goal[1], clearance], 35, 0.003)

    for iteration in range(2):
        state = sdk.get_state()
        error = square_yaw_error(state["objects"]["nut"]["yaw"])
        if abs(error) > 0.025:
            bounded_move(
                sdk, list(state["eef_pos"]),
                wrap_angle(state["eef_yaw"] - error), "close", 10, 0.002
            )
        state = sdk.get_state()
        goal = state["goal_pos"]
        move_nut_center(sdk, [goal[0], goal[1], clearance], 10, 0.002)

    state = sdk.get_state()
    if abs(square_yaw_error(state["objects"]["nut"]["yaw"])) > 0.10:
        return {"status": "failed", "reason": "Unable to align square hole"}

    approach_height = min(
        clearance, max(rest_height + 0.075, state["goal_pos"][2] + 0.065)
    )
    goal = state["goal_pos"]
    move_nut_center(sdk, [goal[0], goal[1], approach_height], 20, 0.003)

    stalled = 0
    tilt_repaired = False

    for iteration in range(12):
        if remaining_steps(sdk) < 28:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        center = nut["pos"]
        lateral_error = hypot(center[0] - goal[0], center[1] - goal[1])
        yaw_error = square_yaw_error(nut["yaw"])

        if (
            center[2] <= rest_height + 0.012
            and lateral_error < 0.006
            and abs(yaw_error) < 0.10
        ):
            break

        normal = nut_normal(nut)
        if (
            stalled >= 2
            and not tilt_repaired
            and (
                hypot(normal[0], normal[1]) > 0.12
                or abs(yaw_error) > 0.035
            )
            and remaining_steps(sdk) >= 80
        ):
            # Recover only after measured binding. Reserve time for seating
            # and release rather than spending it on repeated blocked descent.
            move_nut_center(
                sdk, [center[0], center[1], approach_height],
                16, 0.003
            )
            state = sdk.get_state()
            if state["objects"]["nut"]["pos"][2] >= approach_height - 0.008:
                for correction in range(10):
                    state = sdk.get_state()
                    normal = nut_normal(state["objects"]["nut"])
                    if hypot(normal[0], normal[1]) < 0.035:
                        break
                    sdk.step_raw(
                        [
                            0.0, 0.0, 0.0,
                            max(-0.25, min(0.25, 0.6 * normal[1])),
                            max(-0.25, min(0.25, -0.6 * normal[0])),
                            0.0, 1.0,
                        ],
                        steps=1
                    )
                state = sdk.get_state()
                bounded_move(
                    sdk, list(state["eef_pos"]),
                    wrap_angle(
                        state["eef_yaw"]
                        - square_yaw_error(state["objects"]["nut"]["yaw"])
                    ),
                    "close", 10, 0.002
                )
                state = sdk.get_state()
                goal = state["goal_pos"]
                move_nut_center(
                    sdk, [goal[0], goal[1], approach_height], 12, 0.002
                )
            tilt_repaired = True
            stalled = 0
            continue

        if abs(yaw_error) > 0.10 or lateral_error > 0.012:
            if remaining_steps(sdk) < 65:
                break
            move_nut_center(
                sdk, [center[0], center[1], approach_height], 12, 0.003
            )
            state = sdk.get_state()
            bounded_move(
                sdk, list(state["eef_pos"]),
                wrap_angle(
                    state["eef_yaw"]
                    - square_yaw_error(state["objects"]["nut"]["yaw"])
                ),
                "close", 10, 0.002
            )
            move_nut_center(
                sdk, [goal[0], goal[1], approach_height], 12, 0.002
            )
            stalled = 0
            continue

        if lateral_error > 0.0025:
            move_nut_center(
                sdk, [goal[0], goal[1], center[2]], 7, 0.0015
            )

        state = sdk.get_state()
        center = state["objects"]["nut"]["pos"]
        previous_height = center[2]
        move_nut_center(
            sdk,
            [goal[0], goal[1], max(rest_height + 0.002, center[2] - 0.014)],
            min(9, max(0, remaining_steps(sdk) - 24)), 0.002
        )
        state = sdk.get_state()
        if previous_height - state["objects"]["nut"]["pos"][2] < 0.003:
            stalled += 1
        else:
            stalled = 0

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

    if remaining_steps(sdk) < 20:
        return {"status": "failed", "reason": "Insufficient budget to release"}

    hold_gripper(sdk, "open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    bounded_move(
        sdk, [eef[0], eef[1], eef[2] + 0.09],
        state["eef_yaw"], "open", 18, 0.008
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
