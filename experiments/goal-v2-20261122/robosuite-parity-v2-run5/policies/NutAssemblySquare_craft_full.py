def angle_wrap(angle):
    return (angle + pi) % (2.0 * pi) - pi


def clamp(value, limit):
    return max(-limit, min(limit, value))


def remaining_steps(sdk):
    state = sdk.get_state()
    return min(400, state["max_steps"]) - state["steps_used"]


def policy_move(sdk, position, yaw, grip, ticks, tolerance=0.012):
    remaining = remaining_steps(sdk)
    if remaining <= 0:
        return False
    result = sdk.move_to(
        position,
        yaw=yaw,
        gripper=grip,
        max_steps=min(ticks, remaining),
        tol=tolerance,
    )
    return result["reached"] or result["pos_err_m"] <= tolerance


def policy_gripper(sdk, action, ticks):
    remaining = remaining_steps(sdk)
    if remaining > 0:
        sdk.set_gripper(action, steps=min(ticks, remaining))


def center_nut(sdk, target, ticks, tolerance=0.003, align_yaw=True):
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    eef = state["eef_pos"]
    correction = 0.0
    if align_yaw:
        error = (nut["yaw"] + pi / 4.0) % (pi / 2.0) - pi / 4.0
        correction = clamp(-error, 0.12)

    # Rotate the measured grasp offset with the commanded yaw, so
    # orientation correction does not sweep the hole away from the peg.
    offset_x = nut["pos"][0] - eef[0]
    offset_y = nut["pos"][1] - eef[1]
    rotated_x = cos(correction) * offset_x - sin(correction) * offset_y
    rotated_y = sin(correction) * offset_x + cos(correction) * offset_y
    position = [
        target[0] - rotated_x,
        target[1] - rotated_y,
        eef[2] + target[2] - nut["pos"][2],
    ]
    return policy_move(
        sdk, position, angle_wrap(state["eef_yaw"] + correction),
        "close", ticks, tolerance
    )


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    clearance = max(nut["pos"][2] + 0.17, goal[2] + 0.12)
    grasped = False

    for attempt in range(2):
        if remaining_steps(sdk) < 200:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        handle_offset = 0.055 if attempt == 0 else 0.060
        handle = [
            nut["pos"][0] + handle_offset * cos(yaw),
            nut["pos"][1] + handle_offset * sin(yaw),
            nut["pos"][2],
        ]

        policy_gripper(sdk, "open", 8)
        policy_move(
            sdk, [handle[0], handle[1], clearance],
            yaw, "open", 35, 0.015
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        handle = [
            nut["pos"][0] + handle_offset * cos(yaw),
            nut["pos"][1] + handle_offset * sin(yaw),
            nut["pos"][2],
        ]
        policy_move(sdk, handle, yaw, "open", 35, 0.012)

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        handle = [
            nut["pos"][0] + handle_offset * cos(yaw),
            nut["pos"][1] + handle_offset * sin(yaw),
            nut["pos"][2],
        ]
        eef = state["eef_pos"]
        grasp_error = sqrt(sum(
            (eef[axis] - handle[axis]) ** 2 for axis in range(3)
        ))
        if grasp_error > 0.006:
            policy_move(sdk, handle, yaw, "open", 12, 0.003)

        state = sdk.get_state()
        initial_height = state["objects"]["nut"]["pos"][2]
        policy_gripper(sdk, "close", 18)
        state = sdk.get_state()
        eef = state["eef_pos"]
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.09],
            state["eef_yaw"], "close", 25, 0.006
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        lift = nut["pos"][2] - initial_height
        if lift > 0.045:
            grasped = True
            break

        if lift > 0.025 and state["gripper_width"] >= 0.015:
            eef = state["eef_pos"]
            policy_move(
                sdk, [eef[0], eef[1], eef[2] + 0.025],
                state["eef_yaw"], "close", 10, 0.003
            )
            state = sdk.get_state()
            if state["objects"]["nut"]["pos"][2] > initial_height + 0.045:
                grasped = True
                break

        policy_gripper(sdk, "open", 6)

    if not grasped:
        return {"status": "failed", "reason": "Nut did not follow the lift"}

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    transport_height = max(nut["pos"][2], goal[2] + 0.100)

    # Establish clearance before translating or rotating over the peg.
    center_nut(
        sdk, [nut["pos"][0], nut["pos"][1], transport_height],
        min(18, max(0, remaining_steps(sdk) - 132)),
        0.003, False
    )

    seated = False
    descent_started = False
    lost_grasp_ticks = 0

    for iteration in range(400):
        state = sdk.get_state()
        remaining = min(400, state["max_steps"]) - state["steps_used"]
        if remaining <= 32:
            break

        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        position = nut["pos"]
        eef = state["eef_pos"]
        lateral_error = hypot(
            goal[0] - position[0], goal[1] - position[1]
        )
        yaw_error = abs(
            (nut["yaw"] + pi / 4.0) % (pi / 2.0) - pi / 4.0
        )

        # A dropped nut must not drive the arm farther away indefinitely.
        separation = sqrt(sum(
            (eef[axis] - position[axis]) ** 2 for axis in range(3)
        ))
        if separation > 0.14:
            lost_grasp_ticks += 1
        else:
            lost_grasp_ticks = 0
        if lost_grasp_ticks >= 3:
            return {
                "status": "failed",
                "reason": "Nut separated from gripper during transport",
                "center_error_m": lateral_error,
                "height_above_goal_m": position[2] - goal[2],
                "steps_used": state["steps_used"],
            }

        if (
            lateral_error <= 0.003
            and yaw_error <= 0.045
            and position[2] <= goal[2] + 0.020
        ):
            seated = True
            break

        aligned = lateral_error <= 0.003 and yaw_error <= 0.035
        if aligned:
            descent_started = True

        if not descent_started:
            target_height = transport_height
        elif lateral_error > 0.004 or yaw_error > 0.045:
            # Correct residual error at the current height; never abort
            # simply because a previous move left a 6-8 mm XY residual.
            target_height = position[2]
        else:
            # Short absolute-pose moves preserve tool orientation and
            # refresh the measured hole-to-gripper offset during descent.
            descent = 0.018 if position[2] > goal[2] + 0.065 else 0.008
            target_height = max(goal[2] + 0.015, position[2] - descent)

        before = state["steps_used"]
        center_nut(
            sdk, [goal[0], goal[1], target_height],
            min(10, remaining - 32), 0.003
        )

        state = sdk.get_state()
        if state["steps_used"] == before:
            # Handle an already-reached EEF target without spending the
            # entire outer loop on zero-tick calls.
            nut = state["objects"]["nut"]
            eef = state["eef_pos"]
            yaw_correction = clamp(
                -((nut["yaw"] + pi / 4.0) % (pi / 2.0) - pi / 4.0),
                0.12
            )
            policy_move(
                sdk,
                [
                    eef[0] + goal[0] - nut["pos"][0],
                    eef[1] + goal[1] - nut["pos"][1],
                    eef[2] + target_height - nut["pos"][2],
                ],
                angle_wrap(state["eef_yaw"] + yaw_correction),
                "close", min(6, remaining_steps(sdk) - 32), 0.001
            )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    yaw_error = abs((nut["yaw"] + pi / 4.0) % (pi / 2.0) - pi / 4.0)
    seated = seated or (
        error <= 0.003
        and yaw_error <= 0.045
        and nut["pos"][2] <= goal[2] + 0.020
    )
    if not seated:
        return {
            "status": "failed",
            "reason": "Insertion did not reach aligned release height",
            "center_error_m": error,
            "height_above_goal_m": nut["pos"][2] - goal[2],
            "steps_used": state["steps_used"],
        }

    policy_gripper(sdk, "open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.09],
        state["eef_yaw"], "open", 20, 0.006
    )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    settled = error < 0.008 and nut["pos"][2] <= goal[2] + 0.020
    return {
        "status": "done" if settled else "failed",
        "reason": "Released nut is seated" if settled else "Nut moved after release",
        "center_error_m": error,
        "height_above_goal_m": nut["pos"][2] - goal[2],
        "steps_used": state["steps_used"],
    }
