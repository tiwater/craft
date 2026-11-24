def angle_wrap(angle):
    return (angle + pi) % (2.0 * pi) - pi


def square_error(yaw):
    return (yaw + pi / 4.0) % (pi / 2.0) - pi / 4.0


def steps_left(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def policy_move(sdk, position, yaw, grip, ticks, tolerance=0.003):
    remaining = steps_left(sdk)
    if remaining <= 0:
        return False
    result = sdk.move_to(
        position,
        yaw=angle_wrap(yaw),
        gripper=grip,
        max_steps=min(ticks, remaining),
        tol=tolerance,
    )
    return result["reached"]


def policy_gripper(sdk, action, ticks):
    remaining = steps_left(sdk)
    if remaining > 0:
        sdk.set_gripper(action, steps=min(ticks, remaining))


def center_nut(sdk, target, ticks, align=True):
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    eef = state["eef_pos"]
    rotation = -square_error(nut["yaw"]) if align else 0.0
    rotation = max(-0.20, min(0.20, rotation))
    offset_x = eef[0] - nut["pos"][0]
    offset_y = eef[1] - nut["pos"][1]
    return policy_move(
        sdk,
        [
            target[0] + cos(rotation) * offset_x - sin(rotation) * offset_y,
            target[1] + sin(rotation) * offset_x + cos(rotation) * offset_y,
            target[2] + eef[2] - nut["pos"][2],
        ],
        state["eef_yaw"] + rotation,
        "close",
        ticks,
        0.0015,
    )


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    resting_height = state["objects"]["nut"]["pos"][2]
    clearance = max(resting_height + 0.13, state["goal_pos"][2] + 0.10)
    grasped = False

    for attempt in range(2):
        if steps_left(sdk) < 150:
            break

        policy_gripper(sdk, "open", 7)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        eef = state["eef_pos"]
        if eef[2] < nut["pos"][2] + 0.055:
            policy_move(
                sdk,
                [eef[0], eef[1], nut["pos"][2] + 0.080],
                state["eef_yaw"], "open", 18, 0.006,
            )

        handle_offset = 0.055 if attempt == 0 else 0.050
        grip_rotation = 0.0 if attempt == 0 else pi / 2.0
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        policy_move(
            sdk,
            [
                nut["pos"][0] + handle_offset * cos(yaw),
                nut["pos"][1] + handle_offset * sin(yaw),
                nut["pos"][2] + 0.075,
            ],
            yaw + grip_rotation, "open", 30, 0.005,
        )

        for correction in range(2):
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            yaw = nut["yaw"]
            policy_move(
                sdk,
                [
                    nut["pos"][0] + handle_offset * cos(yaw),
                    nut["pos"][1] + handle_offset * sin(yaw),
                    nut["pos"][2] - 0.003,
                ],
                yaw + grip_rotation, "open",
                20 if correction == 0 else 8, 0.003,
            )
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            eef = state["eef_pos"]
            if (
                hypot(
                    eef[0] - nut["pos"][0] - handle_offset * cos(nut["yaw"]),
                    eef[1] - nut["pos"][1] - handle_offset * sin(nut["yaw"]),
                ) < 0.009
                and abs(eef[2] - nut["pos"][2]) < 0.016
            ):
                break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        eef = state["eef_pos"]
        if (
            hypot(
                eef[0] - nut["pos"][0] - handle_offset * cos(nut["yaw"]),
                eef[1] - nut["pos"][1] - handle_offset * sin(nut["yaw"]),
            ) > 0.015
            or abs(eef[2] - nut["pos"][2]) > 0.025
        ):
            continue

        policy_gripper(sdk, "close", 18)
        state = sdk.get_state()
        initial_height = state["objects"]["nut"]["pos"][2]
        initial_eef_height = state["eef_pos"][2]
        eef = state["eef_pos"]
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.065],
            state["eef_yaw"], "close", 24, 0.004,
        )
        state = sdk.get_state()
        nut_rise = state["objects"]["nut"]["pos"][2] - initial_height
        eef_rise = state["eef_pos"][2] - initial_eef_height
        if nut_rise > 0.030 and abs(nut_rise - eef_rise) < 0.022:
            grasped = True
            break

    if not grasped:
        return {
            "status": "failed",
            "reason": "Nut did not follow the lift",
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    center_nut(sdk, [nut["pos"][0], nut["pos"][1], clearance], 20)
    state = sdk.get_state()
    goal = state["goal_pos"]
    center_nut(sdk, [goal[0], goal[1], clearance], 26)

    release_ready = False
    stalled = 0
    recoveries = 0
    contact_height = None
    correction_x = 0.0
    correction_y = 0.0
    previous_error = None
    previous_yaw_error = None

    for iteration in range(60):
        if steps_left(sdk) <= 30:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        position = nut["pos"]
        error_x = goal[0] - position[0]
        error_y = goal[1] - position[1]
        lateral_error = hypot(error_x, error_y)
        yaw_error = abs(square_error(nut["yaw"]))

        if lateral_error < 0.006 and position[2] <= resting_height + 0.014:
            release_ready = True
            break

        # Refresh both the grasp offset and yaw after every short motion.
        # Persistent lateral residuals receive a bounded integral correction.
        if previous_error is not None:
            if error_x * previous_error[0] <= 0.0:
                correction_x *= 0.4
            if error_y * previous_error[1] <= 0.0:
                correction_y *= 0.4
        correction_x = max(-0.008, min(0.008, correction_x + 0.3 * error_x))
        correction_y = max(-0.008, min(0.008, correction_y + 0.3 * error_y))
        previous_error = [error_x, error_y]

        above_contact = contact_height is None or position[2] > contact_height + 0.020
        aligned = lateral_error < 0.005 and yaw_error < 0.075
        target_height = position[2]

        if aligned:
            target_height = max(resting_height + 0.002, position[2] - 0.018)
        elif lateral_error < 0.010 and yaw_error < 0.12 and above_contact:
            target_height = max(resting_height + 0.035, position[2] - 0.008)

        # If alignment cannot improve near the peg, first disengage vertically.
        yaw_stuck = (
            previous_yaw_error is not None
            and yaw_error > 0.12
            and yaw_error >= previous_yaw_error - 0.008
        )
        if not above_contact and (lateral_error > 0.010 or yaw_stuck):
            center_nut(
                sdk,
                [position[0], position[1], min(clearance, contact_height + 0.035)],
                min(12, steps_left(sdk) - 30),
                False,
            )
            previous_yaw_error = None
            stalled = 0
            continue

        previous_yaw_error = yaw_error
        previous_height = position[2]
        descending = target_height < previous_height - 0.004
        center_nut(
            sdk,
            [goal[0] + correction_x, goal[1] + correction_y, target_height],
            min(8, steps_left(sdk) - 30),
            above_contact or yaw_error < 0.075,
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        current_height = nut["pos"][2]
        lateral_error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])

        if lateral_error < 0.006 and current_height <= resting_height + 0.014:
            release_ready = True
            break

        if descending and previous_height - current_height < 0.002:
            stalled += 1
        elif previous_height - current_height > 0.003:
            stalled = 0

        if stalled >= 2:
            if lateral_error < 0.005 and current_height < resting_height + 0.024:
                release_ready = True
                break

            contact_height = current_height
            if steps_left(sdk) > 60:
                center_nut(
                    sdk,
                    [
                        nut["pos"][0],
                        nut["pos"][1],
                        min(clearance, current_height + 0.035),
                    ],
                    min(12, steps_left(sdk) - 30),
                    False,
                )
                correction_x = 0.0
                correction_y = 0.0
                previous_error = None
                previous_yaw_error = None
                recoveries += 1
            stalled = 0

    if not release_ready:
        state = sdk.get_state()
        return {
            "status": "failed",
            "reason": "Nut has not reached a safe seated release height",
            "steps_used": state["steps_used"],
        }

    policy_gripper(sdk, "open", 10)
    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.080],
        state["eef_yaw"], "open", 16, 0.005,
    )
    policy_gripper(sdk, "open", 4)

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    settled = error < 0.010 and abs(nut["pos"][2] - resting_height) < 0.012
    return {
        "status": "done" if settled else "failed",
        "reason": "Released nut is seated" if settled else "Nut did not settle after release",
        "center_error_m": error,
        "steps_used": state["steps_used"],
    }
