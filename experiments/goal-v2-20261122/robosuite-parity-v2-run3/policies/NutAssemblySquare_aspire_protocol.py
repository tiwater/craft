def wrap_angle(angle):
    return (angle + pi) % (2.0 * pi) - pi


def square_error(angle):
    return (angle + pi / 4.0) % (pi / 2.0) - pi / 4.0


def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def move(sdk, position, yaw, gripper, steps, tolerance=0.003):
    budget = min(steps, remaining_steps(sdk))
    if budget <= 0:
        return False
    result = sdk.move_to(
        position, yaw=yaw, gripper=gripper,
        max_steps=budget, tol=tolerance
    )
    return result["reached"]


def grip(sdk, action, steps):
    budget = min(steps, remaining_steps(sdk))
    if budget > 0:
        sdk.set_gripper(action, steps=budget)


def rotate(sdk, target, steps):
    state = sdk.get_state()
    budget = min(steps, remaining_steps(sdk))
    error = wrap_angle(target - state["eef_yaw"])
    if budget > 0 and abs(error) > 0.015:
        sdk.move_delta(
            [0.0, 0.0, 0.0], dyaw=error,
            gripper="close", steps=budget
        )


def center_error(state):
    position = state["objects"]["nut"]["pos"]
    goal = state["goal_pos"]
    return hypot(position[0] - goal[0], position[1] - goal[1])


def upright(nut):
    quat = nut["quat"]
    return 1.0 - 2.0 * (quat[0] * quat[0] + quat[1] * quat[1])


def seating_ok(state, resting_z):
    nut = state["objects"]["nut"]
    return (
        center_error(state) < 0.007
        and abs(nut["pos"][2] - resting_z) < 0.010
        and abs(square_error(nut["yaw"])) < 0.10
        and upright(nut) > 0.96
    )


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or goal"}

    resting_z = state["objects"]["nut"]["pos"][2]
    held = False

    # The previous rollouts ended with the nut still at table height.
    # Grasp lower on the outer handle and verify actual object motion.
    for attempt in range(3):
        if remaining_steps(sdk) < 165:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        offset = 0.055 if attempt < 2 else 0.060
        depth = -0.010 if attempt == 0 else -0.016
        jaw_yaw = nut["yaw"] + (pi / 2.0 if attempt == 1 else 0.0)
        target_yaw = state["eef_yaw"] + (
            (jaw_yaw - state["eef_yaw"] + pi / 2.0) % pi - pi / 2.0
        )
        clearance = max(nut["pos"][2] + 0.085, state["eef_pos"][2])

        grip(sdk, "open", 5)
        move(
            sdk,
            [state["eef_pos"][0], state["eef_pos"][1], clearance],
            state["eef_yaw"], "open", 12, 0.006
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        move(
            sdk,
            [
                nut["pos"][0] + offset * cos(nut["yaw"]),
                nut["pos"][1] + offset * sin(nut["yaw"]),
                clearance
            ],
            target_yaw, "open", 25, 0.003
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        move(
            sdk,
            [
                nut["pos"][0] + offset * cos(nut["yaw"]),
                nut["pos"][1] + offset * sin(nut["yaw"]),
                nut["pos"][2] + depth
            ],
            target_yaw, "open", 25, 0.002
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        target = [
            nut["pos"][0] + offset * cos(nut["yaw"]),
            nut["pos"][1] + offset * sin(nut["yaw"]),
            nut["pos"][2] + depth
        ]
        if hypot(
            state["eef_pos"][0] - target[0],
            state["eef_pos"][1] - target[1]
        ) > 0.005:
            move(sdk, target, target_yaw, "open", 8, 0.002)

        grip(sdk, "close", 12)
        state = sdk.get_state()
        before = list(state["objects"]["nut"]["pos"])
        hand_before = list(state["eef_pos"])

        move(
            sdk,
            [hand_before[0], hand_before[1], hand_before[2] + 0.055],
            state["eef_yaw"], "close", 18, 0.003
        )

        state = sdk.get_state()
        rise = state["objects"]["nut"]["pos"][2] - before[2]
        hand_rise = state["eef_pos"][2] - hand_before[2]
        held = (
            rise > 0.028
            and abs(rise - hand_rise) < 0.018
            and state["gripper_width"] > 0.010
        )
        if held:
            break

    if not held:
        grip(sdk, "open", 4)
        return {
            "status": "failed",
            "reason": "Handle grasp failed lift verification",
            "steps_used": sdk.get_state()["steps_used"]
        }

    state = sdk.get_state()
    safe_z = max(resting_z + 0.115, state["goal_pos"][2] + 0.080)
    nut = state["objects"]["nut"]
    eef = state["eef_pos"]
    move(
        sdk,
        [eef[0], eef[1], eef[2] + max(0.0, safe_z - nut["pos"][2])],
        state["eef_yaw"], "close", 18, 0.003
    )

    for correction in range(3):
        state = sdk.get_state()
        error = square_error(state["objects"]["nut"]["yaw"])
        if abs(error) < 0.025 or remaining_steps(sdk) < 100:
            break
        rotate(sdk, state["eef_yaw"] - error, 9)

    # Recompute the grasp offset from measured poses after every motion.
    for correction in range(3):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        eef = state["eef_pos"]
        goal = state["goal_pos"]
        if nut["pos"][2] < resting_z + 0.025:
            return {"status": "failed", "reason": "Nut slipped during transfer"}
        if center_error(state) < 0.003 and abs(nut["pos"][2] - safe_z) < 0.005:
            break
        if remaining_steps(sdk) < 65:
            break
        move(
            sdk,
            [
                eef[0] + goal[0] - nut["pos"][0],
                eef[1] + goal[1] - nut["pos"][1],
                eef[2] + safe_z - nut["pos"][2]
            ],
            state["eef_yaw"], "close",
            32 if correction == 0 else 9, 0.002
        )

    stalls = 0
    recoveries = 0

    for insertion in range(30):
        if remaining_steps(sdk) <= 22:
            break
        state = sdk.get_state()
        if seating_ok(state, resting_z):
            break

        nut = state["objects"]["nut"]
        position = nut["pos"]
        eef = state["eef_pos"]
        goal = state["goal_pos"]
        lateral = center_error(state)

        if lateral > 0.035:
            return {
                "status": "failed",
                "reason": "Nut not centered over peg",
                "center_error_m": lateral
            }

        if upright(nut) < 0.88 or stalls >= 2:
            if recoveries >= 2 or remaining_steps(sdk) < 65:
                break
            recoveries += 1
            move(
                sdk,
                [eef[0], eef[1], eef[2] + max(0.025, safe_z - position[2])],
                state["eef_yaw"], "close", 14, 0.003
            )
            state = sdk.get_state()
            rotate(
                sdk,
                state["eef_yaw"] - square_error(state["objects"]["nut"]["yaw"]),
                9
            )
            stalls = 0
            continue

        yaw_error = square_error(nut["yaw"])
        if position[2] > goal[2] + 0.045 and abs(yaw_error) > 0.03:
            rotate(sdk, state["eef_yaw"] - yaw_error, 8)
            continue

        # Descend in larger free-space increments, then gently near contact.
        drop = 0.020 if position[2] > goal[2] + 0.035 else 0.009
        drop = min(drop, max(0.0, position[2] - resting_z))
        if lateral > 0.006:
            drop = 0.0

        old_z = position[2]
        old_lateral = lateral
        move(
            sdk,
            [
                eef[0] + goal[0] - position[0],
                eef[1] + goal[1] - position[1],
                eef[2] - drop
            ],
            state["eef_yaw"], "close",
            min(8, remaining_steps(sdk) - 22), 0.0015
        )

        updated = sdk.get_state()
        progress = old_z - updated["objects"]["nut"]["pos"][2]
        lateral_progress = old_lateral - center_error(updated)
        if progress < 0.001 and lateral_progress < 0.0008:
            stalls += 1
        else:
            stalls = 0

    state = sdk.get_state()
    if not seating_ok(state, resting_z):
        return {
            "status": "failed",
            "reason": "Insertion not seated",
            "center_error_m": center_error(state),
            "height_error_m": state["objects"]["nut"]["pos"][2] - resting_z,
            "steps_used": state["steps_used"]
        }

    grip(sdk, "open", 8)
    state = sdk.get_state()
    move(
        sdk,
        [
            state["eef_pos"][0],
            state["eef_pos"][1],
            state["eef_pos"][2] + 0.080
        ],
        state["eef_yaw"], "open", 11, 0.005
    )
    grip(sdk, "open", 3)

    state = sdk.get_state()
    success = seating_ok(state, resting_z)
    return {
        "status": "done" if success else "failed",
        "reason": "Released seating verified" if success else "Released nut moved",
        "center_error_m": center_error(state),
        "height_error_m": state["objects"]["nut"]["pos"][2] - resting_z,
        "steps_used": state["steps_used"]
    }
