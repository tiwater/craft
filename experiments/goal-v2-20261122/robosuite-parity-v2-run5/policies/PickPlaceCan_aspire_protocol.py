def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, action, ticks, tolerance=0.012):
        budget = min(ticks, remaining())
        if budget > 0:
            return sdk.move_to(
                pos, yaw=yaw, gripper=action,
                max_steps=budget, tol=tolerance
            )
        return {"reached": False}

    def grip(action, ticks):
        budget = min(ticks, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def finish(reason, placed=False):
        return {
            "status": "done" if placed else "failed",
            "reason": reason,
            "steps_used": sdk.get_state()["steps_used"]
        }

    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return finish("Missing can or target")

    yaw = state["eef_yaw"]
    held = False
    offset = None
    initial_z = state["objects"]["can"]["pos"][2]

    for attempt in range(2):
        if remaining() < 170:
            break

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        goal = state["goal_pos"]
        approach_z = max(can[2] + 0.13, goal[2] + 0.12)

        move(
            [eef[0], eef[1], max(eef[2], approach_z)],
            yaw, "open", 18, 0.018
        )
        move([can[0], can[1], approach_z], yaw, "open", 32, 0.010)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.015:
            move([can[0], can[1], approach_z], yaw, "open", 16, 0.008)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        grasp_z = can[2] + 0.005 + attempt * 0.006
        move([can[0], can[1], grasp_z], yaw, "open", 30, 0.006)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - can[0], eef[1] - can[1]) > 0.018
            or abs(eef[2] - grasp_z) > 0.022
        ):
            continue

        initial_z = can[2]
        grip("close", 12)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move(
            [eef[0], eef[1], eef[2] + 0.10],
            yaw, "close", 24, 0.012
        )

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        held = (
            can[2] > initial_z + 0.05
            and hypot(can[0] - eef[0], can[1] - eef[1]) < 0.055
            and abs(can[2] - eef[2]) < 0.10
            and state["gripper_width"] > 0.012
        )
        if held:
            offset = [eef[i] - can[i] for i in range(3)]
            break
        grip("open", 8)

    if not held:
        return finish("Could not verify a lifted grasp")

    state = sdk.get_state()
    goal = state["goal_pos"]
    travel_z = max(initial_z + 0.12, goal[2] + 0.12)
    eef = state["eef_pos"]

    move(
        [eef[0], eef[1], travel_z + offset[2]],
        yaw, "close", min(22, max(0, remaining() - 90)), 0.012
    )

    state = sdk.get_state()
    if state["objects"]["can"]["pos"][2] < travel_z - 0.025:
        return finish("Insufficient clearance for transport")

    # Longer moves and relaxed intermediate tolerances avoid spending
    # the transport budget settling at many short waypoints.
    previous_distance = None
    aligned = False
    for segment in range(10):
        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        goal = state["goal_pos"]
        current_offset = [eef[i] - can[i] for i in range(3)]

        if (
            hypot(
                current_offset[0] - offset[0],
                current_offset[1] - offset[1]
            ) > 0.045
            or abs(current_offset[2] - offset[2]) > 0.05
            or state["gripper_width"] < 0.01
        ):
            return finish("Can slipped during transport")

        distance = hypot(goal[0] - can[0], goal[1] - can[1])
        if distance < 0.025:
            aligned = True
            break

        budget = min(28, max(0, remaining() - 65))
        if budget == 0:
            break

        # Reduce excessive reach height if horizontal progress stalls,
        # while retaining clearance above the bin walls.
        if previous_distance is not None:
            if previous_distance - distance < 0.012:
                travel_z = max(
                    initial_z + 0.105,
                    goal[2] + 0.105,
                    travel_z - 0.015
                )
        previous_distance = distance
        scale = min(1.0, 0.18 / max(distance, 0.001))

        move(
            [
                eef[0] + scale * (goal[0] - can[0]),
                eef[1] + scale * (goal[1] - can[1]),
                travel_z + current_offset[2]
            ],
            yaw, "close", budget,
            0.018 if distance > 0.18 else 0.010
        )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    aligned = hypot(can[0] - goal[0], can[1] - goal[1]) < 0.03
    if not aligned:
        return finish("Transport did not reach the target")

    for segment in range(4):
        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        goal = state["goal_pos"]
        if (
            hypot(can[0] - goal[0], can[1] - goal[1]) < 0.025
            and goal[2] - 0.025 < can[2] < goal[2] + 0.065
        ):
            break

        budget = min(16, max(0, remaining() - 30))
        if budget == 0:
            break

        move(
            [
                eef[0] + goal[0] - can[0],
                eef[1] + goal[1] - can[1],
                eef[2] + max(-0.065, goal[2] + 0.045 - can[2])
            ],
            yaw, "close", budget, 0.010
        )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) >= 0.03
        or not goal[2] - 0.035 < can[2] < goal[2] + 0.085
        or remaining() < 25
    ):
        return finish("Could not reach a safe release position")

    grip("open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.15],
        yaw, "open", min(22, max(0, remaining() - 6)), 0.015
    )
    grip("open", min(10, remaining()))

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    eef = state["eef_pos"]
    placed = (
        hypot(can[0] - goal[0], can[1] - goal[1]) < 0.06
        and goal[2] - 0.045 < can[2] < goal[2] + 0.08
        and state["gripper_width"] > 0.055
        and eef[2] - can[2] > 0.10
    )
    return finish(
        "Can released in target bin" if placed else "Placement verification failed",
        placed
    )
