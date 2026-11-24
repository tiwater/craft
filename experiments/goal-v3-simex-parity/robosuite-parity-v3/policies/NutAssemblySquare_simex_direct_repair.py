def run_policy(sdk):
    def square_error(angle):
        return (angle + pi / 4.0) % (pi / 2.0) - pi / 4.0

    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(position, yaw, grip, steps, tolerance=0.004):
        budget = max(0, min(steps, remaining()))
        if budget:
            return sdk.move_to(
                position, yaw=yaw, gripper=grip,
                max_steps=budget, tol=tolerance
            )
        return {"reached": False}

    def actuate(action, steps):
        budget = max(0, min(steps, remaining()))
        if budget:
            sdk.set_gripper(action, steps=budget)

    def fail(reason):
        return {
            "status": "failed",
            "reason": reason,
            "steps_used": sdk.get_state()["steps_used"],
        }

    def handle_pose(nut, height):
        return [
            nut["pos"][0] + 0.055 * cos(nut["yaw"]),
            nut["pos"][1] + 0.055 * sin(nut["yaw"]),
            nut["pos"][2] + height,
        ]

    def center_target(state, height, turn=0.0):
        nut = state["objects"]["nut"]["pos"]
        eef = state["eef_pos"]
        goal = state["goal_pos"]
        offset_x = eef[0] - nut[0]
        offset_y = eef[1] - nut[1]
        return [
            goal[0] + cos(turn) * offset_x - sin(turn) * offset_y,
            goal[1] + sin(turn) * offset_x + cos(turn) * offset_y,
            eef[2] + height - nut[2],
        ]

    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return fail("Missing nut or peg pose")

    resting_height = state["objects"]["nut"]["pos"][2]
    held = False

    for attempt in range(2):
        if remaining() < 185:
            break

        actuate("open", 5)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        grasp_yaw = state["eef_yaw"] + (
            (nut["yaw"] - state["eef_yaw"] + pi / 2.0) % pi
            - pi / 2.0
        )
        clearance = max(state["eef_pos"][2], nut["pos"][2] + 0.09)

        if state["eef_pos"][2] < nut["pos"][2] + 0.065:
            move(
                [state["eef_pos"][0], state["eef_pos"][1], clearance],
                state["eef_yaw"], "open", 18, 0.006
            )

        target = handle_pose(sdk.get_state()["objects"]["nut"], 0.018)
        target[2] = clearance
        move(target, grasp_yaw, "open", 38, 0.004)

        state = sdk.get_state()
        target = handle_pose(state["objects"]["nut"], 0.018)
        if hypot(
            state["eef_pos"][0] - target[0],
            state["eef_pos"][1] - target[1]
        ) > 0.010:
            target[2] = clearance
            move(target, grasp_yaw, "open", 16, 0.003)

        # Keep the finger center above the table contact region. A small
        # residual pose error is acceptable; the lifting test verifies grasp.
        grasp_height = 0.018 if attempt == 0 else 0.010
        state = sdk.get_state()
        target = handle_pose(state["objects"]["nut"], grasp_height)
        move(target, grasp_yaw, "open", 38, 0.004)

        state = sdk.get_state()
        target = handle_pose(state["objects"]["nut"], grasp_height)
        lateral = hypot(
            state["eef_pos"][0] - target[0],
            state["eef_pos"][1] - target[1]
        )
        vertical = state["eef_pos"][2] - target[2]
        if lateral > 0.008 or vertical > 0.012:
            move(target, grasp_yaw, "open", 18, 0.004)

        state = sdk.get_state()
        target = handle_pose(state["objects"]["nut"], grasp_height)
        lateral = hypot(
            state["eef_pos"][0] - target[0],
            state["eef_pos"][1] - target[1]
        )
        vertical = state["eef_pos"][2] - state["objects"]["nut"]["pos"][2]
        if lateral > 0.018 or vertical > 0.045:
            continue

        before_lift = state["objects"]["nut"]["pos"][2]
        actuate("close", 14)
        state = sdk.get_state()
        target = list(state["eef_pos"])
        target[2] += 0.115
        move(target, grasp_yaw, "close", 28, 0.005)

        state = sdk.get_state()
        held = (
            state["objects"]["nut"]["pos"][2] > before_lift + 0.05
            and state["gripper_width"] > 0.010
        )
        if held:
            break
        actuate("open", 5)

    if not held:
        return fail("Could not establish a lifting grasp")

    state = sdk.get_state()
    clearance_height = max(
        resting_height + 0.115,
        state["goal_pos"][2] + 0.075,
        state["objects"]["nut"]["pos"][2],
    )
    target = list(state["eef_pos"])
    target[2] += clearance_height - state["objects"]["nut"]["pos"][2]
    move(target, state["eef_yaw"], "close", 16, 0.004)

    state = sdk.get_state()
    error = square_error(state["objects"]["nut"]["yaw"])
    yaw_target = state["eef_yaw"] - error
    move(
        center_target(state, clearance_height, -error),
        yaw_target, "close", 36, 0.001
    )

    for correction in range(4):
        if remaining() < 105:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        error = square_error(nut["yaw"])
        lateral = hypot(
            nut["pos"][0] - state["goal_pos"][0],
            nut["pos"][1] - state["goal_pos"][1]
        )
        if abs(error) < 0.035 and lateral < 0.003:
            break

        pending = atan2(
            sin(yaw_target - state["eef_yaw"]),
            cos(yaw_target - state["eef_yaw"])
        )
        turn = pending
        if abs(pending) < 0.035:
            turn = -error
            yaw_target = state["eef_yaw"] + turn
        move(
            center_target(state, clearance_height, turn),
            yaw_target, "close", 14, 0.001
        )

    commanded_height = sdk.get_state()["objects"]["nut"]["pos"][2]
    stalled = 0
    seated = False

    for iteration in range(35):
        if remaining() <= 26:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        position = list(nut["pos"])
        lateral = hypot(position[0] - goal[0], position[1] - goal[1])
        error = square_error(nut["yaw"])

        if (
            position[2] <= resting_height + 0.009
            and lateral < 0.006
            and abs(error) < 0.075
        ):
            seated = True
            break

        if state["gripper_width"] < 0.010:
            return fail("Grasp lost during insertion")

        if abs(error) > 0.065 or lateral > 0.009:
            commanded_height = max(position[2], clearance_height)
            move(
                center_target(state, commanded_height),
                yaw_target, "close", min(16, remaining() - 26), 0.002
            )
            state = sdk.get_state()
            if state["objects"]["nut"]["pos"][2] < clearance_height - 0.010:
                continue
            error = square_error(state["objects"]["nut"]["yaw"])
            yaw_target = state["eef_yaw"] - error
            move(
                center_target(state, commanded_height, -error),
                yaw_target, "close", min(18, remaining() - 26), 0.001
            )
            stalled = 0
            continue

        if lateral > 0.0035:
            move(
                center_target(state, position[2]),
                yaw_target, "close", min(10, remaining() - 26), 0.001
            )
            continue

        if position[2] <= commanded_height + 0.005:
            drop = 0.025
            if position[2] < goal[2] + 0.035:
                drop = 0.012
            commanded_height = max(
                resting_height + 0.001, position[2] - drop
            )

        move(
            center_target(state, commanded_height),
            yaw_target, "close", min(12, remaining() - 26), 0.001
        )
        after = sdk.get_state()
        descent = position[2] - after["objects"]["nut"]["pos"][2]
        if commanded_height < position[2] - 0.006 and descent < 0.0015:
            stalled += 1
        else:
            stalled = 0

        if stalled >= 3 and remaining() > 75:
            commanded_height = clearance_height
            move(
                center_target(after, commanded_height),
                yaw_target, "close", 20, 0.002
            )
            state = sdk.get_state()
            if state["objects"]["nut"]["pos"][2] >= clearance_height - 0.010:
                error = square_error(state["objects"]["nut"]["yaw"])
                yaw_target = state["eef_yaw"] - error
                move(
                    center_target(state, commanded_height, -error),
                    yaw_target, "close", 16, 0.001
                )
            stalled = 0

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    seated = seated or (
        nut["pos"][2] <= resting_height + 0.009
        and hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.006
        and abs(square_error(nut["yaw"])) < 0.075
    )
    if not seated:
        return fail("Nut did not reach the seated height")

    actuate("open", 10)
    state = sdk.get_state()
    target = list(state["eef_pos"])
    target[2] += 0.085
    move(target, state["eef_yaw"], "open", 16, 0.008)

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    center_error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    success = (
        center_error < 0.010
        and abs(nut["pos"][2] - resting_height) < 0.015
        and abs(square_error(nut["yaw"])) < 0.10
        and state["gripper_width"] > 0.04
    )
    return {
        "status": "done" if success else "failed",
        "reason": "Seated and released" if success else "Release verification failed",
        "center_error_m": center_error,
        "steps_used": state["steps_used"],
    }
