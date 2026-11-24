def run_policy(sdk):
    def wrap(angle):
        return atan2(sin(angle), cos(angle))

    def square_error(angle):
        return (angle + pi / 4.0) % (pi / 2.0) - pi / 4.0

    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(position, yaw, grip, steps, tolerance=0.004):
        budget = min(steps, remaining())
        if budget <= 0:
            return False
        result = sdk.move_to(
            position, yaw=yaw, gripper=grip,
            max_steps=budget, tol=tolerance,
        )
        return result["reached"]

    def actuate(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def fail(reason):
        return {
            "status": "failed",
            "reason": reason,
            "steps_used": sdk.get_state()["steps_used"],
        }

    def grasp_angle(state):
        rotation = wrap(
            state["objects"]["nut"]["yaw"] - state["eef_yaw"]
        )
        if rotation > pi / 2.0:
            rotation -= pi
        if rotation < -pi / 2.0:
            rotation += pi
        return state["eef_yaw"] + rotation

    def align(height, steps, tolerance=0.003):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        eef = state["eef_pos"]
        return move(
            [
                eef[0] + goal[0] - nut["pos"][0],
                eef[1] + goal[1] - nut["pos"][1],
                eef[2] + height - nut["pos"][2],
            ],
            state["eef_yaw"] - square_error(nut["yaw"]),
            "close", steps, tolerance,
        )

    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return fail("Missing nut or peg pose")

    resting_height = state["objects"]["nut"]["pos"][2]
    held = False

    for attempt in range(3):
        if remaining() < 145:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        offset = 0.055 if attempt < 2 else 0.060
        height_offset = 0.008 if attempt == 0 else -0.004
        before_lift = nut["pos"][2]
        grasp_yaw = grasp_angle(state)
        clearance = max(state["eef_pos"][2], before_lift + 0.090)

        actuate("open", 5)
        state = sdk.get_state()
        if state["eef_pos"][2] < before_lift + 0.075:
            move(
                [state["eef_pos"][0], state["eef_pos"][1], clearance],
                grasp_yaw, "open", 12, 0.008,
            )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        move(
            [
                nut["pos"][0] + offset * cos(nut["yaw"]),
                nut["pos"][1] + offset * sin(nut["yaw"]),
                clearance,
            ],
            grasp_yaw, "open", 25, 0.005,
        )

        for correction in range(3):
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            grasp_yaw = grasp_angle(state)
            handle = [
                nut["pos"][0] + offset * cos(nut["yaw"]),
                nut["pos"][1] + offset * sin(nut["yaw"]),
                nut["pos"][2] + height_offset,
            ]
            move(
                handle, grasp_yaw, "open",
                19 if correction == 0 else 7, 0.003,
            )
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            lateral_error = hypot(
                state["eef_pos"][0]
                - nut["pos"][0] - offset * cos(nut["yaw"]),
                state["eef_pos"][1]
                - nut["pos"][1] - offset * sin(nut["yaw"]),
            )
            vertical_error = abs(
                state["eef_pos"][2] - nut["pos"][2] - height_offset
            )
            if lateral_error < 0.006 and vertical_error < 0.006:
                break

        state = sdk.get_state()
        before_lift = state["objects"]["nut"]["pos"][2]
        if lateral_error < 0.010 and vertical_error < 0.012:
            actuate("close", 11)
            state = sdk.get_state()
            target = list(state["eef_pos"])
            target[2] += 0.065
            move(target, state["eef_yaw"], "close", 17, 0.004)

            state = sdk.get_state()
            held = (
                state["objects"]["nut"]["pos"][2] > before_lift + 0.035
                and state["gripper_width"] > 0.010
            )
            if held:
                break

        state = sdk.get_state()
        lift = state["objects"]["nut"]["pos"][2] - before_lift
        if lift > 0.012:
            target = list(state["eef_pos"])
            target[2] -= max(0.0, lift - 0.003)
            move(target, state["eef_yaw"], "close", 10, 0.005)
        actuate("open", 5)

    if not held:
        return fail("Could not establish a lifting grasp")

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    clearance_height = max(
        resting_height + 0.145,
        state["goal_pos"][2] + 0.110,
    )
    target = list(state["eef_pos"])
    target[2] += max(0.0, clearance_height - nut["pos"][2])
    move(target, state["eef_yaw"], "close", 18, 0.005)

    state = sdk.get_state()
    move(
        list(state["eef_pos"]),
        state["eef_yaw"] - square_error(state["objects"]["nut"]["yaw"]),
        "close", 13, 0.003,
    )

    for iteration in range(5):
        if remaining() < 45:
            break
        align(clearance_height, 25 if iteration == 0 else 7)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        if (
            hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.003
            and abs(square_error(nut["yaw"])) < 0.035
        ):
            break

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    if hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) > 0.010:
        return fail("Could not center the nut above the peg")

    stalls = 0
    searches = 0

    for iteration in range(40):
        if remaining() <= 22:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        position = nut["pos"]
        goal = state["goal_pos"]
        lateral_error = hypot(position[0] - goal[0], position[1] - goal[1])
        yaw_error = abs(square_error(nut["yaw"]))

        if (
            position[2] <= resting_height + 0.009
            and lateral_error < 0.008
            and yaw_error < 0.10
        ):
            break

        if state["gripper_width"] < 0.009:
            return fail("Grasp lost during insertion")

        if stalls >= 2 and remaining() > 43:
            eef = state["eef_pos"]
            move(
                [eef[0], eef[1], eef[2] + 0.010],
                state["eef_yaw"], "close", 5, 0.003,
            )
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            goal = state["goal_pos"]
            eef = state["eef_pos"]
            radius = 0.0 if searches == 0 else 0.0015
            angle = (searches - 1) * pi / 2.0
            move(
                [
                    eef[0] + goal[0] - nut["pos"][0] + radius * cos(angle),
                    eef[1] + goal[1] - nut["pos"][1] + radius * sin(angle),
                    eef[2],
                ],
                state["eef_yaw"] - square_error(nut["yaw"]),
                "close", 6, 0.002,
            )
            searches += 1
            stalls = 0
            continue

        previous_height = position[2]
        if lateral_error > 0.004 or yaw_error > 0.060:
            next_height = previous_height
        else:
            descent = 0.030 if position[2] > resting_height + 0.080 else 0.012
            next_height = max(resting_height + 0.002, position[2] - descent)

        align(next_height, min(7, remaining() - 22), 0.002)
        state = sdk.get_state()
        current_height = state["objects"]["nut"]["pos"][2]

        if next_height < previous_height - 0.001:
            if previous_height - current_height < 0.0015:
                stalls += 1
            else:
                stalls = 0
        else:
            stalls += 1

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    if not (
        nut["pos"][2] <= resting_height + 0.012
        and hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.009
        and abs(square_error(nut["yaw"])) < 0.10
    ):
        return fail("Nut did not reach the seated height")

    actuate("open", 9)
    state = sdk.get_state()
    target = list(state["eef_pos"])
    target[2] += 0.085
    move(target, state["eef_yaw"], "open", 13, 0.008)

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    center_error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    success = (
        center_error < 0.012
        and abs(nut["pos"][2] - resting_height) < 0.018
        and abs(square_error(nut["yaw"])) < 0.12
        and state["gripper_width"] > 0.04
    )
    return {
        "status": "done" if success else "failed",
        "reason": "Seated and released" if success else "Release verification failed",
        "center_error_m": center_error,
        "steps_used": state["steps_used"],
    }
