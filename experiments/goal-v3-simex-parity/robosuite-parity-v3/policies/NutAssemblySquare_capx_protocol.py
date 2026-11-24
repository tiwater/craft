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
        if budget > 0:
            sdk.move_to(
                position,
                yaw=yaw,
                gripper=grip,
                max_steps=budget,
                tol=tolerance,
            )

    def actuate(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def finish(reason):
        return {
            "status": "failed",
            "reason": reason,
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return finish("Missing nut or peg pose")

    resting_height = state["objects"]["nut"]["pos"][2]
    held = False

    for attempt in range(2):
        if remaining() < 180:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        grasp_yaw = state["eef_yaw"] + wrap(yaw - state["eef_yaw"])
        handle_offset = 0.055
        clearance = nut["pos"][2] + 0.095

        actuate("open", 6)
        move(
            [
                nut["pos"][0] + handle_offset * cos(yaw),
                nut["pos"][1] + handle_offset * sin(yaw),
                clearance,
            ],
            grasp_yaw, "open", 60, 0.006,
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        grasp_yaw = state["eef_yaw"] + wrap(yaw - state["eef_yaw"])
        grasp_target = [
            nut["pos"][0] + handle_offset * cos(yaw),
            nut["pos"][1] + handle_offset * sin(yaw),
            nut["pos"][2] + 0.003,
        ]
        before_lift = nut["pos"][2]

        # Allow the controller to finish settling after a long descent.
        move(grasp_target, grasp_yaw, "open", 55, 0.003)
        state = sdk.get_state()
        error = sqrt(sum(
            (state["eef_pos"][axis] - grasp_target[axis]) ** 2
            for axis in range(3)
        ))
        if error > 0.006 and remaining() > 210:
            move(grasp_target, grasp_yaw, "open", 20, 0.003)

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        handle = [
            nut["pos"][0] + handle_offset * cos(nut["yaw"]),
            nut["pos"][1] + handle_offset * sin(nut["yaw"]),
            nut["pos"][2] + 0.003,
        ]
        if (
            hypot(state["eef_pos"][0] - handle[0],
                  state["eef_pos"][1] - handle[1]) > 0.010
            or abs(state["eef_pos"][2] - handle[2]) > 0.010
        ):
            continue

        actuate("close", 12)
        state = sdk.get_state()
        target = list(state["eef_pos"])
        target[2] += 0.110
        move(target, grasp_yaw, "close", 38, 0.005)

        state = sdk.get_state()
        held = (
            state["objects"]["nut"]["pos"][2] > before_lift + 0.055
            and state["gripper_width"] > 0.012
        )
        if held:
            break

        target = list(state["eef_pos"])
        target[2] = before_lift + 0.008
        move(target, state["eef_yaw"], "close", 30, 0.005)
        actuate("open", 6)

    if not held:
        return finish("Could not establish a lifting grasp")

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    clearance = max(resting_height + 0.135, state["goal_pos"][2] + 0.100)
    target = list(state["eef_pos"])
    target[2] += max(0.0, clearance - nut["pos"][2])
    move(
        target,
        state["eef_yaw"] - square_error(nut["yaw"]),
        "close", 30, 0.004,
    )

    # Measure the actual grasp offset after rotating the nut.
    for iteration in range(3):
        if remaining() < 65:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        target = [
            state["eef_pos"][0] + goal[0] - nut["pos"][0],
            state["eef_pos"][1] + goal[1] - nut["pos"][1],
            state["eef_pos"][2] + clearance - nut["pos"][2],
        ]
        move(
            target,
            state["eef_yaw"] - square_error(nut["yaw"]),
            "close", 55 if iteration == 0 else 16, 0.002,
        )
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        if (
            hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.003
            and abs(square_error(nut["yaw"])) < 0.045
        ):
            break

    stalls = 0
    seated = False

    for iteration in range(18):
        if remaining() <= 24:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        position = nut["pos"]
        lateral_error = hypot(position[0] - goal[0], position[1] - goal[1])

        if position[2] < resting_height + 0.010 and lateral_error < 0.008:
            seated = True
            break

        if state["gripper_width"] < 0.010:
            return finish("Grasp lost during insertion")

        next_height = position[2]
        if lateral_error < 0.006 and abs(square_error(nut["yaw"])) < 0.07:
            next_height = max(resting_height + 0.002, position[2] - 0.028)
        if stalls >= 2:
            next_height = position[2] + 0.012
            stalls = 0

        target = [
            state["eef_pos"][0] + goal[0] - position[0],
            state["eef_pos"][1] + goal[1] - position[1],
            state["eef_pos"][2] + next_height - position[2],
        ]
        move(
            target,
            state["eef_yaw"] - square_error(nut["yaw"]),
            "close", min(12, remaining() - 24), 0.002,
        )
        current_height = sdk.get_state()["objects"]["nut"]["pos"][2]
        if next_height < position[2] and position[2] - current_height < 0.002:
            stalls += 1
        else:
            stalls = 0

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    seated = (
        nut["pos"][2] <= resting_height + 0.012
        and hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.008
        and abs(square_error(nut["yaw"])) < 0.10
    )
    if not seated:
        return finish("Nut did not reach the seated height")

    actuate("open", 10)
    state = sdk.get_state()
    target = list(state["eef_pos"])
    target[2] += 0.080
    move(target, state["eef_yaw"], "open", min(20, remaining()), 0.006)

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    center_error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    success = (
        center_error < 0.010
        and abs(nut["pos"][2] - resting_height) < 0.015
        and abs(square_error(nut["yaw"])) < 0.12
        and state["gripper_width"] > 0.04
    )
    return {
        "status": "done" if success else "failed",
        "reason": "Seated and released" if success else "Release verification failed",
        "center_error_m": center_error,
        "steps_used": state["steps_used"],
    }
