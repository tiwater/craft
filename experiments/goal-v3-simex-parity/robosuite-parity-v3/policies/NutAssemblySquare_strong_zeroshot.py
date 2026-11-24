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
            position,
            yaw=yaw,
            gripper=grip,
            max_steps=budget,
            tol=tolerance,
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

    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return fail("Missing nut or peg pose")

    nut = state["objects"]["nut"]
    resting_height = nut["pos"][2]
    held = False

    # The square nut's grasping handle extends along its local positive X axis.
    for attempt in range(2):
        if remaining() < 150:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        position = nut["pos"]
        yaw = nut["yaw"]
        handle_offset = 0.055 if attempt == 0 else 0.065
        handle_x = position[0] + handle_offset * cos(yaw)
        handle_y = position[1] + handle_offset * sin(yaw)
        grasp_yaw = state["eef_yaw"] + wrap(yaw - state["eef_yaw"])
        clearance = max(state["eef_pos"][2], position[2] + 0.105)

        actuate("open", 8)
        move(
            [state["eef_pos"][0], state["eef_pos"][1], clearance],
            grasp_yaw, "open", 18,
        )
        if not move(
            [handle_x, handle_y, clearance],
            grasp_yaw, "open", 30,
        ):
            continue

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        handle_x = nut["pos"][0] + handle_offset * cos(yaw)
        handle_y = nut["pos"][1] + handle_offset * sin(yaw)
        before_lift = nut["pos"][2]

        if not move(
            [handle_x, handle_y, before_lift + 0.003],
            grasp_yaw, "open", 25, 0.003,
        ):
            continue

        actuate("close", 12)
        state = sdk.get_state()
        lift_target = list(state["eef_pos"])
        lift_target[2] += 0.105
        move(lift_target, grasp_yaw, "close", 28)

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        held = (
            nut["pos"][2] > before_lift + 0.055
            and state["gripper_width"] > 0.012
        )
        if held:
            break

        # Open only after returning a possibly loose grasp toward the table.
        position = list(state["eef_pos"])
        position[2] = before_lift + 0.012
        move(position, grasp_yaw, "close", 18)
        actuate("open", 8)

    if not held:
        return fail("Could not establish a lifting grasp")

    # Align the hole with the world-aligned square peg, using square symmetry.
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    clearance_height = max(
        resting_height + 0.145, goal[2] + 0.105, nut["pos"][2]
    )
    target = list(state["eef_pos"])
    target[2] += clearance_height - nut["pos"][2]
    move(target, state["eef_yaw"], "close", 22)

    state = sdk.get_state()
    correction = -square_error(state["objects"]["nut"]["yaw"])
    move(
        list(state["eef_pos"]),
        state["eef_yaw"] + correction,
        "close", 22,
    )

    # Recompute the grasp offset after rotation instead of assuming a rigid,
    # perfectly centered grasp.
    for iteration in range(3):
        if remaining() < 65:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        eef = state["eef_pos"]
        target = [
            eef[0] + goal[0] - nut["pos"][0],
            eef[1] + goal[1] - nut["pos"][1],
            eef[2] + clearance_height - nut["pos"][2],
        ]
        move(
            target,
            state["eef_yaw"] - square_error(nut["yaw"]),
            "close", 22, 0.002,
        )
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        if (
            hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
            < 0.003
            and abs(square_error(nut["yaw"])) < 0.04
        ):
            break

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    if hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) > 0.012:
        return fail("Could not center the nut above the peg")

    # Descend in short closed-loop segments. A stalled descent triggers a
    # small upward reset and a bounded lateral search around the peg center.
    previous_height = nut["pos"][2]
    stalls = 0
    search_index = 0
    seated = False

    for iteration in range(24):
        if remaining() < 26:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        eef = state["eef_pos"]
        position = nut["pos"]
        lateral_error = hypot(position[0] - goal[0], position[1] - goal[1])

        if (
            position[2] <= resting_height + 0.009
            and lateral_error < 0.010
            and abs(square_error(nut["yaw"])) < 0.10
        ):
            seated = True
            break

        if state["gripper_width"] < 0.010:
            return fail("Grasp lost during insertion")

        offset_x = 0.0
        offset_y = 0.0
        next_height = max(resting_height + 0.003, position[2] - 0.014)

        if stalls >= 2:
            next_height = position[2] + 0.008
            angle = search_index * pi / 2.0
            offset_x = 0.0025 * cos(angle)
            offset_y = 0.0025 * sin(angle)
            search_index += 1
            stalls = 0

        target = [
            eef[0] + goal[0] + offset_x - position[0],
            eef[1] + goal[1] + offset_y - position[1],
            eef[2] + next_height - position[2],
        ]
        move(
            target,
            state["eef_yaw"] - square_error(nut["yaw"]),
            "close", 8, 0.002,
        )

        current_height = sdk.get_state()["objects"]["nut"]["pos"][2]
        if next_height < position[2] and previous_height - current_height < 0.002:
            stalls += 1
        else:
            stalls = 0
        previous_height = current_height

    if not seated:
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        seated = (
            nut["pos"][2] <= resting_height + 0.012
            and hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.010
            and abs(square_error(nut["yaw"])) < 0.10
        )

    if not seated:
        return fail("Nut did not reach the seated height")

    actuate("open", 10)
    state = sdk.get_state()
    target = list(state["eef_pos"])
    target[2] += 0.085
    move(target, state["eef_yaw"], "open", 16)

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
