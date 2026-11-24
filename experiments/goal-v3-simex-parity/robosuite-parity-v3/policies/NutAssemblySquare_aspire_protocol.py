def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def square_error(angle):
        return (angle + pi / 4.0) % (pi / 2.0) - pi / 4.0

    def wrap(angle):
        return (angle + pi) % (2.0 * pi) - pi

    def move(position, yaw, grip, steps, tolerance=0.002):
        budget = min(steps, remaining())
        if budget > 0:
            return sdk.move_to(
                position,
                yaw=yaw,
                gripper=grip,
                max_steps=budget,
                tol=tolerance,
            )
        return {"reached": False}

    def actuate(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def finish(reason, success=False):
        state = sdk.get_state()
        return {
            "status": "done" if success else "failed",
            "reason": reason,
            "steps_used": state["steps_used"],
        }

    def handle_position(nut):
        return [
            nut["pos"][0] + 0.055 * cos(nut["yaw"]),
            nut["pos"][1] + 0.055 * sin(nut["yaw"]),
            nut["pos"][2] + 0.003,
        ]

    def grasp_yaw(state, offset):
        desired = state["objects"]["nut"]["yaw"] + offset
        error = (desired - state["eef_yaw"] + pi / 2.0) % pi - pi / 2.0
        return state["eef_yaw"] + error

    def lateral_error(state):
        nut = state["objects"]["nut"]["pos"]
        goal = state["goal_pos"]
        return hypot(nut[0] - goal[0], nut[1] - goal[1])

    def object_move(height, steps, turn=True):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        position = nut["pos"]
        eef = state["eef_pos"]
        goal = state["goal_pos"]
        angle = -square_error(nut["yaw"]) if turn else 0.0

        # Rotate the measured handle-to-hole offset along with the wrist.
        # Translating the old offset while turning moves the hole off target.
        offset_x = eef[0] - position[0]
        offset_y = eef[1] - position[1]
        target = [
            goal[0] + cos(angle) * offset_x - sin(angle) * offset_y,
            goal[1] + sin(angle) * offset_x + cos(angle) * offset_y,
            height + eef[2] - position[2],
        ]
        return move(
            target,
            state["eef_yaw"] + angle,
            "close",
            steps,
            0.0015,
        )

    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return finish("Missing nut or peg pose")

    resting_height = state["objects"]["nut"]["pos"][2]
    held = False

    for attempt in range(2):
        if remaining() < 195:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        offset = 0.0 if attempt == 0 else pi / 2.0
        yaw = grasp_yaw(state, offset)
        clearance = max(state["eef_pos"][2], nut["pos"][2] + 0.09)

        actuate("open", 7)
        move(
            [state["eef_pos"][0], state["eef_pos"][1], clearance],
            yaw, "open", 12, 0.005,
        )

        state = sdk.get_state()
        handle = handle_position(state["objects"]["nut"])
        move([handle[0], handle[1], clearance], yaw, "open", 26, 0.003)

        state = sdk.get_state()
        move(
            handle_position(state["objects"]["nut"]),
            grasp_yaw(state, offset),
            "open", 28, 0.002,
        )

        state = sdk.get_state()
        handle = handle_position(state["objects"]["nut"])
        if (
            hypot(state["eef_pos"][0] - handle[0],
                  state["eef_pos"][1] - handle[1]) > 0.005
            or abs(state["eef_pos"][2] - handle[2]) > 0.008
        ):
            move(handle, grasp_yaw(state, offset), "open", 12, 0.002)

        state = sdk.get_state()
        baseline = state["objects"]["nut"]["pos"][2]
        actuate("close", 12)

        state = sdk.get_state()
        target = list(state["eef_pos"])
        target[2] += 0.09
        move(target, state["eef_yaw"], "close", 24, 0.003)

        state = sdk.get_state()
        held = (
            state["objects"]["nut"]["pos"][2] > baseline + 0.045
            and state["gripper_width"] > 0.010
        )
        if held:
            break
        actuate("open", 6)

    if not held:
        return finish("Handle grasp did not lift the nut")

    state = sdk.get_state()
    clearance_height = max(
        resting_height + 0.115,
        state["goal_pos"][2] + 0.100,
        state["objects"]["nut"]["pos"][2],
    )

    target = list(state["eef_pos"])
    target[2] += clearance_height - state["objects"]["nut"]["pos"][2]
    move(target, state["eef_yaw"], "close", 16, 0.003)

    # Bound overhead alignment so insertion retains a useful control budget.
    for iteration in range(4):
        if remaining() <= 155:
            break
        state = sdk.get_state()
        if (
            lateral_error(state) < 0.003
            and abs(square_error(state["objects"]["nut"]["yaw"])) < 0.045
        ):
            break
        object_move(
            clearance_height,
            min(32 if iteration == 0 else 18, remaining() - 155),
        )

    state = sdk.get_state()
    entry_height = max(
        resting_height + 0.065,
        state["goal_pos"][2] + 0.045,
    )
    commanded_height = clearance_height
    contact_stalls = 0
    recoveries = 0
    seated = False

    for iteration in range(24):
        if remaining() <= 28:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        position = nut["pos"]
        lateral = lateral_error(state)
        yaw_error = abs(square_error(nut["yaw"]))

        if state["gripper_width"] < 0.009:
            return finish("Lost the handle before seating")

        if (
            abs(position[2] - resting_height) < 0.007
            and lateral < 0.005
            and yaw_error < 0.08
        ):
            seated = True
            break

        # Maintain an absolute descent target across updates. Small wrist
        # moves can transiently raise a tilted nut without indicating contact.
        if position[2] > entry_height + 0.012:
            if lateral < 0.015 and yaw_error < 0.12:
                commanded_height = max(
                    entry_height,
                    min(commanded_height, position[2] - 0.030),
                )
        elif lateral < 0.004 and yaw_error < 0.06:
            commanded_height = max(
                resting_height + 0.001,
                min(commanded_height, position[2] - 0.018),
            )

        # Retract only after sustained lack of progress near the peg.
        if contact_stalls >= 2 and recoveries < 2:
            recovery_height = max(entry_height, position[2] + 0.015)
            object_move(
                recovery_height,
                min(20, remaining() - 28),
            )
            commanded_height = recovery_height
            recoveries += 1
            contact_stalls = 0
            continue

        old_height = position[2]
        old_lateral = lateral
        old_yaw = nut["yaw"]
        old_eef = list(state["eef_pos"])
        descending = commanded_height < old_height - 0.006

        object_move(
            commanded_height,
            min(20 if descending else 16, remaining() - 28),
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        new_height = nut["pos"][2]
        wrist_motion = sqrt(sum(
            (state["eef_pos"][axis] - old_eef[axis]) ** 2
            for axis in range(3)
        ))

        # Do not mistake active rotation or overhead pose settling for a jam.
        contact_likely = (
            descending
            and old_height <= entry_height + 0.012
            and old_lateral < 0.006
            and abs(wrap(nut["yaw"] - old_yaw)) < 0.025
            and wrist_motion < 0.006
            and old_height - new_height < 0.0015
        )
        contact_stalls = contact_stalls + 1 if contact_likely else 0

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    seated = (
        abs(nut["pos"][2] - resting_height) < 0.008
        and lateral_error(state) < 0.005
        and abs(square_error(nut["yaw"])) < 0.08
    )
    if not seated:
        return finish("Hole alignment or seating did not converge")

    actuate("open", 10)
    state = sdk.get_state()
    target = list(state["eef_pos"])
    target[2] += 0.075
    move(target, state["eef_yaw"], "open", 18, 0.005)

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    success = (
        abs(nut["pos"][2] - resting_height) < 0.012
        and lateral_error(state) < 0.008
        and abs(square_error(nut["yaw"])) < 0.10
        and state["gripper_width"] > 0.040
        and state["eef_pos"][2] > nut["pos"][2] + 0.045
    )
    return finish(
        "Seated and released" if success else "Release verification failed",
        success,
    )
