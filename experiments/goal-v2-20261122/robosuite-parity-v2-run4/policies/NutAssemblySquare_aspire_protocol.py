def run_policy(sdk):
    def wrap(angle):
        return atan2(sin(angle), cos(angle))

    def square_error(angle):
        return atan2(sin(4.0 * angle), cos(4.0 * angle)) / 4.0

    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, action, ticks, tolerance=0.004):
        ticks = min(ticks, remaining())
        if ticks > 0:
            return sdk.move_to(
                pos, yaw=wrap(yaw), gripper=action,
                max_steps=ticks, tol=tolerance
            )
        return {"reached": False}

    def grip(action, ticks):
        ticks = min(ticks, remaining())
        if ticks > 0:
            sdk.set_gripper(action, steps=ticks)

    def tilt(nut):
        quat = nut["quat"]
        normal_z = 1.0 - 2.0 * (quat[0] ** 2 + quat[1] ** 2)
        return sqrt(max(0.0, 1.0 - min(1.0, normal_z ** 2)))

    def grasp_yaw(nut_yaw, wrist_yaw):
        difference = wrap(nut_yaw + pi / 2.0 - wrist_yaw)
        if difference > pi / 2.0:
            difference -= pi
        if difference < -pi / 2.0:
            difference += pi
        return wrap(wrist_yaw + difference)

    def center_move(target, ticks, rotate=False):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        eef = state["eef_pos"]
        correction = -square_error(nut["yaw"]) if rotate else 0.0
        dx = eef[0] - nut["pos"][0]
        dy = eef[1] - nut["pos"][1]
        return move(
            [
                target[0] + cos(correction) * dx - sin(correction) * dy,
                target[1] + sin(correction) * dx + cos(correction) * dy,
                target[2] + eef[2] - nut["pos"][2]
            ],
            state["eef_yaw"] + correction,
            "close", ticks
        )

    def finish(reason, success=False):
        state = sdk.get_state()
        return {
            "status": "done" if success else "failed",
            "reason": reason,
            "steps_used": state["steps_used"]
        }

    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return finish("Missing nut or goal pose")

    resting_z = state["objects"]["nut"]["pos"][2]
    held = False
    grasp_distance = 0.0
    grasp_height = 0.0

    for attempt in range(2):
        if remaining() < 205:
            break

        grip("open", 5)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        clearance = max(
            state["eef_pos"][2],
            nut["pos"][2] + 0.085
        )
        yaw = grasp_yaw(nut["yaw"], state["eef_yaw"])

        if state["eef_pos"][2] < nut["pos"][2] + 0.055:
            move(
                [state["eef_pos"][0], state["eef_pos"][1], clearance],
                state["eef_yaw"], "open", 15
            )

        offset = 0.050 if attempt == 0 else 0.045
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        move(
            [
                nut["pos"][0] + offset * cos(nut["yaw"]),
                nut["pos"][1] + offset * sin(nut["yaw"]),
                clearance
            ],
            yaw, "open", 30
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = grasp_yaw(nut["yaw"], state["eef_yaw"])
        move(
            [
                nut["pos"][0] + offset * cos(nut["yaw"]),
                nut["pos"][1] + offset * sin(nut["yaw"]),
                nut["pos"][2]
            ],
            yaw, "open", 30, 0.003
        )
        grip("close", 13)

        before = sdk.get_state()
        nut_z = before["objects"]["nut"]["pos"][2]
        wrist_z = before["eef_pos"][2]
        move(
            [
                before["eef_pos"][0],
                before["eef_pos"][1],
                wrist_z + 0.075
            ],
            before["eef_yaw"], "close", 27
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        rise = nut["pos"][2] - nut_z
        wrist_rise = state["eef_pos"][2] - wrist_z
        held = (
            rise > 0.035
            and abs(rise - wrist_rise) < 0.020
            and 0.012 < state["gripper_width"] < 0.070
            and tilt(nut) < 0.28
        )
        if held:
            grasp_distance = hypot(
                state["eef_pos"][0] - nut["pos"][0],
                state["eef_pos"][1] - nut["pos"][1]
            )
            grasp_height = state["eef_pos"][2] - nut["pos"][2]
            break

        if rise > 0.015:
            center_move(
                [nut["pos"][0], nut["pos"][1], resting_z + 0.003],
                20
            )
        grip("open", 6)

    if not held:
        return finish("Could not establish a stable grasp")

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    hover_z = max(resting_z + 0.115, goal[2] + 0.105)

    # The supplied endpoints remain near hover height. Bound alignment
    # work and reserve a continuous descent instead of repeatedly gating it.
    center_move(
        [nut["pos"][0], nut["pos"][1], hover_z],
        min(30, max(0, remaining() - 140)), True
    )
    state = sdk.get_state()
    goal = state["goal_pos"]
    center_move(
        [goal[0], goal[1], hover_z],
        min(45, max(0, remaining() - 105)), True
    )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    lateral = hypot(
        nut["pos"][0] - goal[0],
        nut["pos"][1] - goal[1]
    )
    if lateral > 0.004 or abs(square_error(nut["yaw"])) > 0.045:
        center_move(
            [goal[0], goal[1], hover_z],
            min(25, max(0, remaining() - 85)), True
        )

    captured = False
    stalls = 0
    recovery_used = False

    for attempt in range(7):
        if remaining() <= 32:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        pos = list(nut["pos"])
        goal = state["goal_pos"]
        lateral = hypot(pos[0] - goal[0], pos[1] - goal[1])
        yaw_error = abs(square_error(nut["yaw"]))

        captured = (
            lateral < 0.0055
            and pos[2] < goal[2] + 0.005
            and yaw_error < 0.08
            and tilt(nut) < 0.20
        )
        if captured:
            break

        distance = hypot(
            state["eef_pos"][0] - pos[0],
            state["eef_pos"][1] - pos[1]
        )
        height = state["eef_pos"][2] - pos[2]
        if (
            state["gripper_width"] < 0.010
            or abs(distance - grasp_distance) > 0.025
            or abs(height - grasp_height) > 0.030
            or tilt(nut) > 0.35
        ):
            return finish("Grasp became unstable before peg capture")

        above_peg = pos[2] > goal[2] + 0.080
        if lateral > 0.009 or yaw_error > 0.12:
            if not above_peg:
                center_move(
                    [pos[0], pos[1], hover_z],
                    min(22, max(0, remaining() - 65))
                )
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            goal = state["goal_pos"]
            center_move(
                [goal[0], goal[1], max(hover_z, nut["pos"][2])],
                min(25, max(0, remaining() - 55)), True
            )
            continue

        # A full-height target gives the controller time to descend.
        # Recompute the wrist offset only between substantial moves.
        ticks = min(50 if attempt == 0 else 28, remaining() - 32)
        center_move(
            [goal[0], goal[1], resting_z + 0.002],
            ticks
        )

        updated = sdk.get_state()
        new_nut = updated["objects"]["nut"]
        progress = pos[2] - new_nut["pos"][2]
        stalls = stalls + 1 if progress < 0.004 else 0

        if stalls > 0 and not recovery_used and remaining() > 90:
            # Unload contact before correcting orientation or centering.
            center_move(
                [new_nut["pos"][0], new_nut["pos"][1], hover_z],
                23
            )
            updated = sdk.get_state()
            goal = updated["goal_pos"]
            center_move([goal[0], goal[1], hover_z], 25, True)
            recovery_used = True
            stalls = 0

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    lateral = hypot(
        nut["pos"][0] - goal[0],
        nut["pos"][1] - goal[1]
    )
    captured = (
        lateral < 0.0055
        and nut["pos"][2] < goal[2] + 0.005
        and abs(square_error(nut["yaw"])) < 0.08
        and tilt(nut) < 0.20
    )
    if not captured:
        return finish("Nut did not reach verified peg capture")

    grip("open", 10)
    state = sdk.get_state()
    move(
        [
            state["eef_pos"][0],
            state["eef_pos"][1],
            state["eef_pos"][2] + 0.080
        ],
        state["eef_yaw"], "open", min(17, remaining()), 0.006
    )
    grip("open", min(5, remaining()))

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    success = (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.008
        and abs(nut["pos"][2] - resting_z) < 0.010
        and abs(square_error(nut["yaw"])) < 0.10
        and tilt(nut) < 0.20
    )
    return finish(
        "Released and verified seating" if success
        else "Nut did not settle after release",
        success
    )
