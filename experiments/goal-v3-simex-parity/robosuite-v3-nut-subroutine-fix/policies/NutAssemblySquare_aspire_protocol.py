def wrap_angle(angle):
    return atan2(sin(angle), cos(angle))


def square_error(angle):
    return atan2(sin(4.0 * angle), cos(4.0 * angle)) / 4.0


def budget(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def move(sdk, target, yaw, action, steps, tolerance=0.003):
    count = min(steps, budget(sdk))
    if count > 0:
        return sdk.move_to(
            target, yaw=yaw, gripper=action,
            max_steps=count, tol=tolerance
        )
    return {"reached": False}


def grip(sdk, action, steps):
    count = min(steps, budget(sdk))
    if count > 0:
        sdk.set_gripper(action, steps=count)


def upright(nut):
    quat = nut["quat"]
    return 1.0 - 2.0 * (quat[0] * quat[0] + quat[1] * quat[1])


def move_nut(sdk, target, steps, tolerance=0.003):
    state = sdk.get_state()
    position = state["objects"]["nut"]["pos"]
    eef = state["eef_pos"]
    return move(
        sdk,
        [eef[i] + target[i] - position[i] for i in range(3)],
        state["eef_yaw"], "close", steps, tolerance
    )


def align_nut(sdk, steps):
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    correction = -square_error(nut["yaw"])
    if abs(correction) < 0.012:
        return
    eef = state["eef_pos"]
    dx = eef[0] - nut["pos"][0]
    dy = eef[1] - nut["pos"][1]
    move(
        sdk,
        [
            nut["pos"][0] + cos(correction) * dx - sin(correction) * dy,
            nut["pos"][1] + sin(correction) * dx + cos(correction) * dy,
            eef[2],
        ],
        state["eef_yaw"] + correction, "close", steps, 0.003
    )


def finish(sdk, success, reason):
    state = sdk.get_state()
    return {
        "status": "done" if success else "failed",
        "reason": reason,
        "steps_used": state["steps_used"],
    }


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return finish(sdk, False, "Missing nut or goal")

    table_nut_z = state["objects"]["nut"]["pos"][2]
    acquired = False

    # Deeper engagement near the ring can lever the nut over during lifting.
    # Grasp farther along the handle and verify both lift and orientation.
    for attempt in range(2):
        if budget(sdk) < 210:
            break

        grip(sdk, "open", 5)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        safe_z = max(table_nut_z + 0.105, nut["pos"][2] + 0.085)
        eef = state["eef_pos"]
        if eef[2] < safe_z - 0.008:
            move(
                sdk, [eef[0], eef[1], safe_z],
                state["eef_yaw"], "open", 18, 0.006
            )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        offset = 0.055 if attempt == 0 else 0.050
        correction = wrap_angle(nut["yaw"] - state["eef_yaw"])
        if correction > pi / 2.0:
            correction -= pi
        if correction < -pi / 2.0:
            correction += pi
        grasp_yaw = state["eef_yaw"] + correction

        move(
            sdk,
            [
                nut["pos"][0] + offset * cos(nut["yaw"]),
                nut["pos"][1] + offset * sin(nut["yaw"]),
                safe_z,
            ],
            grasp_yaw, "open", 28, 0.004
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        target = [
            nut["pos"][0] + offset * cos(nut["yaw"]),
            nut["pos"][1] + offset * sin(nut["yaw"]),
            nut["pos"][2] + 0.002,
        ]
        move(sdk, target, grasp_yaw, "open", 28, 0.003)

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        target = [
            nut["pos"][0] + offset * cos(nut["yaw"]),
            nut["pos"][1] + offset * sin(nut["yaw"]),
            nut["pos"][2] + 0.002,
        ]
        if hypot(
            state["eef_pos"][0] - target[0],
            state["eef_pos"][1] - target[1]
        ) > 0.005:
            move(sdk, target, grasp_yaw, "open", 10, 0.003)

        baseline = sdk.get_state()["objects"]["nut"]["pos"][2]
        grip(sdk, "close", 14)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move(
            sdk, [eef[0], eef[1], eef[2] + 0.065],
            state["eef_yaw"], "close", 25, 0.004
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        acquired = (
            nut["pos"][2] > baseline + 0.040
            and 0.010 < state["gripper_width"] < 0.070
            and upright(nut) > 0.94
        )
        if acquired:
            break

        if nut["pos"][2] > baseline + 0.020:
            move_nut(
                sdk,
                [nut["pos"][0], nut["pos"][1], table_nut_z + 0.006],
                20
            )
        grip(sdk, "open", 6)

    if not acquired:
        return finish(sdk, False, "Stable upright grasp not acquired")

    state = sdk.get_state()
    goal = state["goal_pos"]
    clearance = max(table_nut_z + 0.135, goal[2] + 0.115)
    nut = state["objects"]["nut"]
    move_nut(sdk, [nut["pos"][0], nut["pos"][1], clearance], 22)
    align_nut(sdk, 20)

    for iteration in range(3):
        if budget(sdk) < 105:
            break
        state = sdk.get_state()
        goal = state["goal_pos"]
        move_nut(
            sdk, [goal[0], goal[1], clearance],
            min(30, budget(sdk) - 85), 0.0025
        )
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        if (
            hypot(
                nut["pos"][0] - goal[0],
                nut["pos"][1] - goal[1]
            ) < 0.004
            and abs(nut["pos"][2] - clearance) < 0.007
        ):
            break

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    if (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) > 0.010
        or upright(nut) < 0.94
    ):
        return finish(sdk, False, "Transport alignment or grasp unstable")

    align_nut(sdk, 12)

    # Use persistent depth targets with enough settling time. Recomputing
    # tiny descents from the current height can consume the entire horizon.
    depth = clearance
    stalled = 0
    recovered = False
    seated = False

    for iteration in range(12):
        if budget(sdk) <= 30:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        lateral = hypot(
            nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]
        )

        if upright(nut) < 0.90 or lateral > 0.025:
            return finish(sdk, False, "Grasp lost alignment during insertion")

        if lateral < 0.007 and nut["pos"][2] <= table_nut_z + 0.010:
            seated = True
            break

        if lateral > 0.008:
            move_nut(
                sdk, [goal[0], goal[1], nut["pos"][2]],
                min(15, budget(sdk) - 30), 0.002
            )
            continue

        if abs(square_error(nut["yaw"])) > 0.045:
            align_nut(sdk, min(12, budget(sdk) - 30))
            state = sdk.get_state()
            nut = state["objects"]["nut"]

        previous_z = nut["pos"][2]
        if previous_z <= depth + 0.008:
            depth = max(table_nut_z + 0.002, depth - 0.028)

        move_nut(
            sdk, [goal[0], goal[1], depth],
            min(22, budget(sdk) - 30), 0.0025
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        if previous_z - nut["pos"][2] < 0.003 and depth < previous_z - 0.008:
            stalled += 1
        else:
            stalled = 0

        if stalled >= 2:
            if not recovered and budget(sdk) > 90:
                goal = state["goal_pos"]
                raised = min(clearance, nut["pos"][2] + 0.025)
                move_nut(sdk, [goal[0], goal[1], raised], 18)
                align_nut(sdk, 14)
                depth = raised
                stalled = 0
                recovered = True
            else:
                break

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    lateral = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])

    # Release only after substantial insertion, allowing gravity to finish
    # seating without the fingers contacting the table.
    release_ready = (
        lateral < 0.007
        and table_nut_z - 0.008 <= nut["pos"][2] <= table_nut_z + 0.025
        and upright(nut) > 0.96
        and abs(square_error(nut["yaw"])) < 0.10
    )
    if not release_ready:
        return finish(sdk, False, "Insertion depth not verified")

    grip(sdk, "open", 9)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        sdk, [eef[0], eef[1], eef[2] + 0.085],
        state["eef_yaw"], "open", min(17, budget(sdk)), 0.006
    )
    grip(sdk, "open", min(4, budget(sdk)))

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    success = (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.008
        and table_nut_z - 0.008 <= nut["pos"][2] <= table_nut_z + 0.010
        and upright(nut) > 0.97
        and abs(square_error(nut["yaw"])) < 0.12
        and state["gripper_width"] > 0.055
    )
    return finish(
        sdk, success,
        "Verified released seating" if success else "Released seating check failed"
    )
