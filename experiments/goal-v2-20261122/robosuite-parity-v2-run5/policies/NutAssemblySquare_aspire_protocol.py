def wrap_angle(angle):
    return (angle + pi) % (2.0 * pi) - pi


def square_error(angle):
    return (angle + pi / 4.0) % (pi / 2.0) - pi / 4.0


def remaining(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def move(sdk, position, yaw, action, ticks, tolerance=0.003):
    ticks = min(ticks, remaining(sdk))
    if ticks <= 0:
        return False
    result = sdk.move_to(
        position,
        yaw=wrap_angle(yaw),
        gripper=action,
        max_steps=ticks,
        tol=tolerance,
    )
    return result["reached"]


def grip(sdk, action, ticks):
    ticks = min(ticks, remaining(sdk))
    if ticks > 0:
        sdk.set_gripper(action, steps=ticks)


def flatness(nut):
    quat = nut["quat"]
    return 1.0 - 2.0 * (quat[0] * quat[0] + quat[1] * quat[1])


def center_error(state):
    position = state["objects"]["nut"]["pos"]
    goal = state["goal_pos"]
    return hypot(position[0] - goal[0], position[1] - goal[1])


def move_nut(sdk, position, ticks, align=True):
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    eef = state["eef_pos"]
    correction = -square_error(nut["yaw"]) if align else 0.0
    correction = max(-0.35, min(0.35, correction))
    offset_x = eef[0] - nut["pos"][0]
    offset_y = eef[1] - nut["pos"][1]
    target = [
        position[0] + cos(correction) * offset_x - sin(correction) * offset_y,
        position[1] + sin(correction) * offset_x + cos(correction) * offset_y,
        position[2] + eef[2] - nut["pos"][2],
    ]
    return move(
        sdk, target, state["eef_yaw"] + correction, "close", ticks, 0.002
    )


def seated(state, resting_height):
    nut = state["objects"]["nut"]
    return (
        center_error(state) < 0.007
        and abs(nut["pos"][2] - resting_height) < 0.008
        and abs(square_error(nut["yaw"])) < 0.10
        and flatness(nut) > 0.97
    )


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or goal"}

    resting_height = state["objects"]["nut"]["pos"][2]
    attached = False

    # The failed approaches stopped roughly 0.027 m above the nut center.
    # Target above that contact floor instead of demanding a table-blocked pose.
    for handle_offset, height_offset in [(0.055, 0.033), (0.050, 0.029)]:
        if remaining(sdk) < 205:
            break

        grip(sdk, "open", 5)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        eef = state["eef_pos"]
        safe_height = max(eef[2], nut["pos"][2] + 0.095)
        if eef[2] < safe_height - 0.005:
            move(
                sdk, [eef[0], eef[1], safe_height],
                state["eef_yaw"], "open", 16, 0.005,
            )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        grasp_yaw = state["eef_yaw"] + (
            (yaw - state["eef_yaw"] + pi / 2.0) % pi - pi / 2.0
        )
        move(
            sdk,
            [
                nut["pos"][0] + handle_offset * cos(yaw),
                nut["pos"][1] + handle_offset * sin(yaw),
                safe_height,
            ],
            grasp_yaw, "open", 30, 0.004,
        )

        for height in [0.052, height_offset]:
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            yaw = nut["yaw"]
            grasp_yaw = state["eef_yaw"] + (
                (yaw - state["eef_yaw"] + pi / 2.0) % pi - pi / 2.0
            )
            move(
                sdk,
                [
                    nut["pos"][0] + handle_offset * cos(yaw),
                    nut["pos"][1] + handle_offset * sin(yaw),
                    nut["pos"][2] + height,
                ],
                grasp_yaw, "open", 16, 0.003,
            )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        eef = state["eef_pos"]
        horizontal_error = hypot(
            eef[0] - nut["pos"][0] - handle_offset * cos(nut["yaw"]),
            eef[1] - nut["pos"][1] - handle_offset * sin(nut["yaw"]),
        )
        vertical_offset = eef[2] - nut["pos"][2]
        if horizontal_error > 0.012 or not 0.023 < vertical_offset < 0.047:
            continue

        grip(sdk, "close", 16)
        state = sdk.get_state()
        before_nut = list(state["objects"]["nut"]["pos"])
        before_eef = list(state["eef_pos"])
        move(
            sdk,
            [before_eef[0], before_eef[1], before_eef[2] + 0.070],
            state["eef_yaw"], "close", 23, 0.004,
        )
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        nut_rise = nut["pos"][2] - before_nut[2]
        eef_rise = state["eef_pos"][2] - before_eef[2]
        attached = (
            nut_rise > 0.040
            and abs(nut_rise - eef_rise) < 0.018
            and 0.010 < state["gripper_width"] < 0.075
            and flatness(nut) > 0.96
        )
        if attached:
            break

        if nut_rise > 0.012:
            move_nut(
                sdk,
                [nut["pos"][0], nut["pos"][1], resting_height + 0.004],
                18, False,
            )
        grip(sdk, "open", 6)

    if not attached:
        return {
            "status": "failed",
            "reason": "Handle grasp did not lift and track the nut",
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    goal = state["goal_pos"]
    clearance = max(resting_height + 0.145, goal[2] + 0.115)
    nut = state["objects"]["nut"]
    move_nut(sdk, [nut["pos"][0], nut["pos"][1], clearance], 22, False)

    state = sdk.get_state()
    goal = state["goal_pos"]
    move_nut(sdk, [goal[0], goal[1], clearance], 35)

    aligned = False
    for iteration in range(7):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        if nut["pos"][2] < resting_height + 0.035 or flatness(nut) < 0.95:
            return {"status": "failed", "reason": "Nut slipped during transport"}
        if (
            center_error(state) < 0.0025
            and abs(square_error(nut["yaw"])) < 0.035
            and abs(nut["pos"][2] - clearance) < 0.008
            and flatness(nut) > 0.98
        ):
            aligned = True
            break
        if remaining(sdk) < 105:
            break
        goal = state["goal_pos"]
        move_nut(sdk, [goal[0], goal[1], clearance], 11)

    if not aligned:
        return {
            "status": "failed",
            "reason": "Could not establish alignment above peg",
            "center_error_m": center_error(sdk.get_state()),
            "steps_used": sdk.get_state()["steps_used"],
        }

    stalls = 0
    recoveries = 0
    release_ready = False
    for iteration in range(24):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        height = nut["pos"][2]

        if seated(state, resting_height):
            release_ready = True
            break
        if remaining(sdk) < 35:
            break
        if flatness(nut) < 0.95 or center_error(state) > 0.018:
            return {"status": "failed", "reason": "Nut became unstable during insertion"}

        precise = (
            center_error(state) < 0.0025
            and abs(square_error(nut["yaw"])) < 0.035
            and flatness(nut) > 0.98
        )
        descent = 0.020 if height > goal[2] + 0.075 else 0.008
        target_height = max(resting_height + 0.002, height - descent)
        if not precise:
            target_height = height
        move_nut(sdk, [goal[0], goal[1], target_height], 8)

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        if precise and height - nut["pos"][2] < 0.0015:
            stalls += 1
        else:
            stalls = 0

        if stalls >= 2:
            # Unload peg contact before correcting lateral or yaw error.
            if recoveries < 2 and remaining(sdk) > 65:
                goal = state["goal_pos"]
                raised_height = nut["pos"][2] + 0.012
                move_nut(sdk, [goal[0], goal[1], raised_height], 9, False)
                move_nut(sdk, [goal[0], goal[1], raised_height], 9)
                recoveries += 1
                stalls = 0
            else:
                break

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    release_ready = release_ready or seated(state, resting_height)

    # Permit gravity settling only after the nut has descended below the tip
    # region; precise alignment alone above the peg is not proof of engagement.
    if not release_ready:
        release_ready = (
            center_error(state) < 0.002
            and abs(square_error(nut["yaw"])) < 0.030
            and flatness(nut) > 0.99
            and resting_height - 0.004 < nut["pos"][2] < resting_height + 0.018
        )

    if not release_ready:
        return {
            "status": "failed",
            "reason": "Insertion not sufficiently engaged for release",
            "center_error_m": center_error(state),
            "steps_used": state["steps_used"],
        }

    grip(sdk, "open", 10)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        sdk, [eef[0], eef[1], eef[2] + 0.090],
        state["eef_yaw"], "open", 16, 0.005,
    )
    grip(sdk, "open", 8)

    state = sdk.get_state()
    success = seated(state, resting_height)
    return {
        "status": "done" if success else "failed",
        "reason": "Released nut is seated" if success else "Released nut did not settle",
        "center_error_m": center_error(state),
        "steps_used": state["steps_used"],
    }
