def wrap_angle(angle):
    return atan2(sin(angle), cos(angle))


def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def policy_move(sdk, position, yaw, gripper, steps, tolerance, reserve=0):
    budget = remaining_steps(sdk) - reserve
    if budget <= 0:
        return False
    result = sdk.move_to(
        position,
        yaw=yaw,
        gripper=gripper,
        max_steps=min(steps, budget),
        tol=tolerance,
    )
    return result["reached"]


def policy_gripper(sdk, action, steps, reserve=0):
    budget = remaining_steps(sdk) - reserve
    if budget > 0:
        sdk.set_gripper(action, steps=min(steps, budget))


def square_error(yaw):
    return -0.25 * atan2(sin(4.0 * yaw), cos(4.0 * yaw))


def run_policy(sdk):
    state = sdk.get_state()
    if state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing peg position"}

    resting_z = state["objects"]["nut"]["pos"][2]
    grasped = False

    for attempt in range(2):
        if remaining_steps(sdk) < 175:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        radius = 0.055 if attempt == 0 else 0.050
        handle_x = nut["pos"][0] + radius * cos(yaw)
        handle_y = nut["pos"][1] + radius * sin(yaw)
        approach_z = nut["pos"][2] + 0.095
        grasp_yaw = wrap_angle(
            state["eef_yaw"]
            + 0.5 * atan2(
                sin(2.0 * (yaw - state["eef_yaw"])),
                cos(2.0 * (yaw - state["eef_yaw"])),
            )
        )

        # Let translation and wrist rotation converge together. A timeout
        # alone does not mean the approach failed.
        policy_move(
            sdk,
            [handle_x, handle_y, approach_z],
            grasp_yaw,
            "open",
            65,
            0.008,
            135,
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        handle_x = nut["pos"][0] + radius * cos(yaw)
        handle_y = nut["pos"][1] + radius * sin(yaw)
        lateral_error = hypot(
            state["eef_pos"][0] - handle_x,
            state["eef_pos"][1] - handle_y,
        )
        if lateral_error > 0.012:
            policy_move(
                sdk,
                [handle_x, handle_y, approach_z],
                grasp_yaw,
                "open",
                30,
                0.006,
                130,
            )

        state = sdk.get_state()
        if hypot(
            state["eef_pos"][0] - handle_x,
            state["eef_pos"][1] - handle_y,
        ) > 0.018:
            return {"status": "failed", "reason": "Handle approach did not converge"}

        for descent in range(2):
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            handle = [
                nut["pos"][0] + radius * cos(nut["yaw"]),
                nut["pos"][1] + radius * sin(nut["yaw"]),
                nut["pos"][2] + 0.003,
            ]
            policy_move(
                sdk, handle, grasp_yaw, "open",
                35 if descent == 0 else 18, 0.004, 115
            )
            state = sdk.get_state()
            eef = state["eef_pos"]
            if (
                hypot(eef[0] - handle[0], eef[1] - handle[1]) < 0.008
                and abs(eef[2] - handle[2]) < 0.009
            ):
                break

        state = sdk.get_state()
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - handle[0], eef[1] - handle[1]) > 0.012
            or abs(eef[2] - handle[2]) > 0.015
        ):
            return {"status": "failed", "reason": "Handle descent did not converge"}

        before_lift = state["objects"]["nut"]["pos"][2]
        policy_gripper(sdk, "close", 12, 100)
        state = sdk.get_state()
        eef = state["eef_pos"]
        policy_move(
            sdk,
            [eef[0], eef[1], eef[2] + 0.115],
            grasp_yaw,
            "close",
            35,
            0.006,
            90,
        )

        state = sdk.get_state()
        lift = state["objects"]["nut"]["pos"][2] - before_lift
        if 0.012 < state["gripper_width"] < 0.070 and lift > 0.055:
            grasped = True
            break

        if lift > 0.012:
            eef = state["eef_pos"]
            policy_move(
                sdk,
                [eef[0], eef[1], eef[2] - lift],
                state["eef_yaw"],
                "close",
                22,
                0.006,
                25,
            )
        policy_gripper(sdk, "open", 8)

    if not grasped:
        return {"status": "failed", "reason": "Handle grasp did not survive lift"}

    # The standard square peg is world aligned. Rotate the measured
    # handle-to-hole offset along with the wrist during the transfer.
    for iteration in range(4):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        eef = state["eef_pos"]
        angle = square_error(nut["yaw"])
        dx = eef[0] - nut["pos"][0]
        dy = eef[1] - nut["pos"][1]
        clearance_z = max(goal[2], resting_z) + 0.105
        target = [
            goal[0] + cos(angle) * dx - sin(angle) * dy,
            goal[1] + sin(angle) * dx + cos(angle) * dy,
            eef[2] + max(0.0, clearance_z - nut["pos"][2]),
        ]
        policy_move(
            sdk,
            target,
            wrap_angle(state["eef_yaw"] + angle),
            "close",
            55 if iteration == 0 else 18,
            0.003,
            70,
        )
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        if (
            hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.0035
            and abs(square_error(nut["yaw"])) < 0.035
            and nut["pos"][2] > clearance_z - 0.012
        ):
            break

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    if (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) > 0.005
        or abs(square_error(nut["yaw"])) > 0.045
    ):
        return {"status": "failed", "reason": "Hole or square yaw alignment failed"}
    if state["gripper_width"] < 0.012:
        return {"status": "failed", "reason": "Grasp lost during transfer"}
    if remaining_steps(sdk) < 45:
        return {"status": "failed", "reason": "Insufficient insertion budget"}

    eef = state["eef_pos"]
    insertion_x = eef[0]
    insertion_y = eef[1]
    insertion_yaw = state["eef_yaw"]
    vertical_offset = eef[2] - nut["pos"][2]
    start_z = nut["pos"][2]
    seat_z = resting_z + 0.001

    for fraction in [0.50, 0.80, 1.0]:
        target_z = start_z + fraction * (seat_z - start_z)
        policy_move(
            sdk,
            [insertion_x, insertion_y, target_z + vertical_offset],
            insertion_yaw,
            "close",
            18,
            0.003,
            22,
        )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    if nut["pos"][2] > resting_z + 0.018 and remaining_steps(sdk) > 30:
        policy_move(
            sdk,
            [insertion_x, insertion_y, seat_z + vertical_offset],
            insertion_yaw,
            "close",
            15,
            0.003,
            22,
        )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    if (
        nut["pos"][2] > resting_z + 0.018
        or hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) > 0.009
    ):
        return {"status": "failed", "reason": "Nut did not seat on peg"}

    policy_gripper(sdk, "open", 12, 8)
    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk,
        [eef[0], eef[1], eef[2] + 0.10],
        state["eef_yaw"],
        "open",
        20,
        0.008,
    )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    seated = (
        error < 0.012
        and abs(nut["pos"][2] - resting_z) < 0.018
        and abs(square_error(nut["yaw"])) < 0.08
        and state["gripper_width"] > 0.05
    )
    return {
        "status": "done" if seated else "failed",
        "reason": "Released and geometrically seated" if seated else "Seating verification failed",
        "hole_position_error_m": error,
        "steps_used": state["steps_used"],
    }
