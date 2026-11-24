def square_yaw_error(yaw):
    error = -yaw
    while error > pi / 4:
        error -= pi / 2
    while error < -pi / 4:
        error += pi / 2
    return error


def wrap_yaw(yaw):
    while yaw > pi:
        yaw -= 2 * pi
    while yaw < -pi:
        yaw += 2 * pi
    return yaw


def policy_move(sdk, pos, yaw, grip, steps, tol=0.004):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        pos,
        yaw=wrap_yaw(yaw),
        gripper=grip,
        max_steps=min(steps, remaining),
        tol=tol,
    )
    return result["reached"]


def policy_grip(sdk, action, steps):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining > 0:
        sdk.set_gripper(action, steps=min(steps, remaining))


def place_nut_center(sdk, center, steps, align=True):
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    eef = state["eef_pos"]
    offset = [eef[i] - nut["pos"][i] for i in range(3)]
    correction = square_yaw_error(nut["yaw"]) if align else 0.0
    c = cos(correction)
    s = sin(correction)
    target = [
        center[0] + c * offset[0] - s * offset[1],
        center[1] + s * offset[0] + c * offset[1],
        center[2] + offset[2],
    ]
    return policy_move(
        sdk, target, state["eef_yaw"] + correction, "close", steps
    )


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    resting_z = state["objects"]["nut"]["pos"][2]
    acquired = False

    # The square nut's grasping handle extends along its local positive X axis.
    for attempt in range(2):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 160:
            break

        policy_grip(sdk, "open", 6)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        pos = nut["pos"]
        yaw = nut["yaw"]
        handle_offset = 0.055 if attempt == 0 else 0.050
        grasp_yaw = yaw + attempt * pi / 2
        handle = [
            pos[0] + handle_offset * cos(yaw),
            pos[1] + handle_offset * sin(yaw),
            pos[2],
        ]
        approach_z = max(
            pos[2] + 0.11, state["goal_pos"][2] + 0.09
        )

        # Raise vertically before traversing the table.
        eef = state["eef_pos"]
        policy_move(
            sdk,
            [eef[0], eef[1], max(eef[2], approach_z)],
            state["eef_yaw"],
            "open",
            15,
        )
        policy_move(
            sdk,
            [handle[0], handle[1], approach_z],
            grasp_yaw,
            "open",
            28,
        )

        # Refresh the grasp target after the approach.
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        handle = [
            nut["pos"][0] + handle_offset * cos(yaw),
            nut["pos"][1] + handle_offset * sin(yaw),
            nut["pos"][2],
        ]
        grasp_yaw = yaw + attempt * pi / 2
        before_z = nut["pos"][2]
        policy_move(sdk, handle, grasp_yaw, "open", 24, 0.003)
        policy_grip(sdk, "close", 12)

        state = sdk.get_state()
        eef = state["eef_pos"]
        policy_move(
            sdk,
            [eef[0], eef[1], eef[2] + 0.11],
            state["eef_yaw"],
            "close",
            25,
        )
        state = sdk.get_state()
        acquired = (
            state["objects"]["nut"]["pos"][2] > before_z + 0.055
            and state["gripper_width"] > 0.012
        )
        if acquired:
            break

    if not acquired:
        policy_grip(sdk, "open", 5)
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    goal = state["goal_pos"]
    clearance_z = max(resting_z + 0.14, goal[2] + 0.10)

    # Rotate above the source before carrying the hole over the peg.
    nut_pos = state["objects"]["nut"]["pos"]
    place_nut_center(
        sdk, [nut_pos[0], nut_pos[1], clearance_z], 24
    )
    state = sdk.get_state()
    goal = state["goal_pos"]
    place_nut_center(sdk, [goal[0], goal[1], clearance_z], 35)

    # Use measured nut orientation and center, including any grasp slip.
    for refinement in range(3):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        xy_error = hypot(
            nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]
        )
        if xy_error < 0.003 and abs(square_yaw_error(nut["yaw"])) < 0.035:
            break
        place_nut_center(sdk, [goal[0], goal[1], clearance_z], 15)

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    if (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) > 0.008
        or abs(square_yaw_error(nut["yaw"])) > 0.10
    ):
        return {"status": "failed", "reason": "Could not align hole above peg"}

    # Descend in short stages; reserve time for release and verification.
    target_z = clearance_z
    stalled = 0
    for stage in range(12):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 32:
            break
        goal = state["goal_pos"]
        before_z = state["objects"]["nut"]["pos"][2]
        target_z = max(resting_z + 0.002, target_z - 0.025)
        place_nut_center(sdk, [goal[0], goal[1], target_z], 12)
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        xy_error = hypot(
            nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]
        )
        if nut["pos"][2] <= resting_z + 0.012 and xy_error < 0.008:
            break

        if before_z - nut["pos"][2] < 0.003 and target_z < before_z - 0.012:
            stalled += 1
            if stalled > 1:
                break
            # Unload contact before correcting orientation and centering.
            recovery_z = max(clearance_z, nut["pos"][2] + 0.04)
            place_nut_center(
                sdk, [nut["pos"][0], nut["pos"][1], recovery_z], 16, False
            )
            place_nut_center(sdk, [goal[0], goal[1], recovery_z], 16)
            target_z = recovery_z

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    if (
        nut["pos"][2] > resting_z + 0.018
        or hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) > 0.009
    ):
        return {"status": "failed", "reason": "Nut did not seat on peg"}

    policy_grip(sdk, "open", 10)
    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk,
        [eef[0], eef[1], eef[2] + 0.08],
        state["eef_yaw"],
        "open",
        18,
    )

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    xy_error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    seated = (
        xy_error < 0.009
        and abs(nut["pos"][2] - resting_z) < 0.018
        and abs(square_yaw_error(nut["yaw"])) < 0.12
        and state["gripper_width"] > 0.05
    )
    return {
        "status": "done" if seated else "failed",
        "reason": "Released and geometrically seated" if seated else "Seat verification failed",
        "center_error_m": xy_error,
        "steps_used": state["steps_used"],
    }
