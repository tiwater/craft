def policy_move(sdk, pos, yaw, gripper, limit, reserve, tol=0.012):
    state = sdk.get_state()
    available = state["max_steps"] - state["steps_used"] - reserve
    if available <= 0:
        return False
    result = sdk.move_to(
        pos,
        yaw=yaw,
        gripper=gripper,
        max_steps=min(limit, available),
        tol=tol,
    )
    return result["reached"]


def policy_gripper(sdk, action, steps, reserve):
    state = sdk.get_state()
    available = state["max_steps"] - state["steps_used"] - reserve
    if available < steps:
        return False
    sdk.set_gripper(action, steps=steps)
    return True


def run_policy(sdk):
    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or target"}

    yaw = state["eef_yaw"]
    grasped = False

    for attempt in range(2):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 205:
            break

        # Every retry starts open and uses the pose after reopening.
        if not policy_gripper(sdk, "open", 10, 190):
            break
        state = sdk.get_state()
        if state["gripper_width"] < 0.070:
            if not policy_gripper(sdk, "open", 10, 180):
                break
            state = sdk.get_state()
        if state["gripper_width"] < 0.070:
            continue

        can = list(state["objects"]["can"]["pos"])
        goal = list(state["goal_pos"])
        eef = state["eef_pos"]
        clearance = max(1.08, can[2] + 0.19, goal[2] + 0.25)

        if attempt == 0:
            policy_move(
                sdk, [eef[0], eef[1], clearance],
                yaw, "open", 25, 200,
            )
            approach_height = clearance
        else:
            # The failed vertical probe leaves the hand near the can.
            approach_height = max(eef[2], can[2] + 0.085)
            policy_move(
                sdk, [eef[0], eef[1], approach_height],
                yaw, "open", 12, 180,
            )

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        above = [can[0], can[1], approach_height]
        policy_move(
            sdk, above, yaw, "open",
            40 if attempt == 0 else 18, 175, 0.010,
        )

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.018:
            policy_move(
                sdk, [can[0], can[1], approach_height],
                yaw, "open", 12, 170, 0.008,
            )

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - can[0], eef[1] - can[1]) > 0.018
            or state["gripper_width"] < 0.070
        ):
            continue

        policy_move(sdk, can, yaw, "open", 35, 160, 0.008)

        # The observed 8–12 mm residual is not an empty grasp.
        # Avoid repeatedly biasing below the can and pushing it sideways.
        for refinement in range(2):
            state = sdk.get_state()
            can = list(state["objects"]["can"]["pos"])
            eef = state["eef_pos"]
            if (
                hypot(eef[0] - can[0], eef[1] - can[1]) <= 0.009
                and abs(eef[2] - can[2]) <= 0.014
            ):
                break
            policy_move(sdk, can, yaw, "open", 8, 160, 0.006)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - can[0], eef[1] - can[1]) > 0.012
            or abs(eef[2] - can[2]) > 0.016
            or state["gripper_width"] < 0.070
        ):
            continue

        before = can[2]
        if not policy_gripper(sdk, "close", 20, 140):
            break

        state = sdk.get_state()
        eef = list(state["eef_pos"])

        # Test pickup before committing to the full clearance lift.
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.085],
            yaw, "close", 22, 120, 0.008,
        )

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        grasped = (
            can[2] - before >= 0.040
            and hypot(can[0] - eef[0], can[1] - eef[1]) < 0.055
            and 0.012 < state["gripper_width"] < 0.075
        )
        if grasped:
            break

        # Reopen immediately after a failed lift, then re-read on retry.
        if not policy_gripper(sdk, "open", 10, 0):
            break

    if not grasped:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    clearance = max(1.08, state["goal_pos"][2] + 0.25)
    policy_move(
        sdk,
        [eef[0], eef[1], eef[2] + clearance - min(can[2], eef[2])],
        yaw, "close", 30, 105, 0.008,
    )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    if min(can[2], eef[2]) < 1.06:
        policy_move(
            sdk,
            [eef[0], eef[1], eef[2] + 1.08 - min(can[2], eef[2])],
            yaw, "close", 12, 100, 0.008,
        )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    if min(can[2], eef[2]) < 1.06:
        return {"status": "failed", "reason": "Insufficient verified clearance"}

    offset = [can[i] - eef[i] for i in range(3)]
    goal = list(state["goal_pos"])
    transport_height = max(1.08, can[2], goal[2] + 0.24)

    for correction in range(3):
        state = sdk.get_state()
        goal = list(state["goal_pos"])
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        target = [
            eef[0] + goal[0] - can[0],
            eef[1] + goal[1] - can[1],
            eef[2] + transport_height - can[2],
        ]
        policy_move(
            sdk, target, yaw, "close",
            55 if correction == 0 else 12, 75, 0.008,
        )
        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        goal = state["goal_pos"]
        if hypot(can[0] - goal[0], can[1] - goal[1]) <= 0.012:
            break

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    goal = list(state["goal_pos"])
    if (
        hypot(can[0] - eef[0], can[1] - eef[1]) > 0.065
        or not 0.012 < state["gripper_width"] < 0.075
        or abs((can[2] - eef[2]) - offset[2]) > 0.045
        or min(can[2], eef[2]) < 1.06
    ):
        return {"status": "failed", "reason": "Can slipped during transport"}

    # An 8.7–11 mm residual over this bin is safe to correct during descent.
    if hypot(can[0] - goal[0], can[1] - goal[1]) > 0.025:
        return {"status": "failed", "reason": "Could not align above live target"}

    for correction in range(3):
        state = sdk.get_state()
        goal = list(state["goal_pos"])
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        release_height = goal[2] + 0.015
        if (
            hypot(can[0] - goal[0], can[1] - goal[1]) <= 0.012
            and abs(can[2] - release_height) <= 0.012
        ):
            break
        target = [
            eef[0] + goal[0] - can[0],
            eef[1] + goal[1] - can[1],
            eef[2] + release_height - can[2],
        ]
        policy_move(
            sdk, target, yaw, "close",
            35 if correction == 0 else 12, 30, 0.008,
        )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.015
        or abs(can[2] - (goal[2] + 0.015)) > 0.020
        or not 0.012 < state["gripper_width"] < 0.075
    ):
        return {"status": "failed", "reason": "Can is not safely positioned for release"}

    if state["max_steps"] - state["steps_used"] < 30:
        return {"status": "failed", "reason": "Insufficient time to verify release"}
    sdk.set_gripper("open", steps=12)

    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.16],
        yaw, "open", 20, 8,
    )

    state = sdk.get_state()
    previous = list(state["objects"]["can"]["pos"])
    remaining = state["max_steps"] - state["steps_used"]
    if remaining > 0:
        sdk.set_gripper("open", steps=min(10, remaining))

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    settled = sqrt(sum((can[i] - previous[i]) ** 2 for i in range(3))) < 0.012
    placed = (
        hypot(can[0] - goal[0], can[1] - goal[1]) < 0.065
        and goal[2] - 0.025 < can[2] < goal[2] + 0.12
        and state["eef_pos"][2] > can[2] + 0.07
        and state["gripper_width"] > 0.070
        and settled
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Placement verified" if placed else "Placement could not be verified",
        "steps_used": state["steps_used"],
    }
