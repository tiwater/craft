def policy_move(sdk, pos, yaw, gripper, steps, tol=0.01):
    state = sdk.get_state()
    remaining = min(400, state["max_steps"]) - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        pos,
        yaw=yaw,
        gripper=gripper,
        max_steps=min(steps, remaining),
        tol=tol,
    )
    return result["reached"]


def policy_grip(sdk, action, steps):
    state = sdk.get_state()
    remaining = min(400, state["max_steps"]) - state["steps_used"]
    if remaining < steps:
        return False
    sdk.set_gripper(action, steps=steps)
    return True


def policy_holding(state):
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    separation = sqrt(sum((eef[i] - can[i]) ** 2 for i in range(3)))
    return (
        0.015 <= state["gripper_width"] <= 0.065
        and separation < 0.10
    )


def policy_grasp_aligned(state):
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    return (
        hypot(eef[0] - can[0], eef[1] - can[1]) <= 0.006
        and -0.008 <= eef[2] - can[2] <= 0.014
        and state["gripper_width"] >= 0.075
    )


def run_policy(sdk):
    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or goal"}

    yaw = state["eef_yaw"]
    grasped = False

    for attempt in range(2):
        state = sdk.get_state()
        remaining = min(400, state["max_steps"]) - state["steps_used"]
        if remaining < 175:
            break

        if not policy_grip(sdk, "open", 10):
            break
        state = sdk.get_state()
        if state["gripper_width"] < 0.075:
            if not policy_grip(sdk, "open", 8):
                break
            state = sdk.get_state()
        if state["gripper_width"] < 0.075:
            continue

        # Refresh the object pose after opening, including on the retry.
        can = list(state["objects"]["can"]["pos"])
        goal = state["goal_pos"]
        clearance = max(1.08, max(can[2], goal[2]) + 0.20)
        eef = state["eef_pos"]

        if eef[2] < clearance - 0.025:
            if not policy_move(
                sdk, [eef[0], eef[1], clearance],
                yaw, "open", 45
            ):
                continue

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        if not policy_move(
            sdk, [can[0], can[1], clearance],
            yaw, "open", 55
        ):
            continue

        state = sdk.get_state()
        if state["gripper_width"] < 0.075:
            if not policy_grip(sdk, "open", 8):
                break
            state = sdk.get_state()
        if state["gripper_width"] < 0.075:
            continue

        # Target the live center. Stop spending descent steps once the
        # observed 11--13 mm positive tracking residual is grasp-compatible.
        for descent_steps in (35, 20):
            state = sdk.get_state()
            can = list(state["objects"]["can"]["pos"])
            policy_move(
                sdk, [can[0], can[1], can[2]],
                yaw, "open", descent_steps, 0.006
            )
            state = sdk.get_state()
            if policy_grasp_aligned(state):
                break

        state = sdk.get_state()
        if not policy_grasp_aligned(state):
            can = list(state["objects"]["can"]["pos"])
            eef = state["eef_pos"]
            z_correction = min(
                0.005, max(0.0, eef[2] - can[2] - 0.005)
            )
            policy_move(
                sdk, [can[0], can[1], can[2] - z_correction],
                yaw, "open", 18, 0.005
            )

        state = sdk.get_state()
        if not policy_grasp_aligned(state):
            continue

        before = list(state["objects"]["can"]["pos"])
        if not policy_grip(sdk, "close", 20):
            break
        state = sdk.get_state()
        if not policy_holding(state):
            continue

        # Check the lift early so an empty or slipping grasp does not
        # consume the entire vertical-motion allowance before recovery.
        eef = state["eef_pos"]
        lift_target = [eef[0], eef[1], clearance]
        policy_move(sdk, lift_target, yaw, "close", 22, 0.008)

        state = sdk.get_state()
        lifted = state["objects"]["can"]["pos"]
        if (
            not policy_holding(state)
            or lifted[2] - before[2] < 0.04
        ):
            # The next iteration opens and reads the live pose again.
            continue

        policy_move(sdk, lift_target, yaw, "close", 23, 0.008)
        state = sdk.get_state()
        lifted = state["objects"]["can"]["pos"]
        if policy_holding(state) and lifted[2] - before[2] >= 0.04:
            grasped = True
            break

    if not grasped:
        policy_grip(sdk, "open", 10)
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    # Finish the vertical lift before any long horizontal motion.
    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    goal = state["goal_pos"]
    transport_height = max(1.075, goal[2] + 0.20, can[2])
    if can[2] < transport_height - 0.008 or eef[2] < 1.06:
        policy_move(
            sdk,
            [
                eef[0],
                eef[1],
                max(1.075, transport_height + eef[2] - can[2]),
            ],
            yaw, "close", 18, 0.008
        )

    state = sdk.get_state()
    if (
        not policy_holding(state)
        or state["objects"]["can"]["pos"][2] < 1.06
        or state["eef_pos"][2] < 1.06
    ):
        return {"status": "failed", "reason": "High clearance lift unverified"}

    # Preserve the placement reserve while correcting live object position.
    for correction in range(4):
        state = sdk.get_state()
        if not policy_holding(state):
            return {"status": "failed", "reason": "Grasp lost during transport"}

        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        goal = state["goal_pos"]
        horizontal_error = hypot(can[0] - goal[0], can[1] - goal[1])
        if horizontal_error <= 0.008 and can[2] >= 1.06:
            break

        remaining = min(400, state["max_steps"]) - state["steps_used"]
        budget = min(65 if correction == 0 else 20, remaining - 76)
        if budget <= 0:
            break

        offset = [eef[i] - can[i] for i in range(3)]
        policy_move(
            sdk,
            [
                goal[0] + offset[0],
                goal[1] + offset[1],
                max(1.075, transport_height + offset[2]),
            ],
            yaw, "close", budget,
            0.008 if correction == 0 else 0.005
        )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    aligned = (
        policy_holding(state)
        and hypot(can[0] - goal[0], can[1] - goal[1]) <= 0.008
        and can[2] >= 1.06
    )
    if not aligned:
        return {"status": "failed", "reason": "Could not verify alignment above target"}

    # Descend in two closed-loop stages, refreshing the grasp offset.
    for height, steps in [(0.105, 22), (0.025, 28)]:
        state = sdk.get_state()
        if not policy_holding(state):
            return {"status": "failed", "reason": "Grasp lost during descent"}

        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        goal = state["goal_pos"]
        offset = [eef[i] - can[i] for i in range(3)]
        remaining = min(400, state["max_steps"]) - state["steps_used"]
        budget = min(steps, remaining - 26)
        if budget <= 0:
            return {"status": "failed", "reason": "Insufficient time for descent"}

        policy_move(
            sdk,
            [
                goal[0] + offset[0],
                goal[1] + offset[1],
                goal[2] + height + offset[2],
            ],
            yaw, "close", budget, 0.008
        )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    remaining = min(400, state["max_steps"]) - state["steps_used"]
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.008
        or can[2] - goal[2] > 0.045
    ) and remaining > 34:
        eef = state["eef_pos"]
        offset = [eef[i] - can[i] for i in range(3)]
        policy_move(
            sdk,
            [
                goal[0] + offset[0],
                goal[1] + offset[1],
                goal[2] + 0.025 + offset[2],
            ],
            yaw, "close", min(16, remaining - 26), 0.006
        )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    if (
        not policy_holding(state)
        or hypot(can[0] - goal[0], can[1] - goal[1]) > 0.008
        or not -0.015 <= can[2] - goal[2] <= 0.045
    ):
        return {"status": "failed", "reason": "Release position unverified"}

    if not policy_grip(sdk, "open", 18):
        return {"status": "failed", "reason": "Insufficient time to release"}

    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.16],
        yaw, "open", 25, 0.015
    )

    state = sdk.get_state()
    remaining = min(400, state["max_steps"]) - state["steps_used"]
    if remaining > 0:
        policy_grip(sdk, "open", min(12, remaining))

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    horizontal_error = hypot(can[0] - goal[0], can[1] - goal[1])
    height_error = can[2] - goal[2]
    separation = sqrt(sum(
        (state["eef_pos"][i] - can[i]) ** 2 for i in range(3)
    ))
    placed = (
        horizontal_error < 0.035
        and -0.025 < height_error < 0.045
        and state["gripper_width"] > 0.055
        and separation > 0.08
        and state["eef_pos"][2] - can[2] > 0.075
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Placement verified geometrically" if placed else "Placement unverified",
        "horizontal_error_m": horizontal_error,
        "height_error_m": height_error,
        "steps_used": state["steps_used"],
    }
