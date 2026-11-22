def policy_move(sdk, target, gripper, limit=45, tol=0.018):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    sdk.move_to(
        target,
        gripper=gripper,
        max_steps=min(limit, remaining),
        tol=tol,
    )
    state = sdk.get_state()
    eef = state["eef_pos"]
    return sqrt(sum((eef[i] - target[i]) ** 2 for i in range(3))) < 0.027


def policy_gripper(sdk, action, steps=12):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    sdk.set_gripper(action, steps=min(steps, remaining))
    return True


def run_policy(sdk):
    state = sdk.get_state()
    if state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing target position"}

    source_z = state["objects"]["can"]["pos"][2]
    clearance = max(1.12, source_z + 0.24)
    grasped = False

    # OSC motion can settle about a centimeter short of its command.
    # Judge measured progress instead of requiring millimeter convergence.
    for attempt in range(2):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 175:
            break

        eef = state["eef_pos"]
        if not policy_move(sdk, [eef[0], eef[1], clearance], "open"):
            return {"status": "failed", "reason": "Cannot reach approach height"}

        policy_gripper(sdk, "open", 8)
        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        if not policy_move(sdk, [can[0], can[1], clearance], "open"):
            return {"status": "failed", "reason": "Cannot align above can"}

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        grasp_z = can[2] - 0.005 - 0.008 * attempt
        policy_move(sdk, [can[0], can[1], grasp_z], "open", 45)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]

        # Correct residual lateral error before closing around the cylinder.
        if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.008:
            target = [
                can[0] + max(-0.012, min(0.012, can[0] - eef[0])),
                can[1] + max(-0.012, min(0.012, can[1] - eef[1])),
                grasp_z,
            ]
            policy_move(sdk, target, "open", 15, 0.012)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - can[0], eef[1] - can[1]) > 0.022
            or abs(eef[2] - can[2]) > 0.04
        ):
            continue

        before_lift = list(can)
        policy_gripper(sdk, "close", 18)
        eef = sdk.get_state()["eef_pos"]
        policy_move(sdk, [eef[0], eef[1], eef[2] + 0.11], "close", 32)

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        grasped = (
            can[2] - before_lift[2] > 0.055
            and hypot(can[0] - eef[0], can[1] - eef[1]) < 0.055
            and abs(can[2] - eef[2]) < 0.12
        )
        if grasped:
            break
        policy_gripper(sdk, "open", 8)

    if not grasped:
        return {"status": "failed", "reason": "Could not establish a grasp"}

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    offset = [can[i] - eef[i] for i in range(3)]

    # Keep the can's bottom above the divider, including tracking margin.
    transport_z = max(clearance, 1.115 - offset[2])
    if not policy_move(
        sdk, [eef[0], eef[1], transport_z], "close", 40
    ):
        return {"status": "failed", "reason": "Cannot reach transport height"}

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if can[2] < 1.095:
        return {"status": "failed", "reason": "Can did not clear divider"}

    eef = state["eef_pos"]
    offset = [can[i] - eef[i] for i in range(3)]
    goal = list(state["goal_pos"])
    target = [goal[0] - offset[0], goal[1] - offset[1], transport_z]

    if not policy_move(sdk, target, "close", 65):
        return {"status": "failed", "reason": "Cannot reach target bin"}

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    if (
        can[2] < 1.085
        or hypot(can[0] - goal[0], can[1] - goal[1]) > 0.055
    ):
        return {"status": "failed", "reason": "Can slipped during transport"}

    if state["max_steps"] - state["steps_used"] < 55:
        return {"status": "failed", "reason": "Insufficient placement time"}

    offset = [can[i] - eef[i] for i in range(3)]
    release_z = max(source_z + 0.025, goal[2] + 0.035)
    target = [
        goal[0] - offset[0],
        goal[1] - offset[1],
        release_z - offset[2],
    ]
    if not policy_move(sdk, target, "close", 35):
        return {"status": "failed", "reason": "Cannot lower into target bin"}

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.055
        or can[2] > release_z + 0.035
    ):
        return {"status": "failed", "reason": "Unsafe release position"}

    policy_gripper(sdk, "open", 14)
    eef = sdk.get_state()["eef_pos"]
    policy_move(sdk, [eef[0], eef[1], transport_z], "open", 28)
    policy_gripper(sdk, "open", 10)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    placed = (
        hypot(can[0] - goal[0], can[1] - goal[1]) < 0.06
        and goal[2] - 0.045 < can[2] < goal[2] + 0.055
        and state["eef_pos"][2] - can[2] > 0.09
        and state["gripper_width"] > 0.06
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Can placed in target bin" if placed else "Placement unconfirmed",
        "steps_used": state["steps_used"],
    }
