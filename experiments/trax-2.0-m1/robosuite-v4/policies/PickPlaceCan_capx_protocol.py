def policy_distance(a, b):
    return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def policy_budget(sdk, requested, reserve=0):
    state = sdk.get_state()
    return max(0, min(requested, state["max_steps"] - state["steps_used"] - reserve))


def policy_move(sdk, target, yaw, gripper, steps, reserve=0, tol=0.012):
    budget = policy_budget(sdk, steps, reserve)
    if budget > 0:
        sdk.move_to(
            target, yaw=yaw, gripper=gripper, max_steps=budget, tol=tol
        )
    state = sdk.get_state()
    return policy_distance(state["eef_pos"], target)


def run_policy(sdk):
    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or target bin"}

    yaw = state["eef_yaw"]
    grasped = False
    offset = None

    for attempt in range(2):
        if policy_budget(sdk, 400) < 185:
            break

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        hand = list(state["eef_pos"])
        approach_z = max(hand[2], can[2] + 0.14)

        policy_move(
            sdk, [hand[0], hand[1], approach_z],
            yaw, "open", 25, 160
        )
        policy_move(
            sdk, [can[0], can[1], approach_z],
            yaw, "open", 55, 140
        )

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        hand = state["eef_pos"]
        if hypot(hand[0] - can[0], hand[1] - can[1]) > 0.025:
            continue

        target = [can[0], can[1], can[2] + 0.012]
        policy_move(sdk, target, yaw, "open", 40, 115, 0.008)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        hand = state["eef_pos"]
        if (
            hypot(hand[0] - can[0], hand[1] - can[1]) > 0.012
            or abs(hand[2] - can[2] - 0.012) > 0.025
        ):
            policy_move(
                sdk, [can[0], can[1], can[2] + 0.012],
                yaw, "open", 15, 110, 0.008
            )

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        hand = state["eef_pos"]
        if (
            hypot(hand[0] - can[0], hand[1] - can[1]) > 0.018
            or abs(hand[2] - can[2]) > 0.04
        ):
            continue

        baseline_z = can[2]
        budget = policy_budget(sdk, 20, 95)
        if budget < 12:
            break
        sdk.set_gripper("close", steps=budget)

        state = sdk.get_state()
        hand = list(state["eef_pos"])
        policy_move(
            sdk, [hand[0], hand[1], hand[2] + 0.16],
            yaw, "close", 40, 85
        )

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        hand = state["eef_pos"]
        if (
            can[2] - baseline_z > 0.075
            and 0.012 < state["gripper_width"] < 0.075
            and policy_distance(can, hand) < 0.11
        ):
            offset = [can[i] - hand[i] for i in range(3)]
            grasped = True
            break

    if not grasped:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    can = state["objects"]["can"]["pos"]
    hand = list(state["eef_pos"])
    clearance_z = max(can[2], goal[2] + 0.20)
    lift_target = [hand[0], hand[1], clearance_z - offset[2]]
    policy_move(sdk, lift_target, yaw, "close", 30, 95)

    state = sdk.get_state()
    if state["objects"]["can"]["pos"][2] < clearance_z - 0.035:
        return {"status": "failed", "reason": "Insufficient transport clearance"}

    transit = [
        goal[0] - offset[0],
        goal[1] - offset[1],
        clearance_z - offset[2],
    ]
    error = policy_move(sdk, transit, yaw, "close", 80, 60)
    if error > 0.025:
        error = policy_move(sdk, transit, yaw, "close", 20, 55)
    if error > 0.035:
        return {"status": "failed", "reason": "Could not reach target bin"}

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    hand = state["eef_pos"]
    expected = [hand[i] + offset[i] for i in range(3)]
    if policy_distance(can, expected) > 0.045:
        return {"status": "failed", "reason": "Can slipped during transport"}

    offset = [can[i] - hand[i] for i in range(3)]
    placement = [
        goal[0] - offset[0],
        goal[1] - offset[1],
        goal[2] + 0.065 - offset[2],
    ]
    policy_move(sdk, placement, yaw, "close", 40, 30, 0.01)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.04
        or can[2] > goal[2] + 0.12
    ):
        return {"status": "failed", "reason": "Unsafe release position"}

    budget = policy_budget(sdk, 18, 12)
    if budget < 12:
        return {"status": "failed", "reason": "Insufficient release budget"}
    sdk.set_gripper("open", steps=budget)

    state = sdk.get_state()
    hand = list(state["eef_pos"])
    policy_move(
        sdk, [hand[0], hand[1], hand[2] + 0.16],
        yaw, "open", 25
    )
    budget = policy_budget(sdk, 12)
    if budget > 0:
        sdk.set_gripper("open", steps=budget)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) < 0.065
        and goal[2] - 0.06 < can[2] < goal[2] + 0.13
        and state["gripper_width"] > 0.06
        and state["eef_pos"][2] - can[2] > 0.09
    ):
        return {"status": "done", "steps_used": state["steps_used"]}
    return {
        "status": "failed",
        "reason": "Final placement could not be verified",
        "steps_used": state["steps_used"],
    }
