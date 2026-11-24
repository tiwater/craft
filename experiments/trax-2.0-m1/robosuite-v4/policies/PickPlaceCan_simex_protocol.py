def policy_distance(a, b):
    return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def policy_budget(sdk, requested, reserve=0):
    state = sdk.get_state()
    return max(0, min(requested, state["max_steps"] - state["steps_used"] - reserve))


def policy_move(sdk, position, yaw, gripper, steps, reserve=0, tol=0.008):
    budget = policy_budget(sdk, steps, reserve)
    if budget <= 0:
        return False
    result = sdk.move_to(
        position, yaw=yaw, gripper=gripper, max_steps=budget, tol=tol
    )
    return result["reached"]


def run_policy(sdk):
    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or target bin"}

    yaw = state["eef_yaw"]
    grasped = False
    offset = None

    # The original center-height descent stalls near the support surface.
    # Grasp the upper body so the fingers can close without bottoming out.
    for attempt in range(2):
        if policy_budget(sdk, 400) < 175:
            break

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        hand = list(state["eef_pos"])
        approach_z = max(hand[2], can[2] + 0.14)
        grasp_yaw = yaw + attempt * pi / 2.0

        policy_move(
            sdk, [hand[0], hand[1], approach_z],
            grasp_yaw, "open", 25, 145
        )
        state = sdk.get_state()
        if state["eef_pos"][2] < can[2] + 0.09:
            continue

        can = list(state["objects"]["can"]["pos"])
        policy_move(
            sdk, [can[0], can[1], approach_z],
            grasp_yaw, "open", 40, 130
        )
        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        hand = state["eef_pos"]
        if hypot(hand[0] - can[0], hand[1] - can[1]) > 0.018:
            continue

        grasp_height = 0.025 if attempt == 0 else 0.035
        policy_move(
            sdk, [can[0], can[1], can[2] + grasp_height],
            grasp_yaw, "open", 35, 115, 0.007
        )

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        hand = state["eef_pos"]
        if (
            hypot(hand[0] - can[0], hand[1] - can[1]) > 0.014
            or hand[2] - can[2] < 0.008
            or hand[2] - can[2] > 0.052
        ):
            continue

        budget = policy_budget(sdk, 20, 100)
        if budget < 12:
            break
        sdk.set_gripper("close", steps=budget)

        state = sdk.get_state()
        baseline_z = state["objects"]["can"]["pos"][2]
        hand = list(state["eef_pos"])
        policy_move(
            sdk, [hand[0], hand[1], hand[2] + 0.12],
            grasp_yaw, "close", 30, 85
        )

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        hand = state["eef_pos"]
        if (
            can[2] - baseline_z > 0.065
            and 0.012 < state["gripper_width"] < 0.075
            and policy_distance(can, hand) < 0.11
        ):
            offset = [can[i] - hand[i] for i in range(3)]
            yaw = grasp_yaw
            grasped = True
            break

    if not grasped:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    can = state["objects"]["can"]["pos"]
    hand = state["eef_pos"]
    clearance_z = max(can[2], goal[2] + 0.23)

    policy_move(
        sdk, [hand[0], hand[1], clearance_z - offset[2]],
        yaw, "close", 30, 95
    )
    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if can[2] < clearance_z - 0.025:
        return {"status": "failed", "reason": "Could not establish bin clearance"}

    transit = [
        goal[0] - offset[0],
        goal[1] - offset[1],
        clearance_z - offset[2],
    ]
    policy_move(sdk, transit, yaw, "close", 75, 60)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    hand = state["eef_pos"]
    expected = [hand[i] + offset[i] for i in range(3)]
    if policy_distance(can, expected) > 0.045:
        return {"status": "failed", "reason": "Can slipped during transport"}
    if hypot(can[0] - goal[0], can[1] - goal[1]) > 0.035:
        return {"status": "failed", "reason": "Could not reach target bin"}

    offset = [can[i] - hand[i] for i in range(3)]
    placement = [
        goal[0] - offset[0],
        goal[1] - offset[1],
        goal[2] + 0.065 - offset[2],
    ]
    policy_move(sdk, placement, yaw, "close", 45, 35, 0.008)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.04
        or can[2] > goal[2] + 0.11
        or can[2] < goal[2] - 0.025
    ):
        return {"status": "failed", "reason": "Could not reach release position"}

    budget = policy_budget(sdk, 18, 17)
    if budget < 12:
        return {"status": "failed", "reason": "Insufficient time for release"}
    sdk.set_gripper("open", steps=budget)

    state = sdk.get_state()
    hand = state["eef_pos"]
    policy_move(
        sdk, [hand[0], hand[1], hand[2] + 0.16],
        yaw, "open", 25, 5
    )
    budget = policy_budget(sdk, 8)
    if budget > 0:
        sdk.set_gripper("open", steps=budget)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    centered = hypot(can[0] - goal[0], can[1] - goal[1]) < 0.065
    inside_height = goal[2] - 0.025 < can[2] < goal[2] + 0.13
    released = (
        state["gripper_width"] > 0.06
        and state["eef_pos"][2] - can[2] > 0.09
    )
    if centered and inside_height and released:
        return {"status": "done", "steps_used": state["steps_used"]}
    return {
        "status": "failed",
        "reason": "Final placement could not be verified",
        "steps_used": state["steps_used"],
    }
