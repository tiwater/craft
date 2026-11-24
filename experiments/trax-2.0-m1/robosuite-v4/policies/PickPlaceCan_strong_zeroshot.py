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

    for attempt in range(2):
        state = sdk.get_state()
        if policy_budget(sdk, 400) < 175:
            break

        can = list(state["objects"]["can"]["pos"])
        approach_z = max(state["eef_pos"][2], can[2] + 0.15)

        if not policy_move(
            sdk,
            [state["eef_pos"][0], state["eef_pos"][1], approach_z],
            yaw, "open", 25, 140
        ):
            continue

        if not policy_move(
            sdk, [can[0], can[1], approach_z],
            yaw, "open", 40, 120
        ):
            continue

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        if not policy_move(sdk, can, yaw, "open", 35, 100, 0.005):
            continue

        budget = policy_budget(sdk, 18, 85)
        if budget < 12:
            break
        sdk.set_gripper("close", steps=budget)

        state = sdk.get_state()
        baseline_z = state["objects"]["can"]["pos"][2]
        hand = list(state["eef_pos"])
        policy_move(
            sdk, [hand[0], hand[1], hand[2] + 0.15],
            yaw, "close", 35, 65
        )

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        hand = state["eef_pos"]
        width = state["gripper_width"]
        if (
            can[2] - baseline_z > 0.07
            and 0.012 < width < 0.075
            and policy_distance(can, hand) < 0.12
        ):
            offset = [can[i] - hand[i] for i in range(3)]
            grasped = True
            break

    if not grasped:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    can = state["objects"]["can"]["pos"]
    clearance_z = max(can[2], goal[2] + 0.23)
    hand = state["eef_pos"]

    if not policy_move(
        sdk, [hand[0], hand[1], clearance_z - offset[2]],
        yaw, "close", 30, 75
    ):
        return {"status": "failed", "reason": "Could not establish bin clearance"}

    transit = [
        goal[0] - offset[0],
        goal[1] - offset[1],
        clearance_z - offset[2],
    ]
    if not policy_move(sdk, transit, yaw, "close", 55, 55):
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
    if not policy_move(sdk, placement, yaw, "close", 40, 30, 0.006):
        return {"status": "failed", "reason": "Could not reach release position"}

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if hypot(can[0] - goal[0], can[1] - goal[1]) > 0.045:
        return {"status": "failed", "reason": "Can is not centered over target"}

    budget = policy_budget(sdk, 18, 12)
    if budget < 12:
        return {"status": "failed", "reason": "Insufficient time for release"}
    sdk.set_gripper("open", steps=budget)

    state = sdk.get_state()
    hand = state["eef_pos"]
    policy_move(
        sdk, [hand[0], hand[1], hand[2] + 0.16],
        yaw, "open", 25, 0
    )

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
