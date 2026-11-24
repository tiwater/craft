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


def policy_holding(state, offset):
    can = state["objects"]["can"]["pos"]
    hand = state["eef_pos"]
    expected = [hand[i] + offset[i] for i in range(3)]
    return (
        0.012 < state["gripper_width"] < 0.075
        and policy_distance(can, hand) < 0.12
        and policy_distance(can, expected) < 0.045
    )


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

        policy_move(
            sdk,
            [state["eef_pos"][0], state["eef_pos"][1], approach_z],
            yaw, "open", 25, 140
        )

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        policy_move(
            sdk, [can[0], can[1], approach_z],
            yaw, "open", 40, 120
        )

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        policy_move(sdk, can, yaw, "open", 35, 100, 0.012)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        hand = state["eef_pos"]
        near_can = (
            hypot(hand[0] - can[0], hand[1] - can[1]) < 0.025
            and abs(hand[2] - can[2]) < 0.035
        )

        if not near_can:
            policy_move(sdk, can, yaw, "open", 20, 100, 0.012)
            state = sdk.get_state()
            can = list(state["objects"]["can"]["pos"])
            hand = state["eef_pos"]
            near_can = (
                hypot(hand[0] - can[0], hand[1] - can[1]) < 0.025
                and abs(hand[2] - can[2]) < 0.035
            )

        if not near_can:
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
    can = list(state["objects"]["can"]["pos"])
    clearance_z = max(can[2], state["goal_pos"][2] + 0.23)
    lift_xy = can[:2]
    bias = [0.0, 0.0, 0.012]

    for correction in range(3):
        state = sdk.get_state()
        if not policy_holding(state, offset):
            return {"status": "failed", "reason": "Grasp lost during clearance lift"}
        can = state["objects"]["can"]["pos"]
        hand = state["eef_pos"]
        clearance_z = max(clearance_z, state["goal_pos"][2] + 0.23)
        if can[2] >= clearance_z - 0.015:
            break
        if policy_budget(sdk, 24, 150) <= 0:
            break
        target = [
            hand[0] + lift_xy[0] - can[0],
            hand[1] + lift_xy[1] - can[1],
            hand[2] + clearance_z - can[2] + bias[2],
        ]
        policy_move(sdk, target, yaw, "close", 24, 150, 0.006)
        state = sdk.get_state()
        error_z = clearance_z - state["objects"]["can"]["pos"][2]
        bias[2] = max(0.0, min(0.035, bias[2] + 0.6 * error_z))

    state = sdk.get_state()
    if (
        not policy_holding(state, offset)
        or state["objects"]["can"]["pos"][2] < clearance_z - 0.02
    ):
        return {"status": "failed", "reason": "Object clearance could not be verified"}

    bias = [0.0, 0.0, 0.012]
    aligned = False

    for correction in range(7):
        state = sdk.get_state()
        if not policy_holding(state, offset):
            return {"status": "failed", "reason": "Can slipped during transport"}
        goal = list(state["goal_pos"])
        can = list(state["objects"]["can"]["pos"])
        hand = list(state["eef_pos"])
        clearance_z = max(clearance_z, goal[2] + 0.23)
        error = [
            goal[0] - can[0],
            goal[1] - can[1],
            clearance_z - can[2],
        ]
        if hypot(error[0], error[1]) <= 0.014 and abs(error[2]) <= 0.02:
            aligned = True
            break
        if policy_budget(sdk, 24, 85) <= 0:
            break

        target = [hand[i] + error[i] + bias[i] for i in range(3)]
        policy_move(sdk, target, yaw, "close", 24, 85, 0.005)

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        goal = state["goal_pos"]
        residual = [
            goal[0] - can[0],
            goal[1] - can[1],
            clearance_z - can[2],
        ]
        if hypot(residual[0], residual[1]) < 0.06:
            for axis in range(2):
                bias[axis] = max(
                    -0.025, min(0.025, bias[axis] + 0.65 * residual[axis])
                )
        bias[2] = max(-0.025, min(0.035, bias[2] + 0.6 * residual[2]))

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    aligned = (
        policy_holding(state, offset)
        and hypot(can[0] - goal[0], can[1] - goal[1]) <= 0.018
        and can[2] >= max(clearance_z, goal[2] + 0.23) - 0.025
    )
    if not aligned:
        return {"status": "failed", "reason": "High-clearance goal alignment not verified"}

    bias = [bias[0], bias[1], 0.0]
    for correction in range(4):
        state = sdk.get_state()
        if not policy_holding(state, offset):
            return {"status": "failed", "reason": "Can slipped during placement"}
        can = state["objects"]["can"]["pos"]
        hand = state["eef_pos"]
        goal = state["goal_pos"]
        error = [
            goal[0] - can[0],
            goal[1] - can[1],
            goal[2] + 0.055 - can[2],
        ]
        if hypot(error[0], error[1]) <= 0.015 and abs(error[2]) <= 0.018:
            break
        if policy_budget(sdk, 20, 35) <= 0:
            break
        target = [hand[i] + error[i] + bias[i] for i in range(3)]
        policy_move(sdk, target, yaw, "close", 20, 35, 0.004)
        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        goal = state["goal_pos"]
        residual = [
            goal[0] - can[0],
            goal[1] - can[1],
            goal[2] + 0.055 - can[2],
        ]
        for axis in range(3):
            if abs(residual[axis]) < 0.05:
                bias[axis] = max(
                    -0.025, min(0.025, bias[axis] + 0.65 * residual[axis])
                )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    if not (
        policy_holding(state, offset)
        and hypot(can[0] - goal[0], can[1] - goal[1]) <= 0.02
        and goal[2] + 0.025 <= can[2] <= goal[2] + 0.08
    ):
        return {"status": "failed", "reason": "Release pose could not be verified"}

    budget = policy_budget(sdk, 18, 17)
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
