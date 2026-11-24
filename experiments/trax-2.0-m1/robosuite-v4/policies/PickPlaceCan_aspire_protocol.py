def policy_distance(a, b):
    return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def policy_budget(sdk, requested, reserve=0):
    state = sdk.get_state()
    return max(0, min(requested, state["max_steps"] - state["steps_used"] - reserve))


def policy_move(sdk, target, yaw, gripper, steps, reserve=0, tol=0.012):
    budget = policy_budget(sdk, steps, reserve)
    if budget > 0:
        sdk.move_to(
            target, yaw=yaw, gripper=gripper,
            max_steps=budget, tol=tol
        )
    return policy_distance(sdk.get_state()["eef_pos"], target) < 0.025


def policy_attached(state, offset, minimum_height):
    can = state["objects"]["can"]["pos"]
    hand = state["eef_pos"]
    expected = [hand[i] + offset[i] for i in range(3)]
    return (
        0.012 < state["gripper_width"] < 0.075
        and can[2] > minimum_height
        and policy_distance(can, expected) < 0.04
    )


def run_policy(sdk):
    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or target bin"}

    yaw = state["eef_yaw"]
    held = False
    offset = [0.0, 0.0, 0.0]
    baseline_z = state["objects"]["can"]["pos"][2]

    for attempt in range(3):
        if policy_budget(sdk, 400) < 180:
            break

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        hand = list(state["eef_pos"])
        approach_z = max(hand[2], can[2] + 0.14)

        policy_move(
            sdk, [hand[0], hand[1], approach_z],
            yaw, "open", 24, 150
        )

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        if not policy_move(
            sdk, [can[0], can[1], approach_z],
            yaw, "open", 35, 135
        ):
            continue

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        grasp_height = can[2] + (0.014 if attempt == 1 else 0.006)
        policy_move(
            sdk, [can[0], can[1], grasp_height],
            yaw, "open", 32, 115, 0.008
        )

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        hand = list(state["eef_pos"])
        if (
            hypot(hand[0] - can[0], hand[1] - can[1]) > 0.014
            or not -0.012 < hand[2] - can[2] < 0.035
        ):
            continue

        baseline_z = can[2]
        budget = policy_budget(sdk, 22, 100)
        if budget < 16:
            break
        sdk.set_gripper("close", steps=budget)

        state = sdk.get_state()
        if not 0.012 < state["gripper_width"] < 0.075:
            continue

        hand = list(state["eef_pos"])
        policy_move(
            sdk, [hand[0], hand[1], hand[2] + 0.07],
            yaw, "close", 24, 90
        )

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        hand = list(state["eef_pos"])
        if (
            can[2] - baseline_z < 0.04
            or not 0.012 < state["gripper_width"] < 0.075
            or policy_distance(can, hand) > 0.10
        ):
            continue

        offset = [can[i] - hand[i] for i in range(3)]
        goal = list(state["goal_pos"])
        clearance_z = max(baseline_z + 0.17, goal[2] + 0.18)
        policy_move(
            sdk,
            [hand[0], hand[1], clearance_z - offset[2]],
            yaw, "close", 30, 80
        )

        state = sdk.get_state()
        if not policy_attached(state, offset, baseline_z + 0.10):
            continue

        held = True
        break

    if not held:
        return {
            "status": "failed",
            "reason": "Could not verify lift",
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    hand = list(state["eef_pos"])
    offset = [can[i] - hand[i] for i in range(3)]
    carry_z = max(hand[2], clearance_z - offset[2])
    aligned = False

    # Use one destination throughout transit. Short waypoint moves previously
    # consumed the carry budget while the can was still securely held.
    for segment in range(16):
        state = sdk.get_state()
        if not policy_attached(state, offset, baseline_z + 0.10):
            return {
                "status": "failed",
                "reason": "Grasp lost during transport",
                "steps_used": state["steps_used"],
            }

        can = list(state["objects"]["can"]["pos"])
        hand = list(state["eef_pos"])
        goal = list(state["goal_pos"])
        if hypot(can[0] - goal[0], can[1] - goal[1]) < 0.018:
            aligned = True
            break

        budget = policy_budget(sdk, 14, 72)
        if budget <= 0:
            break

        current_offset = [can[i] - hand[i] for i in range(3)]
        policy_move(
            sdk,
            [
                goal[0] - current_offset[0],
                goal[1] - current_offset[1],
                carry_z,
            ],
            yaw, "close", budget, 72, 0.008
        )

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    goal = list(state["goal_pos"])
    aligned = aligned or hypot(can[0] - goal[0], can[1] - goal[1]) < 0.03

    if not aligned or not policy_attached(state, offset, baseline_z + 0.10):
        return {
            "status": "failed",
            "reason": "Could not safely align above bin",
            "steps_used": state["steps_used"],
        }

    # Correct horizontal error during descent, preserving time to open and clear.
    for segment in range(5):
        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        hand = list(state["eef_pos"])
        goal = list(state["goal_pos"])

        if (
            hypot(can[0] - goal[0], can[1] - goal[1]) < 0.025
            and goal[2] - 0.025 < can[2] < goal[2] + 0.075
        ):
            break

        budget = policy_budget(sdk, 10, 34)
        if budget <= 0:
            break

        current_offset = [can[i] - hand[i] for i in range(3)]
        policy_move(
            sdk,
            [
                goal[0] - current_offset[0],
                goal[1] - current_offset[1],
                goal[2] + 0.045 - current_offset[2],
            ],
            yaw, "close", budget, 34, 0.008
        )

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    goal = list(state["goal_pos"])
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.04
        or not goal[2] - 0.03 < can[2] < goal[2] + 0.11
    ):
        return {
            "status": "failed",
            "reason": "Unsafe release position",
            "steps_used": state["steps_used"],
        }

    budget = policy_budget(sdk, 16, 18)
    if budget < 12:
        return {"status": "failed", "reason": "Insufficient release budget"}
    sdk.set_gripper("open", steps=budget)

    state = sdk.get_state()
    hand = list(state["eef_pos"])
    policy_move(
        sdk, [hand[0], hand[1], hand[2] + 0.16],
        yaw, "open", 22, 5
    )

    budget = policy_budget(sdk, 10)
    if budget > 0:
        sdk.set_gripper("open", steps=budget)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    verified = (
        hypot(can[0] - goal[0], can[1] - goal[1]) < 0.06
        and goal[2] - 0.03 < can[2] < goal[2] + 0.08
        and state["gripper_width"] > 0.06
        and state["eef_pos"][2] - can[2] > 0.09
    )
    return {
        "status": "done" if verified else "failed",
        "reason": "Placement verified" if verified else "Placement not verified",
        "steps_used": state["steps_used"],
    }
