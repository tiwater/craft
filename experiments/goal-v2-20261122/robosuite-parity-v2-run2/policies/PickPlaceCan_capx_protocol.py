def policy_remaining(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def policy_distance(a, b):
    return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def policy_move(sdk, target, yaw, grip, steps, tolerance=0.012):
    budget = min(steps, policy_remaining(sdk))
    if budget <= 0:
        return False
    sdk.move_to(
        target, yaw=yaw, gripper=grip,
        max_steps=budget, tol=tolerance
    )
    return policy_distance(sdk.get_state()["eef_pos"], target) <= tolerance


def policy_gripper(sdk, action, steps):
    budget = min(steps, policy_remaining(sdk))
    if budget > 0:
        sdk.set_gripper(action, steps=budget)


def policy_target(position, offset):
    return [position[i] + offset[i] for i in range(3)]


def policy_holding(state, offset):
    return (
        0.012 < state["gripper_width"] < 0.075
        and policy_distance(
            state["eef_pos"],
            policy_target(state["objects"]["can"]["pos"], offset)
        ) < 0.04
    )


def run_policy(sdk):
    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or target bin"}

    yaw = state["eef_yaw"]
    offset = None

    for attempt in range(2):
        if policy_remaining(sdk) < 190:
            break

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        approach_z = max(state["eef_pos"][2], can[2] + 0.15)

        policy_move(
            sdk,
            [state["eef_pos"][0], state["eef_pos"][1], approach_z],
            yaw, "open", 25, 0.018
        )
        policy_move(
            sdk, [can[0], can[1], approach_z],
            yaw, "open", 45, 0.012
        )

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        if hypot(
            state["eef_pos"][0] - can[0],
            state["eef_pos"][1] - can[1]
        ) > 0.025:
            continue

        target = [can[0], can[1], can[2] + 0.005]
        policy_move(sdk, target, yaw, "open", 55, 0.009)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        horizontal_error = hypot(
            state["eef_pos"][0] - can[0],
            state["eef_pos"][1] - can[1]
        )
        vertical_offset = state["eef_pos"][2] - can[2]

        # Small residual descent errors still leave the fingers around the can.
        if horizontal_error > 0.015 or not -0.012 <= vertical_offset <= 0.028:
            policy_move(
                sdk, [can[0], can[1], can[2] + 0.005],
                yaw, "open", 18, 0.009
            )
            state = sdk.get_state()
            can = list(state["objects"]["can"]["pos"])
            if (
                hypot(state["eef_pos"][0] - can[0],
                      state["eef_pos"][1] - can[1]) > 0.018
                or not -0.015 <= state["eef_pos"][2] - can[2] <= 0.03
            ):
                continue

        policy_gripper(sdk, "close", 18)
        before = sdk.get_state()
        before_can = list(before["objects"]["can"]["pos"])
        candidate_offset = [
            before["eef_pos"][i] - before_can[i] for i in range(3)
        ]
        lift = list(before["eef_pos"])
        lift[2] += 0.13
        policy_move(sdk, lift, yaw, "close", 45, 0.015)

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        if (
            can[2] - before_can[2] > 0.065
            and policy_holding(state, candidate_offset)
        ):
            offset = [state["eef_pos"][i] - can[i] for i in range(3)]
            break

    if offset is None:
        return {"status": "failed", "reason": "Could not verify lifted grasp"}

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    can = list(state["objects"]["can"]["pos"])
    carry_z = max(can[2], goal[2] + 0.18)
    policy_move(
        sdk, policy_target([can[0], can[1], carry_z], offset),
        yaw, "close", 35, 0.015
    )
    state = sdk.get_state()
    if (
        not policy_holding(state, offset)
        or state["objects"]["can"]["pos"][2] < goal[2] + 0.145
    ):
        return {"status": "failed", "reason": "Insufficient carry clearance"}

    # Reserve enough control ticks for lowering, opening, and retreating.
    transport_steps = min(75, max(0, policy_remaining(sdk) - 85))
    policy_move(
        sdk, policy_target([goal[0], goal[1], carry_z], offset),
        yaw, "close", transport_steps, 0.012
    )
    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if (
        not policy_holding(state, offset)
        or hypot(can[0] - goal[0], can[1] - goal[1]) > 0.025
    ):
        return {"status": "failed", "reason": "Transport alignment failed"}

    release_target = policy_target(
        [goal[0], goal[1], goal[2] + 0.06], offset
    )
    descent_steps = min(45, max(0, policy_remaining(sdk) - 40))
    policy_move(
        sdk, release_target, yaw, "close", descent_steps, 0.012
    )
    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if (
        not policy_holding(state, offset)
        or hypot(can[0] - goal[0], can[1] - goal[1]) > 0.03
        or can[2] > goal[2] + 0.10
        or policy_remaining(sdk) < 30
    ):
        return {"status": "failed", "reason": "Unsafe release position"}

    policy_gripper(sdk, "open", 14)
    state = sdk.get_state()
    retreat = list(state["eef_pos"])
    retreat[2] += 0.15
    policy_move(
        sdk, retreat, yaw, "open",
        min(30, max(0, policy_remaining(sdk) - 8)), 0.018
    )
    previous = list(sdk.get_state()["objects"]["can"]["pos"])
    policy_gripper(sdk, "open", 8)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    error = hypot(can[0] - goal[0], can[1] - goal[1])
    placed = (
        error < 0.055
        and goal[2] - 0.03 <= can[2] <= goal[2] + 0.10
        and policy_distance(can, previous) < 0.015
        and state["gripper_width"] > 0.06
        and policy_distance(state["eef_pos"], can) > 0.10
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Released in target bin" if placed else "Placement unverified",
        "horizontal_error_m": error,
        "steps_used": state["steps_used"],
    }
