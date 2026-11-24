def distance_xy(a, b):
    return hypot(a[0] - b[0], a[1] - b[1])


def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def move_bounded(sdk, target, gripper, steps=40, tolerance=0.018, yaw=None):
    budget = min(steps, remaining_steps(sdk))
    if budget <= 0:
        return {"reached": False}
    return sdk.move_to(
        target, yaw=yaw, gripper=gripper,
        max_steps=budget, tol=tolerance
    )


def actuate_bounded(sdk, action, steps=12):
    budget = min(steps, remaining_steps(sdk))
    if budget > 0:
        return sdk.set_gripper(action, steps=budget)
    return None


def grasp_offset(state):
    return [
        state["eef_pos"][i] - state["objects"]["can"]["pos"][i]
        for i in range(3)
    ]


def hand_target(target, offset):
    return [target[i] + offset[i] for i in range(3)]


def run_policy(sdk):
    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or target bin"}

    initial_can = list(state["objects"]["can"]["pos"])
    yaw = state["eef_yaw"]
    held = False
    offset = [0.0, 0.0, 0.0]

    for attempt in range(2):
        if remaining_steps(sdk) < 210:
            break

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        approach_z = max(state["eef_pos"][2], can[2] + 0.15)

        move_bounded(
            sdk,
            [state["eef_pos"][0], state["eef_pos"][1], approach_z],
            "open", steps=20, yaw=yaw
        )
        move_bounded(
            sdk, [can[0], can[1], approach_z],
            "open", steps=35, tolerance=0.014, yaw=yaw
        )

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        if distance_xy(state["eef_pos"], can) > 0.025:
            continue

        grasp_z = can[2] + 0.006
        move_bounded(
            sdk, [can[0], can[1], grasp_z],
            "open", steps=35, tolerance=0.012, yaw=yaw
        )
        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        if (
            distance_xy(state["eef_pos"], can) > 0.020
            or abs(state["eef_pos"][2] - grasp_z) > 0.025
        ):
            continue

        actuate_bounded(sdk, "close", steps=14)
        state = sdk.get_state()
        before_lift = list(state["objects"]["can"]["pos"])
        hand = list(state["eef_pos"])

        move_bounded(
            sdk, [hand[0], hand[1], hand[2] + 0.11],
            "close", steps=30, tolerance=0.018, yaw=yaw
        )
        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        held = (
            can[2] - before_lift[2] > 0.055
            and distance_xy(state["eef_pos"], can) < 0.055
            and 0.012 < state["gripper_width"] < 0.070
        )
        if held:
            offset = grasp_offset(state)
            break
        actuate_bounded(sdk, "open", steps=8)

    if not held:
        return {"status": "failed", "reason": "Could not verify grasp"}

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    can = list(state["objects"]["can"]["pos"])
    carry_z = max(initial_can[2] + 0.21, goal[2] + 0.23, can[2])

    move_bounded(
        sdk, hand_target([can[0], can[1], carry_z], offset),
        "close", steps=35, tolerance=0.022, yaw=yaw
    )

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    if can[2] < carry_z - 0.035:
        return {"status": "failed", "reason": "Insufficient transport clearance"}

    offset = grasp_offset(state)
    start = list(can)
    for segment in range(1, 4):
        if remaining_steps(sdk) < 75:
            return {"status": "failed", "reason": "Insufficient placement budget"}
        fraction = segment / 3.0
        target = [
            start[0] + fraction * (goal[0] - start[0]),
            start[1] + fraction * (goal[1] - start[1]),
            carry_z,
        ]
        move_bounded(
            sdk, hand_target(target, offset),
            "close", steps=min(38, remaining_steps(sdk) - 65),
            tolerance=0.022, yaw=yaw
        )
        state = sdk.get_state()
        current_offset = grasp_offset(state)
        offset_error = sqrt(sum(
            (current_offset[i] - offset[i]) ** 2 for i in range(3)
        ))
        if offset_error > 0.045 or state["gripper_width"] < 0.012:
            return {"status": "failed", "reason": "Grasp lost during transport"}

    state = sdk.get_state()
    can = list(state["objects"]["can"]["pos"])
    if distance_xy(can, goal) > 0.025:
        offset = grasp_offset(state)
        move_bounded(
            sdk, hand_target([goal[0], goal[1], carry_z], offset),
            "close", steps=min(25, max(0, remaining_steps(sdk) - 60)),
            tolerance=0.015, yaw=yaw
        )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if distance_xy(can, goal) > 0.040:
        return {"status": "failed", "reason": "Can is not aligned with bin"}

    if remaining_steps(sdk) < 45:
        return {"status": "failed", "reason": "Insufficient release budget"}

    offset = grasp_offset(state)
    release_center = [goal[0], goal[1], goal[2] + 0.075]
    move_bounded(
        sdk, hand_target(release_center, offset),
        "close", steps=min(45, remaining_steps(sdk) - 30),
        tolerance=0.018, yaw=yaw
    )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if (
        distance_xy(can, goal) > 0.040
        or can[2] > release_center[2] + 0.040
    ):
        return {"status": "failed", "reason": "Safe release pose not reached"}

    actuate_bounded(sdk, "open", steps=12)
    state = sdk.get_state()
    hand = list(state["eef_pos"])
    move_bounded(
        sdk, [hand[0], hand[1], hand[2] + 0.14],
        "open", steps=min(22, max(0, remaining_steps(sdk) - 8)),
        tolerance=0.022, yaw=yaw
    )
    actuate_bounded(sdk, "open", steps=8)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    placed = (
        distance_xy(can, goal) < 0.075
        and goal[2] - 0.035 <= can[2] <= goal[2] + 0.13
        and state["gripper_width"] > 0.055
        and state["eef_pos"][2] - can[2] > 0.08
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Placement verified" if placed else "Placement verification failed",
        "can_pos": list(can),
        "goal_pos": list(goal),
        "steps_used": state["steps_used"],
    }
