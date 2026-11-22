def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def distance_xy(a, b):
    return hypot(a[0] - b[0], a[1] - b[1])


def actuate(sdk, action, ticks):
    ticks = min(ticks, remaining_steps(sdk))
    if ticks > 0:
        sdk.set_gripper(action, steps=ticks)


def move_feedback(sdk, target, yaw, gripper, budget, tolerance):
    correction = [0.0, 0.0, 0.0]
    used = 0
    while used < budget and remaining_steps(sdk) > 0:
        state = sdk.get_state()
        error = [target[i] - state["eef_pos"][i] for i in range(3)]
        if sqrt(sum(value * value for value in error)) <= tolerance:
            return True

        ticks = min(12, budget - used, remaining_steps(sdk))
        result = sdk.move_to(
            [target[i] + correction[i] for i in range(3)],
            yaw=yaw,
            gripper=gripper,
            max_steps=ticks,
            tol=0.004,
        )
        used += max(1, result["steps"])
        state = sdk.get_state()
        error = [target[i] - state["eef_pos"][i] for i in range(3)]
        magnitude = sqrt(sum(value * value for value in error))
        if magnitude <= tolerance:
            return True
        if magnitude < 0.065:
            correction = [
                max(-0.04, min(0.04, correction[i] + 0.65 * error[i]))
                for i in range(3)
            ]

    state = sdk.get_state()
    return sqrt(
        sum((target[i] - state["eef_pos"][i]) ** 2 for i in range(3))
    ) <= tolerance


def run_policy(sdk):
    state = sdk.get_state()
    if state["goal_pos"] is None:
        return {"status": "failed", "reason": "missing_goal"}

    yaw = state["eef_yaw"]
    acquired = False
    offset = None

    for attempt in range(2):
        if remaining_steps(sdk) < 190:
            break

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        clearance = max(1.11, can[2] + 0.20, state["eef_pos"][2])

        raised = move_feedback(
            sdk,
            [state["eef_pos"][0], state["eef_pos"][1], clearance],
            yaw, "open", 36, 0.022,
        )
        if not raised and sdk.get_state()["eef_pos"][2] < 1.04:
            return {"status": "failed", "reason": "clearance_unreachable"}

        can = list(sdk.get_state()["objects"]["can"]["pos"])
        if not move_feedback(
            sdk, [can[0], can[1], clearance],
            yaw, "open", 42, 0.012,
        ):
            continue

        can = list(sdk.get_state()["objects"]["can"]["pos"])
        move_feedback(
            sdk, [can[0], can[1], can[2] + 0.005],
            yaw, "open", 48, 0.008,
        )

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        if (
            distance_xy(state["eef_pos"], can) > 0.018
            or abs(state["eef_pos"][2] - can[2] - 0.005) > 0.018
        ):
            continue

        before_lift = list(can)
        actuate(sdk, "close", 18)
        state = sdk.get_state()
        move_feedback(
            sdk,
            [state["eef_pos"][0], state["eef_pos"][1], 1.15],
            yaw, "close", 48, 0.015,
        )

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        offset = [state["eef_pos"][i] - can[i] for i in range(3)]
        acquired = (
            can[2] - before_lift[2] > 0.10
            and can[2] >= 1.105
            and state["eef_pos"][2] >= 1.04
            and distance_xy(state["eef_pos"], can) < 0.065
            and 0.012 < state["gripper_width"] < 0.075
        )
        if acquired:
            break
        actuate(sdk, "open", 10)

    if not acquired:
        return {"status": "failed", "reason": "grasp_not_verified"}

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    can = state["objects"]["can"]["pos"]
    transit_height = max(1.12, can[2], goal[2] + 0.20)
    target = [
        goal[0] + offset[0],
        goal[1] + offset[1],
        transit_height + offset[2],
    ]

    transfer_budget = min(120, remaining_steps(sdk) - 90)
    if transfer_budget <= 0:
        return {"status": "failed", "reason": "insufficient_transfer_budget"}

    move_feedback(
        sdk, target, yaw, "close", transfer_budget, 0.012,
    )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    current_offset = [state["eef_pos"][i] - can[i] for i in range(3)]
    drift = sqrt(sum((current_offset[i] - offset[i]) ** 2 for i in range(3)))
    if (
        can[2] < 1.075
        or state["eef_pos"][2] < 1.04
        or drift > 0.045
        or state["gripper_width"] < 0.012
    ):
        return {"status": "failed", "reason": "grasp_lost"}

    goal = list(state["goal_pos"])
    if distance_xy(can, goal) > 0.022:
        return {"status": "failed", "reason": "transfer_not_aligned"}

    offset = current_offset
    release_target = [
        goal[0] + offset[0],
        goal[1] + offset[1],
        goal[2] + 0.055 + offset[2],
    ]
    budget = min(60, remaining_steps(sdk) - 46)
    if budget <= 0:
        return {"status": "failed", "reason": "insufficient_release_budget"}

    move_feedback(sdk, release_target, yaw, "close", budget, 0.012)
    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = list(state["goal_pos"])
    if (
        distance_xy(can, goal) > 0.035
        or can[2] > goal[2] + 0.095
        or can[2] < goal[2] - 0.035
    ):
        return {"status": "failed", "reason": "release_pose_not_verified"}

    actuate(sdk, "open", 16)
    state = sdk.get_state()
    retreat = [
        state["eef_pos"][0],
        state["eef_pos"][1],
        max(1.13, state["eef_pos"][2] + 0.17),
    ]
    move_feedback(
        sdk, retreat, yaw, "open",
        min(32, max(0, remaining_steps(sdk) - 6)), 0.025,
    )
    actuate(sdk, "open", min(10, remaining_steps(sdk)))

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    placed = (
        distance_xy(can, goal) < 0.065
        and abs(can[2] - goal[2]) < 0.085
        and state["eef_pos"][2] - can[2] > 0.10
        and state["gripper_width"] > 0.055
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "placement_verified" if placed else "placement_not_verified",
        "can_pos": list(can),
        "goal_pos": list(goal),
        "steps_used": state["steps_used"],
    }
