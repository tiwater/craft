def policy_remaining(sdk):
    state = sdk.get_state()
    return max(0, min(450, state["max_steps"]) - state["steps_used"])


def policy_move(sdk, pos, yaw, grip, limit=55, tol=0.012, reserve=0):
    budget = min(limit, max(0, policy_remaining(sdk) - reserve))
    if budget <= 0:
        return False
    result = sdk.move_to(
        pos, yaw=yaw, gripper=grip, max_steps=budget, tol=tol
    )
    return result["reached"]


def policy_grip(sdk, action, steps=12, reserve=0):
    budget = min(steps, max(0, policy_remaining(sdk) - reserve))
    if budget <= 0:
        return False
    sdk.set_gripper(action, steps=budget)
    return True


def policy_target(state, basket):
    sites = state["sites"]
    preferred = basket + "_contain_region"
    if preferred in sites:
        return list(sites[preferred])
    for name in sites:
        if basket in name and "contain" in name:
            return list(sites[name])
    if basket in state["objects"]:
        pos = state["objects"][basket]["pos"]
    else:
        pos = state["fixtures"][basket]["pos"]
    return [pos[0], pos[1], pos[2] + 0.075]


def policy_yaw(current, desired):
    return current + atan2(
        sin(2.0 * (desired - current)),
        cos(2.0 * (desired - current)),
    ) / 2.0


def policy_grasp(sdk, butter):
    # The butter is thin: the previous positive height offsets missed its sides.
    heights = [-0.004, -0.009, -0.004]
    rotations = [pi / 2.0, pi / 2.0, 0.0]
    reserve = 145

    for attempt in range(3):
        if policy_remaining(sdk) < reserve + 75:
            break

        state = sdk.get_state()
        pos = list(state["objects"][butter]["pos"])
        yaw = policy_yaw(
            state["eef_yaw"],
            state["objects"][butter]["yaw"] + rotations[attempt],
        )
        eef = state["eef_pos"]
        clearance = max(eef[2], pos[2] + 0.11)

        policy_grip(sdk, "open", 8, reserve)
        if eef[2] < pos[2] + 0.075:
            policy_move(
                sdk, [eef[0], eef[1], clearance],
                yaw, "open", 22, 0.012, reserve
            )

        if not policy_move(
            sdk, [pos[0], pos[1], clearance],
            yaw, "open", 40, 0.009, reserve
        ):
            continue

        state = sdk.get_state()
        pos = list(state["objects"][butter]["pos"])
        policy_move(
            sdk, [pos[0], pos[1], pos[2] + 0.035],
            yaw, "open", 25, 0.006, reserve
        )

        state = sdk.get_state()
        pos = list(state["objects"][butter]["pos"])
        goal = [pos[0], pos[1], pos[2] + heights[attempt]]
        policy_move(sdk, goal, yaw, "open", 24, 0.003, reserve)

        state = sdk.get_state()
        pos = list(state["objects"][butter]["pos"])
        eef = state["eef_pos"]

        # Contact with the table can prevent reaching the exact low target.
        if (
            hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.014
            or eef[2] - pos[2] > 0.010
        ):
            continue

        before = list(pos)
        if not policy_grip(sdk, "close", 18, reserve):
            break

        state = sdk.get_state()
        eef = state["eef_pos"]
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.045],
            yaw, "close", 20, 0.006, reserve
        )
        state = sdk.get_state()
        lifted = state["objects"][butter]["pos"]
        eef = state["eef_pos"]

        if (
            lifted[2] - before[2] < 0.022
            or hypot(lifted[0] - eef[0], lifted[1] - eef[1]) > 0.055
        ):
            continue

        first_offset = [lifted[i] - eef[i] for i in range(3)]
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.065],
            yaw, "close", 22, 0.008, reserve
        )
        state = sdk.get_state()
        lifted = state["objects"][butter]["pos"]
        eef = state["eef_pos"]
        offset = [lifted[i] - eef[i] for i in range(3)]

        if (
            lifted[2] - before[2] > 0.060
            and all(abs(offset[i] - first_offset[i]) < 0.020 for i in range(3))
        ):
            return {"held": True, "offset": offset, "yaw": yaw}

    return {"held": False}


def run_policy(sdk):
    state = sdk.get_state()
    butter = None
    basket = None
    for name in state["objects"]:
        if "butter" in name:
            butter = name
        if "basket" in name:
            basket = name
    if basket is None:
        for name in state["fixtures"]:
            if "basket" in name:
                basket = name

    if butter is None or basket is None:
        return {"status": "failed", "reason": "Required objects unavailable"}

    grasp = policy_grasp(sdk, butter)
    if not grasp["held"]:
        return {"status": "failed", "reason": "Could not verify a grasp"}

    offset = grasp["offset"]
    grasp_yaw = grasp["yaw"]
    state = sdk.get_state()
    target = policy_target(state, basket)
    clearance = max(
        state["eef_pos"][2],
        target[2] + 0.14 - offset[2],
        max(obj["pos"][2] for obj in state["objects"].values()) + 0.15,
    )
    eef = state["eef_pos"]
    if not policy_move(
        sdk, [eef[0], eef[1], clearance], grasp_yaw, "close", 30
    ):
        return {"status": "failed", "reason": "Could not reach transport height"}

    above = [target[0] - offset[0], target[1] - offset[1], clearance]
    if not policy_move(sdk, above, grasp_yaw, "close", 60, 0.009, 35):
        return {"status": "failed", "reason": "Could not reach basket"}

    state = sdk.get_state()
    obj = state["objects"][butter]["pos"]
    eef = state["eef_pos"]
    if any(abs(obj[i] - eef[i] - offset[i]) > 0.045 for i in range(3)):
        return {"status": "failed", "reason": "Object slipped during transport"}

    offset = [obj[i] - eef[i] for i in range(3)]
    target = policy_target(state, basket)
    release = [
        target[0] - offset[0],
        target[1] - offset[1],
        target[2] + 0.025 - offset[2],
    ]
    if not policy_move(sdk, release, grasp_yaw, "close", 40, 0.008, 25):
        return {"status": "failed", "reason": "Could not reach release position"}

    if not policy_grip(sdk, "open", 12):
        return {"status": "failed", "reason": "Step budget exhausted before release"}

    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.14],
        grasp_yaw, "open", 25
    )
    remaining = policy_remaining(sdk)
    if remaining > 0:
        sdk.step_raw(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
            steps=min(20, remaining),
        )

    state = sdk.get_state()
    target = policy_target(state, basket)
    pos = state["objects"][butter]["pos"]
    if basket in state["objects"]:
        base = state["objects"][basket]["pos"]
    else:
        base = state["fixtures"][basket]["pos"]
    placed = (
        hypot(pos[0] - target[0], pos[1] - target[1]) < 0.055
        and base[2] - 0.01 <= pos[2] <= target[2] + 0.065
        and state["eef_pos"][2] - pos[2] > 0.07
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Placement checked from live poses"
        if placed else "Final placement could not be verified",
        "steps_used": state["steps_used"],
    }
