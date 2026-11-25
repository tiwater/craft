def policy_move(sdk, pos, yaw, gripper, limit=50, tol=0.012):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        pos,
        yaw=yaw,
        gripper=gripper,
        max_steps=min(limit, remaining),
        tol=tol,
    )
    return result["reached"]


def policy_grip(sdk, action, steps=12):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining > 0:
        return sdk.set_gripper(action, steps=min(steps, remaining))
    return {"gripper_width": state["gripper_width"], "grasped_hint": False}


def basket_target(state, basket):
    sites = state["sites"]
    preferred = basket + "_contain_region"
    if preferred in sites:
        return list(sites[preferred])
    for name in sites:
        if basket in name and "contain" in name:
            return list(sites[name])
    pos = state["objects"][basket]["pos"]
    return [pos[0], pos[1], pos[2] + 0.07]


def run_policy(sdk):
    state = sdk.get_state()
    pudding = None
    basket = None
    for name in state["objects"]:
        if "chocolate_pudding" in name:
            pudding = name
        if "basket" in name:
            basket = name

    if pudding is None or basket is None:
        return {"status": "failed", "reason": "Required objects unavailable"}

    held = False
    grasp_yaw = state["objects"][pudding]["yaw"]
    for attempt in range(3):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 155:
            break

        obj = list(state["objects"][pudding]["pos"])
        target = basket_target(state, basket)
        grasp_yaw = state["objects"][pudding]["yaw"]
        if attempt == 1:
            grasp_yaw += pi / 2
        while grasp_yaw - state["eef_yaw"] > pi / 2:
            grasp_yaw -= pi
        while grasp_yaw - state["eef_yaw"] < -pi / 2:
            grasp_yaw += pi

        safe_z = max(obj[2] + 0.17, target[2] + 0.14)
        eef = state["eef_pos"]
        policy_grip(sdk, "open", 8)
        if eef[2] < safe_z - 0.025:
            policy_move(
                sdk, [eef[0], eef[1], safe_z],
                grasp_yaw, "open", 35
            )

        aligned = policy_move(
            sdk, [obj[0], obj[1], safe_z],
            grasp_yaw, "open", 50
        )
        if not aligned:
            continue

        state = sdk.get_state()
        obj = list(state["objects"][pudding]["pos"])
        grasp_z = obj[2] + (0.018 if attempt < 2 else 0.008)
        reached = policy_move(
            sdk, [obj[0], obj[1], grasp_z],
            grasp_yaw, "open", 45, 0.006
        )
        if not reached:
            continue

        before = list(sdk.get_state()["objects"][pudding]["pos"])
        policy_grip(sdk, "close", 14)
        state = sdk.get_state()
        eef = state["eef_pos"]
        policy_move(
            sdk, [eef[0], eef[1], safe_z],
            grasp_yaw, "close", 45
        )
        state = sdk.get_state()
        lifted = state["objects"][pudding]["pos"]
        eef = state["eef_pos"]
        held = (
            lifted[2] > before[2] + 0.055
            and hypot(lifted[0] - eef[0], lifted[1] - eef[1]) < 0.08
            and abs(lifted[2] - eef[2]) < 0.12
        )
        if held:
            break

    if not held:
        return {"status": "failed", "reason": "Could not verify a grasp"}

    state = sdk.get_state()
    obj = state["objects"][pudding]["pos"]
    eef = state["eef_pos"]
    offset = [obj[i] - eef[i] for i in range(3)]
    target = basket_target(state, basket)
    carry_z = max(eef[2], target[2] + 0.17 - offset[2])

    policy_move(
        sdk, [eef[0], eef[1], carry_z],
        grasp_yaw, "close", 30
    )
    arrived = policy_move(
        sdk,
        [target[0] - offset[0], target[1] - offset[1], carry_z],
        grasp_yaw, "close", 65
    )
    if not arrived:
        return {"status": "failed", "reason": "Basket approach incomplete"}

    state = sdk.get_state()
    obj = state["objects"][pudding]["pos"]
    eef = state["eef_pos"]
    if (
        hypot(obj[0] - eef[0], obj[1] - eef[1]) > 0.09
        or abs(obj[2] - eef[2]) > 0.13
    ):
        return {"status": "failed", "reason": "Object lost during transport"}

    offset = [obj[i] - eef[i] for i in range(3)]
    target = basket_target(state, basket)
    release = [
        target[0] - offset[0],
        target[1] - offset[1],
        target[2] + 0.035 - offset[2],
    ]
    reached = policy_move(sdk, release, grasp_yaw, "close", 45, 0.008)
    if not reached:
        return {"status": "failed", "reason": "Placement approach incomplete"}

    policy_grip(sdk, "open", 16)
    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], max(carry_z, eef[2] + 0.12)],
        grasp_yaw, "open", 30
    )
    policy_grip(sdk, "open", 12)

    state = sdk.get_state()
    obj = state["objects"][pudding]["pos"]
    target = basket_target(state, basket)
    dx = obj[0] - target[0]
    dy = obj[1] - target[1]
    yaw = state["objects"][basket]["yaw"]
    local_x = cos(yaw) * dx + sin(yaw) * dy
    local_y = -sin(yaw) * dx + cos(yaw) * dy
    placed = (
        abs(local_x) < 0.075
        and abs(local_y) < 0.075
        and target[2] - 0.09 < obj[2] < target[2] + 0.065
        and state["eef_pos"][2] - obj[2] > 0.08
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Placement verified by pose" if placed else "Placement not verified",
    }
