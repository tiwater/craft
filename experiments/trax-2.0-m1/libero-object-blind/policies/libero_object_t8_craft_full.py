def policy_move(sdk, pos, yaw, gripper, limit=50, tol=0.012):
    state = sdk.get_state()
    remaining = min(450, state["max_steps"]) - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        pos, yaw=yaw, gripper=gripper,
        max_steps=min(limit, remaining), tol=tol
    )
    return result["reached"]


def policy_grip(sdk, action, steps=12):
    state = sdk.get_state()
    remaining = min(450, state["max_steps"]) - state["steps_used"]
    if remaining > 0:
        return sdk.set_gripper(action, steps=min(steps, remaining))
    return {"gripper_width": state["gripper_width"], "grasped_hint": False}


def basket_target(state, basket):
    preferred = basket + "_contain_region"
    if preferred in state["sites"]:
        return list(state["sites"][preferred])
    for name in state["sites"]:
        if basket in name and "contain" in name:
            return list(state["sites"][name])
    pos = state["objects"][basket]["pos"]
    return [pos[0], pos[1], pos[2] + 0.07]


def clearance_height(state, target):
    height = target[2]
    for name in state["objects"]:
        height = max(height, state["objects"][name]["pos"][2])
    for name in state["fixtures"]:
        height = max(height, state["fixtures"][name]["pos"][2])
    return height + 0.16


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
    grasp_yaw = state["eef_yaw"]
    for attempt in range(2):
        state = sdk.get_state()
        if min(450, state["max_steps"]) - state["steps_used"] < 205:
            break
        obj = list(state["objects"][pudding]["pos"])
        target = basket_target(state, basket)
        safe_z = clearance_height(state, target)
        grasp_yaw = state["objects"][pudding]["yaw"] + attempt * pi / 2
        while grasp_yaw - state["eef_yaw"] > pi / 2:
            grasp_yaw -= pi
        while grasp_yaw - state["eef_yaw"] < -pi / 2:
            grasp_yaw += pi

        eef = list(state["eef_pos"])
        policy_move(
            sdk, [eef[0], eef[1], safe_z],
            grasp_yaw, "open", 30
        )
        policy_grip(sdk, "open", 18)
        if not policy_move(
            sdk, [obj[0], obj[1], safe_z],
            grasp_yaw, "open", 40, 0.006
        ):
            continue

        state = sdk.get_state()
        obj = list(state["objects"][pudding]["pos"])
        policy_move(
            sdk, [obj[0], obj[1], obj[2] + 0.045],
            grasp_yaw, "open", 25, 0.004
        )
        state = sdk.get_state()
        obj = list(state["objects"][pudding]["pos"])
        policy_move(
            sdk, [obj[0], obj[1], obj[2] + 0.002],
            grasp_yaw, "open", 22, 0.003
        )

        state = sdk.get_state()
        obj = list(state["objects"][pudding]["pos"])
        eef = state["eef_pos"]
        lateral_error = hypot(eef[0] - obj[0], eef[1] - obj[1])
        if lateral_error > 0.006:
            policy_move(
                sdk, [obj[0], obj[1], max(obj[2] + 0.002, eef[2])],
                grasp_yaw, "open", 12, 0.003
            )

        state = sdk.get_state()
        before = list(state["objects"][pudding]["pos"])
        eef = list(state["eef_pos"])
        if (
            hypot(eef[0] - before[0], eef[1] - before[1]) > 0.012
            or abs(eef[2] - before[2]) > 0.035
        ):
            continue

        # A floor-limited descent may miss its tolerance while still
        # placing the fingers around this short object.
        policy_grip(sdk, "close", 24)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.075],
            grasp_yaw, "close", 24, 0.006
        )
        state = sdk.get_state()
        lifted = state["objects"][pudding]["pos"]
        eef = state["eef_pos"]
        held = (
            lifted[2] - before[2] > 0.04
            and hypot(lifted[0] - eef[0], lifted[1] - eef[1]) < 0.055
            and abs(lifted[2] - eef[2]) < 0.10
            and state["gripper_width"] > 0.01
        )
        if held:
            break

    if not held:
        return {"status": "failed", "reason": "Could not verify a grasp"}

    state = sdk.get_state()
    obj = list(state["objects"][pudding]["pos"])
    eef = list(state["eef_pos"])
    offset = [obj[index] - eef[index] for index in range(3)]
    target = basket_target(state, basket)
    carry_z = max(eef[2], clearance_height(state, target) - min(0.0, offset[2]))
    if not policy_move(
        sdk, [eef[0], eef[1], carry_z],
        grasp_yaw, "close", 35, 0.008
    ):
        return {"status": "failed", "reason": "Clearance lift incomplete"}

    if not policy_move(
        sdk, [target[0] - offset[0], target[1] - offset[1], carry_z],
        grasp_yaw, "close", 65, 0.005
    ):
        return {"status": "failed", "reason": "Basket approach incomplete"}

    state = sdk.get_state()
    obj = list(state["objects"][pudding]["pos"])
    eef = list(state["eef_pos"])
    current_offset = [obj[index] - eef[index] for index in range(3)]
    if max(abs(current_offset[index] - offset[index]) for index in range(3)) > 0.035:
        return {"status": "failed", "reason": "Object slipped during transport"}

    offset = current_offset
    target = basket_target(state, basket)
    for height in [0.065, 0.02]:
        release = [
            target[0] - offset[0],
            target[1] - offset[1],
            target[2] + height - offset[2],
        ]
        if not policy_move(sdk, release, grasp_yaw, "close", 30, 0.005):
            return {"status": "failed", "reason": "Placement descent incomplete"}

    state = sdk.get_state()
    obj = state["objects"][pudding]["pos"]
    if hypot(obj[0] - target[0], obj[1] - target[1]) > 0.025:
        return {"status": "failed", "reason": "Release alignment not verified"}

    policy_grip(sdk, "open", 18)
    state = sdk.get_state()
    eef = list(state["eef_pos"])
    policy_move(
        sdk, [eef[0], eef[1], carry_z],
        grasp_yaw, "open", 30, 0.012
    )
    state = sdk.get_state()
    first = list(state["objects"][pudding]["pos"])
    remaining = min(450, state["max_steps"]) - state["steps_used"]
    if remaining < 8:
        return {"status": "failed", "reason": "Insufficient time to verify settling"}
    policy_grip(sdk, "open", 8)
    state = sdk.get_state()
    obj = state["objects"][pudding]["pos"]
    target = basket_target(state, basket)
    dx = obj[0] - target[0]
    dy = obj[1] - target[1]
    yaw = state["objects"][basket]["yaw"]
    local_x = cos(yaw) * dx + sin(yaw) * dy
    local_y = -sin(yaw) * dx + cos(yaw) * dy
    stable = (
        hypot(obj[0] - first[0], obj[1] - first[1]) < 0.004
        and abs(obj[2] - first[2]) < 0.004
    )
    placed = (
        stable
        and abs(local_x) < 0.075
        and abs(local_y) < 0.075
        and target[2] - 0.09 < obj[2] < target[2] + 0.065
        and state["eef_pos"][2] - obj[2] > 0.08
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Stable placement verified" if placed else "Placement not verified",
    }
