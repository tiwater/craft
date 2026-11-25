def policy_move(sdk, pos, yaw, gripper, limit=50, tol=0.012):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        pos, yaw=yaw, gripper=gripper,
        max_steps=min(limit, remaining), tol=tol
    )
    return result["reached"]


def policy_grip(sdk, action, steps=12):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining > 0:
        return sdk.set_gripper(action, steps=min(steps, remaining))
    return {"gripper_width": state["gripper_width"], "grasped_hint": False}


def select_entity(entities, stem):
    if stem in entities:
        return stem
    if stem + "_1" in entities:
        return stem + "_1"
    matches = []
    for name in entities:
        if name.startswith(stem + "_"):
            matches.append(name)
    if len(matches) == 1:
        return matches[0]
    return None


def basket_target(state, basket, source):
    sites = state["sites"]
    for suffix in ["_contain_region", "_container_region", "_inside_region"]:
        name = basket + suffix
        if name in sites:
            return list(sites[name])
    matches = []
    for name in sites:
        if name.startswith(basket + "_") and "contain" in name:
            matches.append(name)
    if len(matches) == 1:
        return list(sites[matches[0]])
    pos = state[source][basket]["pos"]
    return [pos[0], pos[1], pos[2] + 0.07]


def run_policy(sdk):
    state = sdk.get_state()
    pudding = select_entity(state["objects"], "chocolate_pudding")
    basket = select_entity(state["objects"], "basket")
    source = "objects"
    if basket is None:
        basket = select_entity(state["fixtures"], "basket")
        source = "fixtures"
    if pudding is None or basket is None:
        return {"status": "failed", "reason": "Required entities unavailable or ambiguous"}

    held = False
    grasp_yaw = state["objects"][pudding]["yaw"]
    for attempt in range(3):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 155:
            break

        obj = list(state["objects"][pudding]["pos"])
        target = basket_target(state, basket, source)
        grasp_yaw = state["objects"][pudding]["yaw"]
        while grasp_yaw - state["eef_yaw"] > pi / 2:
            grasp_yaw -= pi
        while grasp_yaw - state["eef_yaw"] < -pi / 2:
            grasp_yaw += pi

        safe_z = max(obj[2] + 0.17, target[2] + 0.14)
        policy_grip(sdk, "open", 8)
        eef = sdk.get_state()["eef_pos"]
        if eef[2] < safe_z - 0.025:
            policy_move(
                sdk, [eef[0], eef[1], safe_z],
                grasp_yaw, "open", 35
            )

        aligned = policy_move(
            sdk, [obj[0], obj[1], safe_z],
            grasp_yaw, "open", 50,
            0.012 if attempt == 0 else 0.005
        )
        state = sdk.get_state()
        eef = state["eef_pos"]
        obj = list(state["objects"][pudding]["pos"])
        if not aligned and (
            hypot(eef[0] - obj[0], eef[1] - obj[1]) > 0.012
            or eef[2] < safe_z - 0.025
        ):
            continue

        if attempt > 0:
            policy_move(
                sdk, [obj[0], obj[1], obj[2] + 0.065],
                grasp_yaw, "open", 24, 0.004
            )
            state = sdk.get_state()
            obj = list(state["objects"][pudding]["pos"])
            eef = state["eef_pos"]
            if hypot(eef[0] - obj[0], eef[1] - obj[1]) > 0.007:
                continue

        grasp_z = obj[2] + (0.018 if attempt == 0 else 0.005)
        reached = policy_move(
            sdk, [obj[0], obj[1], grasp_z],
            grasp_yaw, "open", 45,
            0.006 if attempt == 0 else 0.003
        )
        state = sdk.get_state()
        eef = state["eef_pos"]
        obj = list(state["objects"][pudding]["pos"])
        if not reached and (
            hypot(eef[0] - obj[0], eef[1] - obj[1]) > 0.009
            or abs(eef[2] - grasp_z) > 0.012
        ):
            continue

        if attempt > 0:
            if hypot(eef[0] - obj[0], eef[1] - obj[1]) > 0.004:
                policy_move(
                    sdk, [obj[0], obj[1], obj[2] + 0.005],
                    grasp_yaw, "open", 12, 0.003
                )
            state = sdk.get_state()
            obj = list(state["objects"][pudding]["pos"])
            eef = state["eef_pos"]
            if hypot(eef[0] - obj[0], eef[1] - obj[1]) > 0.007:
                continue

        before = list(obj)
        policy_grip(sdk, "close", 14 if attempt == 0 else 20)
        state = sdk.get_state()
        eef = state["eef_pos"]

        if attempt > 0:
            policy_move(
                sdk, [eef[0], eef[1], eef[2] + 0.075],
                grasp_yaw, "close", 24, 0.006
            )
            state = sdk.get_state()
            lifted = state["objects"][pudding]["pos"]
            eef = state["eef_pos"]
            if (
                lifted[2] < before[2] + 0.035
                or hypot(lifted[0] - eef[0], lifted[1] - eef[1]) > 0.06
            ):
                continue

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
    target = basket_target(state, basket, source)
    carry_z = max(eef[2], target[2] + 0.17 - offset[2])
    policy_move(
        sdk, [eef[0], eef[1], carry_z],
        grasp_yaw, "close", 30
    )
    arrived = policy_move(
        sdk, [target[0] - offset[0], target[1] - offset[1], carry_z],
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
    target = basket_target(state, basket, source)
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
    target = basket_target(state, basket, source)
    dx = obj[0] - target[0]
    dy = obj[1] - target[1]
    yaw = state[source][basket]["yaw"]
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
        "object": pudding,
        "basket": basket,
        "target": target,
    }
