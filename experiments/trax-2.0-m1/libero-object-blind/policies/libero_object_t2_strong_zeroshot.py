def policy_move(sdk, pos, yaw, gripper, limit=45, tol=0.012):
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
    name = basket + "_contain_region"
    if name in sites:
        return list(sites[name])
    for name in sites:
        if basket in name and "contain" in name:
            return list(sites[name])
    pos = state["objects"][basket]["pos"]
    return [pos[0], pos[1], pos[2] + 0.075]


def run_policy(sdk):
    state = sdk.get_state()
    target = None
    basket = None
    for name in state["objects"]:
        if "salad_dressing" in name:
            target = name
        if "basket" in name:
            basket = name
    if target is None or basket is None:
        return {"status": "failed", "reason": "Required objects are missing"}

    grasped = False
    offset = None
    grasp_yaw = state["objects"][target]["yaw"]

    for attempt in range(3):
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining < 190:
            break

        pos = list(state["objects"][target]["pos"])
        destination = basket_target(state, basket)
        grasp_yaw = state["objects"][target]["yaw"]
        if attempt == 1:
            grasp_yaw += pi / 2
        grasp_height = pos[2] + (0.012 if attempt < 2 else 0.028)
        clearance = max(
            state["eef_pos"][2],
            pos[2] + 0.19,
            destination[2] + 0.22,
        )

        policy_grip(sdk, "open", 8)
        eef = sdk.get_state()["eef_pos"]
        if not policy_move(
            sdk, [eef[0], eef[1], clearance], grasp_yaw, "open", 35
        ):
            continue
        if not policy_move(
            sdk, [pos[0], pos[1], clearance], grasp_yaw, "open", 45
        ):
            continue

        state = sdk.get_state()
        pos = list(state["objects"][target]["pos"])
        grasp_height = pos[2] + (0.012 if attempt < 2 else 0.028)
        if not policy_move(
            sdk,
            [pos[0], pos[1], grasp_height],
            grasp_yaw,
            "open",
            45,
            0.007,
        ):
            continue

        before = list(sdk.get_state()["objects"][target]["pos"])
        policy_grip(sdk, "close", 14)
        eef = sdk.get_state()["eef_pos"]
        policy_move(
            sdk,
            [eef[0], eef[1], eef[2] + 0.13],
            grasp_yaw,
            "close",
            35,
        )
        state = sdk.get_state()
        lifted = state["objects"][target]["pos"]
        eef = state["eef_pos"]
        if (
            lifted[2] - before[2] > 0.055
            and hypot(lifted[0] - eef[0], lifted[1] - eef[1]) < 0.08
        ):
            offset = [lifted[i] - eef[i] for i in range(3)]
            grasped = True
            break

    if not grasped:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    destination = basket_target(state, basket)
    eef = state["eef_pos"]
    travel_height = max(eef[2], destination[2] + 0.24 - offset[2])
    if not policy_move(
        sdk, [eef[0], eef[1], travel_height], grasp_yaw, "close", 35
    ):
        return {"status": "failed", "reason": "Could not reach basket clearance"}

    state = sdk.get_state()
    destination = basket_target(state, basket)
    if not policy_move(
        sdk,
        [
            destination[0] - offset[0],
            destination[1] - offset[1],
            travel_height,
        ],
        grasp_yaw,
        "close",
        60,
    ):
        return {"status": "failed", "reason": "Could not reach above basket"}

    state = sdk.get_state()
    pos = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    if abs((pos[2] - eef[2]) - offset[2]) > 0.055:
        return {"status": "failed", "reason": "Object slipped during transport"}

    offset = [pos[i] - eef[i] for i in range(3)]
    destination = basket_target(state, basket)
    placement = [
        destination[0] - offset[0],
        destination[1] - offset[1],
        destination[2] + 0.045 - offset[2],
    ]
    if not policy_move(sdk, placement, grasp_yaw, "close", 45, 0.009):
        return {"status": "failed", "reason": "Could not reach release position"}

    state = sdk.get_state()
    pos = state["objects"][target]["pos"]
    destination = basket_target(state, basket)
    if hypot(pos[0] - destination[0], pos[1] - destination[1]) > 0.055:
        return {"status": "failed", "reason": "Object is not centered over basket"}

    policy_grip(sdk, "open", 15)
    eef = sdk.get_state()["eef_pos"]
    policy_move(
        sdk,
        [eef[0], eef[1], eef[2] + 0.16],
        grasp_yaw,
        "open",
        30,
    )
    policy_grip(sdk, "open", 16)

    state = sdk.get_state()
    pos = state["objects"][target]["pos"]
    destination = basket_target(state, basket)
    basket_pos = state["objects"][basket]["pos"]
    centered = hypot(pos[0] - destination[0], pos[1] - destination[1]) < 0.075
    settled_height = basket_pos[2] + 0.025 < pos[2] < destination[2] + 0.13
    released = state["eef_pos"][2] - pos[2] > 0.09

    if centered and settled_height and released:
        return {"status": "done"}
    return {"status": "failed", "reason": "Placement could not be verified"}
