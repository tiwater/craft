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


def policy_remaining(sdk):
    state = sdk.get_state()
    return max(0, state["max_steps"] - state["steps_used"])


def grasp_angle(yaw, current):
    while yaw - current > pi / 2:
        yaw -= pi
    while yaw - current < -pi / 2:
        yaw += pi
    return yaw


def object_offset(state, name):
    return [
        state["eef_pos"][axis] - state["objects"][name]["pos"][axis]
        for axis in range(3)
    ]


def lifted_object(state, name, before, eef_before):
    pos = state["objects"][name]["pos"]
    rise = pos[2] - before[2]
    hand_rise = state["eef_pos"][2] - eef_before[2]
    separation = sqrt(sum(
        (state["eef_pos"][axis] - pos[axis]) ** 2
        for axis in range(3)
    ))
    return (
        rise > 0.045
        and rise > 0.55 * hand_rise
        and separation < 0.17
        and state["gripper_width"] > 0.008
    )


def acquire_sauce(sdk, sauce, basket):
    # Raise the grasp above the body origin to avoid table/palm contact.
    variants = [
        [pi / 2, 0.055],
        [0.0, 0.055],
        [pi / 2, 0.030],
        [0.0, 0.075],
    ]
    for variant in variants:
        if policy_remaining(sdk) < 165:
            break
        state = sdk.get_state()
        pos = list(state["objects"][sauce]["pos"])
        yaw = grasp_angle(
            state["objects"][sauce]["yaw"] + variant[0],
            state["eef_yaw"],
        )
        target = basket_target(state, basket)
        clearance = max(pos[2] + 0.17, target[2] + 0.15)
        hand = state["eef_pos"]

        # Retreat vertically before opening or changing wrist orientation.
        policy_move(
            sdk, [hand[0], hand[1], max(hand[2], clearance)],
            state["eef_yaw"], "close", 25, 0.018,
        )
        policy_grip(sdk, "open", 7)
        policy_move(
            sdk, [pos[0], pos[1], clearance],
            yaw, "open", 35, 0.015,
        )
        state = sdk.get_state()
        pos = list(state["objects"][sauce]["pos"])
        if hypot(
            state["eef_pos"][0] - pos[0],
            state["eef_pos"][1] - pos[1],
        ) > 0.025:
            continue

        grasp = [pos[0], pos[1], pos[2] + variant[1]]
        policy_move(sdk, grasp, yaw, "open", 32, 0.009)
        state = sdk.get_state()
        pos = list(state["objects"][sauce]["pos"])
        hand = state["eef_pos"]
        if (
            hypot(hand[0] - pos[0], hand[1] - pos[1]) > 0.018
            or abs(hand[2] - grasp[2]) > 0.025
        ):
            continue

        policy_grip(sdk, "close", 12)
        state = sdk.get_state()
        before = list(state["objects"][sauce]["pos"])
        eef_before = list(state["eef_pos"])
        policy_move(
            sdk,
            [eef_before[0], eef_before[1], eef_before[2] + 0.105],
            yaw, "close", 28, 0.015,
        )
        state = sdk.get_state()
        if lifted_object(state, sauce, before, eef_before):
            return {"held": True, "yaw": yaw}
    return {"held": False}


def transfer_sauce(sdk, sauce, basket, yaw):
    state = sdk.get_state()
    target = basket_target(state, basket)
    offset = object_offset(state, sauce)
    transit = max(
        state["eef_pos"][2],
        target[2] + 0.16 + offset[2],
    )
    hand = state["eef_pos"]
    policy_move(
        sdk, [hand[0], hand[1], transit],
        yaw, "close", 25, 0.018,
    )
    state = sdk.get_state()
    if state["objects"][sauce]["pos"][2] < target[2] + 0.09:
        return False

    # Judge transfer using the live object pose, not a strict EEF reach flag.
    for attempt in range(2):
        if policy_remaining(sdk) < 65:
            break
        state = sdk.get_state()
        target = basket_target(state, basket)
        offset = object_offset(state, sauce)
        policy_move(
            sdk,
            [target[0] + offset[0], target[1] + offset[1], transit],
            yaw, "close", 38, 0.018,
        )
        state = sdk.get_state()
        pos = state["objects"][sauce]["pos"]
        target = basket_target(state, basket)
        if (
            hypot(pos[0] - target[0], pos[1] - target[1]) < 0.035
            and pos[2] > target[2] + 0.075
        ):
            return True
    return False


def release_sauce(sdk, sauce, basket, yaw):
    state = sdk.get_state()
    target = basket_target(state, basket)
    offset = object_offset(state, sauce)
    policy_move(
        sdk,
        [
            target[0] + offset[0],
            target[1] + offset[1],
            target[2] + 0.07 + offset[2],
        ],
        yaw, "close", min(32, max(0, policy_remaining(sdk) - 35)),
        0.015,
    )
    state = sdk.get_state()
    target = basket_target(state, basket)
    pos = state["objects"][sauce]["pos"]
    if (
        hypot(pos[0] - target[0], pos[1] - target[1]) > 0.045
        or pos[2] > target[2] + 0.13
        or policy_remaining(sdk) < 22
    ):
        return False
    policy_grip(sdk, "open", 10)
    state = sdk.get_state()
    hand = state["eef_pos"]
    policy_move(
        sdk, [hand[0], hand[1], hand[2] + 0.13],
        yaw, "open", min(22, max(0, policy_remaining(sdk) - 12)),
        0.02,
    )
    budget = min(20, policy_remaining(sdk))
    if budget > 0:
        sdk.step_raw([0, 0, 0, 0, 0, 0, -1], steps=budget)
    return True


def run_policy(sdk):
    state = sdk.get_state()
    sauce = None
    basket = None
    for name in state["objects"]:
        if "bbq" in name and "sauce" in name:
            sauce = name
        if "basket" in name:
            basket = name
    if sauce is None or basket is None:
        return {"status": "failed", "reason": "Required objects unavailable"}

    grasp = acquire_sauce(sdk, sauce, basket)
    if not grasp["held"]:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    yaw = grasp["yaw"]
    if not transfer_sauce(sdk, sauce, basket, yaw):
        return {"status": "failed", "reason": "Could not center held sauce above basket"}

    if not release_sauce(sdk, sauce, basket, yaw):
        return {"status": "failed", "reason": "Could not safely release above basket"}

    state = sdk.get_state()
    target = basket_target(state, basket)
    pos = state["objects"][sauce]["pos"]
    base = state["objects"][basket]["pos"]
    centered = hypot(pos[0] - target[0], pos[1] - target[1]) < 0.085
    settled = base[2] + 0.025 < pos[2] < target[2] + 0.09
    if centered and settled:
        return {"status": "done"}
    return {"status": "failed", "reason": "Final placement could not be verified"}
