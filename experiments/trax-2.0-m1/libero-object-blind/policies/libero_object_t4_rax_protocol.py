def policy_remaining(sdk):
    state = sdk.get_state()
    return max(0, min(450, state["max_steps"]) - state["steps_used"])


def policy_move(sdk, pos, yaw, grip, steps, tol=0.012):
    budget = min(steps, policy_remaining(sdk))
    if budget > 0:
        return sdk.move_to(
            pos, yaw=yaw, gripper=grip, max_steps=budget, tol=tol
        )
    return {"reached": False}


def policy_grip(sdk, action, steps=12):
    budget = min(steps, policy_remaining(sdk))
    if budget > 0:
        return sdk.set_gripper(action, steps=budget)
    state = sdk.get_state()
    return {
        "gripper_width": state["gripper_width"],
        "grasped_hint": False,
    }


def policy_destination(state, basket):
    name = basket + "_contain_region"
    if name in state["sites"]:
        return list(state["sites"][name])
    for name in state["sites"]:
        if basket in name and "contain" in name:
            return list(state["sites"][name])
    pos = state["objects"][basket]["pos"]
    return [pos[0], pos[1], pos[2] + 0.072]


def policy_yaw(yaw, reference):
    while yaw - reference > pi / 2.0:
        yaw -= pi
    while yaw - reference < -pi / 2.0:
        yaw += pi
    return yaw


def policy_offset(state, ketchup):
    obj = state["objects"][ketchup]["pos"]
    eef = state["eef_pos"]
    return [obj[i] - eef[i] for i in range(3)]


def run_policy(sdk):
    state = sdk.get_state()
    ketchup = None
    basket = None
    for name in state["objects"]:
        if "ketchup" in name:
            ketchup = name
        if "basket" in name:
            basket = name
    if ketchup is None or basket is None:
        return {"status": "failed", "reason": "Required objects unavailable"}

    grasped = False
    offset = [0.0, 0.0, 0.0]
    carry_yaw = state["eef_yaw"]

    # The failed rollouts barely lifted the bottle and often stopped above
    # the requested grasp height. Try a higher grasp before deeper contact,
    # and test the orthogonal jaw orientation first.
    heights = [0.055, 0.045, 0.035]
    rotations = [pi / 2.0, 0.0, pi / 2.0]

    for attempt in range(3):
        if policy_remaining(sdk) < 180:
            break

        state = sdk.get_state()
        obj = list(state["objects"][ketchup]["pos"])
        destination = policy_destination(state, basket)
        yaw = policy_yaw(
            state["objects"][ketchup]["yaw"] + rotations[attempt],
            state["eef_yaw"],
        )
        clearance = max(obj[2] + 0.18, destination[2] + 0.20)
        eef = state["eef_pos"]

        policy_move(
            sdk, [eef[0], eef[1], clearance],
            state["eef_yaw"], "open", 25
        )
        policy_grip(sdk, "open", 6)
        policy_move(
            sdk, [obj[0], obj[1], clearance], yaw, "open", 35
        )

        state = sdk.get_state()
        obj = list(state["objects"][ketchup]["pos"])
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - obj[0], eef[1] - obj[1]) > 0.025
            or eef[2] < obj[2] + 0.10
        ):
            continue

        target = [obj[0], obj[1], obj[2] + heights[attempt]]
        policy_move(sdk, target, yaw, "open", 30, 0.008)

        state = sdk.get_state()
        obj = list(state["objects"][ketchup]["pos"])
        eef = state["eef_pos"]

        # A contact-limited descent need not report reached. Close only
        # when the measured pose is still compatible with the grasp.
        if (
            hypot(eef[0] - obj[0], eef[1] - obj[1]) > 0.022
            or eef[2] - obj[2] < 0.015
            or eef[2] - obj[2] > 0.085
        ):
            continue

        initial_obj = list(obj)
        policy_grip(sdk, "close", 16)
        before = sdk.get_state()
        eef = before["eef_pos"]
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.12],
            yaw, "close", 28
        )
        after = sdk.get_state()
        lifted_obj = after["objects"][ketchup]["pos"]
        lifted_eef = after["eef_pos"]

        if (
            lifted_obj[2] - initial_obj[2] > 0.055
            and hypot(
                lifted_obj[0] - lifted_eef[0],
                lifted_obj[1] - lifted_eef[1],
            ) < 0.075
            and after["gripper_width"] > 0.012
        ):
            grasped = True
            offset = policy_offset(after, ketchup)
            carry_yaw = after["eef_yaw"]
            break

        policy_grip(sdk, "open", 6)

    if not grasped:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    destination = policy_destination(state, basket)
    eef = state["eef_pos"]
    travel_z = max(eef[2], destination[2] + 0.23 - offset[2])
    rise = policy_move(
        sdk, [eef[0], eef[1], travel_z],
        carry_yaw, "close", 25
    )
    if not rise["reached"]:
        return {"status": "failed", "reason": "Transport clearance incomplete"}

    crossing = policy_move(
        sdk,
        [destination[0] - offset[0], destination[1] - offset[1], travel_z],
        carry_yaw, "close", 55
    )
    if not crossing["reached"]:
        return {"status": "failed", "reason": "Basket approach incomplete"}

    state = sdk.get_state()
    current_offset = policy_offset(state, ketchup)
    if sqrt(sum(
        (current_offset[i] - offset[i]) ** 2 for i in range(3)
    )) > 0.055:
        return {"status": "failed", "reason": "Object slipped during transport"}

    destination = policy_destination(state, basket)
    placement = [
        destination[0] - current_offset[0],
        destination[1] - current_offset[1],
        destination[2] + 0.045 - current_offset[2],
    ]
    policy_move(sdk, placement, carry_yaw, "close", 35, 0.01)

    state = sdk.get_state()
    obj = state["objects"][ketchup]["pos"]
    destination = policy_destination(state, basket)
    if (
        hypot(obj[0] - destination[0], obj[1] - destination[1]) > 0.045
        or obj[2] > destination[2] + 0.10
    ):
        return {"status": "failed", "reason": "Placement alignment incomplete"}

    if policy_remaining(sdk) < 15:
        return {"status": "failed", "reason": "Insufficient budget for release"}

    policy_grip(sdk, "open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.16],
        carry_yaw, "open", 22
    )
    budget = min(20, policy_remaining(sdk))
    if budget > 0:
        sdk.step_raw(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
            steps=budget,
        )

    state = sdk.get_state()
    obj = state["objects"][ketchup]["pos"]
    destination = policy_destination(state, basket)
    yaw = state["objects"][basket]["yaw"]
    dx = obj[0] - destination[0]
    dy = obj[1] - destination[1]
    local_x = cos(yaw) * dx + sin(yaw) * dy
    local_y = -sin(yaw) * dx + cos(yaw) * dy
    placed = (
        abs(local_x) < 0.085
        and abs(local_y) < 0.085
        and destination[2] - 0.07 < obj[2] < destination[2] + 0.10
    )
    return {
        "status": "done" if placed else "failed",
        "reason": (
            "Placement verified from live poses" if placed
            else "Object did not settle inside basket"
        ),
        "steps_used": state["steps_used"],
    }
