def policy_remaining(sdk):
    state = sdk.get_state()
    return max(0, min(450, state["max_steps"]) - state["steps_used"])


def policy_move(sdk, pos, yaw, grip, steps, tol=0.012, reserve=0):
    budget = min(steps, max(0, policy_remaining(sdk) - reserve))
    if budget > 0:
        return sdk.move_to(
            pos, yaw=yaw, gripper=grip, max_steps=budget, tol=tol
        )
    return {"reached": False}


def policy_grip(sdk, action, steps=12, reserve=0):
    budget = min(steps, max(0, policy_remaining(sdk) - reserve))
    if budget > 0:
        return sdk.set_gripper(action, steps=budget)
    state = sdk.get_state()
    return {
        "gripper_width": state["gripper_width"],
        "grasped_hint": False,
    }


def basket_target(state, basket):
    sites = state["sites"]
    preferred = basket + "_contain_region"
    if preferred in sites:
        return list(sites[preferred])
    for name in sites:
        if basket in name and "contain" in name:
            return list(sites[name])
    pos = state["objects"][basket]["pos"]
    return [pos[0], pos[1], pos[2] + 0.075]


def policy_grasp_yaw(yaw, reference):
    # Parallel jaws admit equivalent orientations separated by pi.
    while yaw - reference > pi / 2.0:
        yaw -= pi
    while yaw - reference < -pi / 2.0:
        yaw += pi
    return yaw


def policy_grasp(sdk, tomato, basket):
    reserve = 140
    for attempt in range(3):
        if policy_remaining(sdk) < reserve + 90:
            break

        state = sdk.get_state()
        obj = list(state["objects"][tomato]["pos"])
        center = basket_target(state, basket)
        yaw = state["objects"][tomato]["yaw"]
        if attempt == 1:
            yaw += pi / 2.0
        yaw = policy_grasp_yaw(yaw, state["eef_yaw"])
        eef = state["eef_pos"]
        clearance = max(eef[2], obj[2] + 0.16, center[2] + 0.19)

        policy_grip(sdk, "open", 8, reserve)
        if eef[2] < clearance - 0.02:
            policy_move(
                sdk, [eef[0], eef[1], clearance],
                yaw, "open", 25, reserve=reserve
            )

        approach = policy_move(
            sdk, [obj[0], obj[1], clearance],
            yaw, "open", 40, reserve=reserve
        )
        if not approach["reached"]:
            continue

        state = sdk.get_state()
        obj = list(state["objects"][tomato]["pos"])
        height = obj[2] + (0.008 if attempt == 0 else 0.025)
        policy_move(
            sdk, [obj[0], obj[1], height],
            yaw, "open", 38, 0.008, reserve
        )

        # Contact can stop descent before its numerical target is reached.
        # Inspect the live grasp geometry instead of discarding that attempt.
        state = sdk.get_state()
        obj = list(state["objects"][tomato]["pos"])
        eef = state["eef_pos"]
        lateral_error = hypot(obj[0] - eef[0], obj[1] - eef[1])
        if lateral_error > 0.018 and lateral_error < 0.045:
            policy_move(
                sdk, [obj[0], obj[1], max(eef[2], obj[2] + 0.025)],
                yaw, "open", 12, 0.008, reserve
            )
            state = sdk.get_state()
            obj = list(state["objects"][tomato]["pos"])
            eef = state["eef_pos"]
            lateral_error = hypot(obj[0] - eef[0], obj[1] - eef[1])

        if (
            lateral_error > 0.025
            or eef[2] - obj[2] > 0.065
            or eef[2] - obj[2] < -0.015
        ):
            continue
        if policy_remaining(sdk) < reserve + 49:
            break

        before = list(obj)
        policy_grip(sdk, "close", 14, reserve)
        state = sdk.get_state()
        eef = state["eef_pos"]
        center = basket_target(state, basket)
        policy_move(
            sdk,
            [eef[0], eef[1], max(eef[2] + 0.12, center[2] + 0.16)],
            yaw, "close", 35, reserve=reserve
        )

        state = sdk.get_state()
        lifted = state["objects"][tomato]["pos"]
        eef = state["eef_pos"]
        if (
            lifted[2] - before[2] > 0.045
            and hypot(lifted[0] - eef[0], lifted[1] - eef[1]) < 0.065
            and state["gripper_width"] > 0.012
        ):
            return {
                "held": True,
                "yaw": yaw,
                "offset": [lifted[i] - eef[i] for i in range(3)],
            }
        policy_grip(sdk, "open", 8, reserve)

    return {"held": False}


def policy_place(sdk, tomato, basket, grasp):
    yaw = grasp["yaw"]
    offset = grasp["offset"]
    state = sdk.get_state()
    center = basket_target(state, basket)
    eef = state["eef_pos"]
    transit_z = max(eef[2], center[2] + 0.18 - offset[2])

    if transit_z - eef[2] > 0.012:
        result = policy_move(
            sdk, [eef[0], eef[1], transit_z],
            yaw, "close", 25, reserve=100
        )
        if not result["reached"]:
            return {"status": "failed", "reason": "Could not reach safe transfer height"}

    result = policy_move(
        sdk,
        [center[0] - offset[0], center[1] - offset[1], transit_z],
        yaw, "close", 60, reserve=65
    )
    if not result["reached"]:
        return {"status": "failed", "reason": "Could not reach basket"}

    state = sdk.get_state()
    obj = state["objects"][tomato]["pos"]
    eef = state["eef_pos"]
    measured_offset = [obj[i] - eef[i] for i in range(3)]
    if (
        abs(measured_offset[2] - offset[2]) > 0.045
        or hypot(
            measured_offset[0] - offset[0],
            measured_offset[1] - offset[1],
        ) > 0.045
    ):
        return {"status": "failed", "reason": "Object slipped during transfer"}

    center = basket_target(state, basket)
    release = [
        center[0] - measured_offset[0],
        center[1] - measured_offset[1],
        center[2] + 0.035 - measured_offset[2],
    ]
    result = policy_move(
        sdk, release, yaw, "close", 35, 0.008, reserve=30
    )
    if not result["reached"]:
        state = sdk.get_state()
        obj = state["objects"][tomato]["pos"]
        center = basket_target(state, basket)
        if (
            hypot(obj[0] - center[0], obj[1] - center[1]) > 0.035
            or obj[2] < center[2] - 0.02
            or obj[2] > center[2] + 0.065
        ):
            return {"status": "failed", "reason": "Could not reach release pose"}

    policy_grip(sdk, "open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.15],
        yaw, "open", 25, reserve=8
    )
    policy_grip(sdk, "open", 15)

    state = sdk.get_state()
    obj = state["objects"][tomato]["pos"]
    center = basket_target(state, basket)
    basket_yaw = state["objects"][basket]["yaw"]
    dx = obj[0] - center[0]
    dy = obj[1] - center[1]
    local_x = cos(basket_yaw) * dx + sin(basket_yaw) * dy
    local_y = -sin(basket_yaw) * dx + cos(basket_yaw) * dy
    placed = (
        abs(local_x) < 0.075
        and abs(local_y) < 0.075
        and center[2] - 0.065 < obj[2] < center[2] + 0.07
        and state["gripper_width"] > 0.04
    )
    return {
        "status": "done" if placed else "failed",
        "reason": (
            "Placement verified geometrically"
            if placed else "Placement could not be verified"
        ),
        "steps_used": state["steps_used"],
    }


def run_policy(sdk):
    state = sdk.get_state()
    tomato = None
    basket = None
    for name in sorted(state["objects"]):
        if "tomato_sauce" in name:
            tomato = name
        if "basket" in name:
            basket = name
    if tomato is None or basket is None:
        return {"status": "failed", "reason": "Required objects unavailable"}

    grasp = policy_grasp(sdk, tomato, basket)
    if not grasp["held"]:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}
    return policy_place(sdk, tomato, basket, grasp)
