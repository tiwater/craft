def policy_move(sdk, pos, yaw, grip, limit=55, tol=0.012):
    state = sdk.get_state()
    remaining = min(state["max_steps"], 450) - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        pos, yaw=yaw, gripper=grip,
        max_steps=min(limit, remaining), tol=tol
    )
    return result["reached"]


def policy_grip(sdk, action, steps=12):
    state = sdk.get_state()
    remaining = min(state["max_steps"], 450) - state["steps_used"]
    if remaining <= 0:
        return False
    sdk.set_gripper(action, steps=min(steps, remaining))
    return True


def policy_angle(angle, reference):
    while angle - reference > pi / 2:
        angle -= pi
    while angle - reference < -pi / 2:
        angle += pi
    return angle


def run_policy(sdk):
    state = sdk.get_state()
    cheese = None
    basket = None
    for name in state["objects"]:
        if "cream_cheese" in name:
            cheese = name
        if "basket" in name:
            basket = name
    if cheese is None or basket is None:
        return {"status": "failed", "reason": "Required objects unavailable"}

    region = None
    for name in state["sites"]:
        if basket in name and "contain" in name:
            region = name
            break

    held = False
    grasp_yaw = state["eef_yaw"]

    for attempt in range(3):
        state = sdk.get_state()
        remaining = min(state["max_steps"], 450) - state["steps_used"]
        if remaining < 155:
            break

        obj = state["objects"][cheese]
        pos = list(obj["pos"])
        basket_pos = state["objects"][basket]["pos"]
        target = state["sites"][region] if region is not None else basket_pos
        clearance = max(pos[2] + 0.19, target[2] + 0.19)
        for name in state["objects"]:
            clearance = max(
                clearance, state["objects"][name]["pos"][2] + 0.13
            )

        grasp_yaw = policy_angle(
            obj["yaw"] + (pi / 2 if attempt == 1 else 0.0),
            state["eef_yaw"]
        )
        eef = state["eef_pos"]
        if not policy_move(
            sdk, [eef[0], eef[1], max(eef[2], clearance)],
            state["eef_yaw"], "open", 40
        ):
            continue

        policy_grip(sdk, "open", 8)
        if not policy_move(
            sdk, [pos[0], pos[1], clearance], grasp_yaw, "open", 50
        ):
            continue

        state = sdk.get_state()
        pos = list(state["objects"][cheese]["pos"])
        grasp_height = pos[2] + (0.015 if attempt == 2 else 0.025)
        if not policy_move(
            sdk, [pos[0], pos[1], grasp_height],
            grasp_yaw, "open", 45, 0.006
        ):
            continue

        policy_grip(sdk, "close", 15)
        policy_move(
            sdk, [pos[0], pos[1], clearance],
            grasp_yaw, "close", 45
        )
        lifted = sdk.get_state()
        lifted_pos = lifted["objects"][cheese]["pos"]
        eef = lifted["eef_pos"]
        held = (
            lifted_pos[2] - pos[2] > 0.055
            and hypot(lifted_pos[0] - eef[0], lifted_pos[1] - eef[1]) < 0.075
            and abs(lifted_pos[2] - eef[2]) < 0.13
        )
        if held:
            break

    if not held:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    obj_pos = state["objects"][cheese]["pos"]
    eef = state["eef_pos"]
    offset = [obj_pos[i] - eef[i] for i in range(3)]
    target = (
        list(state["sites"][region])
        if region is not None
        else list(state["objects"][basket]["pos"])
    )
    if region is None:
        target[2] += 0.075

    transit_z = max(eef[2], target[2] + 0.18)
    destination = [
        target[0] - offset[0],
        target[1] - offset[1],
        transit_z
    ]
    if not policy_move(sdk, destination, grasp_yaw, "close", 65):
        return {"status": "failed", "reason": "Could not reach basket"}

    state = sdk.get_state()
    current_obj = state["objects"][cheese]["pos"]
    if current_obj[2] < obj_pos[2] - 0.08:
        return {"status": "failed", "reason": "Object slipped during transport"}

    target = (
        list(state["sites"][region])
        if region is not None
        else list(state["objects"][basket]["pos"])
    )
    if region is None:
        target[2] += 0.075
    eef = state["eef_pos"]
    offset = [current_obj[i] - eef[i] for i in range(3)]
    release = [
        target[0] - offset[0],
        target[1] - offset[1],
        target[2] + 0.045 - offset[2]
    ]
    if not policy_move(sdk, release, grasp_yaw, "close", 40, 0.008):
        return {"status": "failed", "reason": "Could not reach release pose"}

    if not policy_grip(sdk, "open", 15):
        return {"status": "failed", "reason": "Step budget exhausted"}

    policy_move(
        sdk, [release[0], release[1], transit_z],
        grasp_yaw, "open", 30
    )
    policy_grip(sdk, "open", 12)

    state = sdk.get_state()
    final_pos = state["objects"][cheese]["pos"]
    basket_pos = state["objects"][basket]["pos"]
    center = state["sites"][region] if region is not None else basket_pos
    inside = (
        hypot(final_pos[0] - center[0], final_pos[1] - center[1]) < 0.065
        and basket_pos[2] - 0.015 < final_pos[2] < center[2] + 0.065
        and state["gripper_width"] > 0.045
    )
    return {
        "status": "done" if inside else "failed",
        "reason": "Placement verified by live pose" if inside else "Placement not verified"
    }
