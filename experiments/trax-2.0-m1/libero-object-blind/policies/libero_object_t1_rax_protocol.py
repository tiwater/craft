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


def policy_target(state, basket, region):
    if region is not None:
        return list(state["sites"][region])
    pos = list(state["objects"][basket]["pos"])
    pos[2] += 0.075
    return pos


def policy_held(state, cheese, baseline):
    pos = state["objects"][cheese]["pos"]
    eef = state["eef_pos"]
    return (
        pos[2] - baseline > 0.045
        and hypot(pos[0] - eef[0], pos[1] - eef[1]) < 0.075
        and abs(pos[2] - eef[2]) < 0.13
    )


def policy_clearance(state, target, cheese):
    clearance = max(state["eef_pos"][2], target[2] + 0.16)
    for name in state["objects"]:
        clearance = max(
            clearance, state["objects"][name]["pos"][2] + 0.12
        )
    return max(clearance, state["objects"][cheese]["pos"][2] + 0.16)


def policy_grasp(sdk, cheese, basket, region, angle, height):
    state = sdk.get_state()
    baseline = state["objects"][cheese]["pos"][2]
    clearance = policy_clearance(
        state, policy_target(state, basket, region), cheese
    )
    eef = state["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], clearance],
        state["eef_yaw"], "open", 25
    )
    policy_grip(sdk, "open", 5)

    state = sdk.get_state()
    obj = state["objects"][cheese]
    yaw = policy_angle(obj["yaw"] + angle, state["eef_yaw"])
    pos = obj["pos"]
    reached = policy_move(
        sdk, [pos[0], pos[1], clearance], yaw, "open", 35, 0.009
    )
    state = sdk.get_state()
    pos = state["objects"][cheese]["pos"]
    eef = state["eef_pos"]
    if not reached and hypot(pos[0] - eef[0], pos[1] - eef[1]) > 0.02:
        return False, yaw, baseline

    policy_move(
        sdk, [pos[0], pos[1], pos[2] + 0.055],
        yaw, "open", 22, 0.006
    )

    # Zero lift on every rollout suggests closure above the thin package.
    # Descend to its live center height before closing; contact can prevent
    # exact convergence, so move_to's reached flag is not a grasp prerequisite.
    state = sdk.get_state()
    pos = list(state["objects"][cheese]["pos"])
    baseline = pos[2]
    policy_move(
        sdk, [pos[0], pos[1], baseline + height],
        yaw, "open", 24, 0.003
    )
    state = sdk.get_state()
    pos = state["objects"][cheese]["pos"]
    eef = state["eef_pos"]
    if hypot(pos[0] - eef[0], pos[1] - eef[1]) > 0.01:
        policy_move(
            sdk, [pos[0], pos[1], pos[2] + height],
            yaw, "open", 10, 0.004
        )

    state = sdk.get_state()
    pos = state["objects"][cheese]["pos"]
    eef = state["eef_pos"]
    if (
        hypot(pos[0] - eef[0], pos[1] - eef[1]) > 0.022
        or eef[2] - pos[2] > 0.035
    ):
        return False, yaw, baseline

    policy_grip(sdk, "close", 16)
    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], max(eef[2] + 0.10, baseline + 0.12)],
        yaw, "close", 25, 0.008
    )
    return policy_held(sdk.get_state(), cheese, baseline), yaw, baseline


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
    baseline = state["objects"][cheese]["pos"][2]

    # Retain the original first orientation while correcting grasp depth.
    # The second attempt tests the orthogonal closing direction.
    configurations = [[pi / 2, 0.005], [0.0, 0.002]]
    for configuration in configurations:
        state = sdk.get_state()
        remaining = min(state["max_steps"], 450) - state["steps_used"]
        if remaining < 245:
            break
        held, grasp_yaw, baseline = policy_grasp(
            sdk, cheese, basket, region, configuration[0], configuration[1]
        )
        if held:
            break

    if not held:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    obj_pos = list(state["objects"][cheese]["pos"])
    eef = state["eef_pos"]
    offset = [obj_pos[i] - eef[i] for i in range(3)]
    target = policy_target(state, basket, region)
    transit_z = max(eef[2], target[2] + 0.18)
    for name in state["objects"]:
        if name != cheese:
            transit_z = max(
                transit_z,
                state["objects"][name]["pos"][2] + 0.13 - offset[2]
            )

    if not policy_move(
        sdk, [eef[0], eef[1], transit_z],
        grasp_yaw, "close", 25
    ):
        return {"status": "failed", "reason": "Could not establish transport clearance"}

    state = sdk.get_state()
    if not policy_held(state, cheese, baseline):
        return {"status": "failed", "reason": "Object slipped during lift"}

    target = policy_target(state, basket, region)
    obj_pos = list(state["objects"][cheese]["pos"])
    eef = state["eef_pos"]
    offset = [obj_pos[i] - eef[i] for i in range(3)]
    destination = [
        target[0] - offset[0],
        target[1] - offset[1],
        transit_z
    ]
    if not policy_move(sdk, destination, grasp_yaw, "close", 60):
        return {"status": "failed", "reason": "Could not reach basket"}

    state = sdk.get_state()
    current_obj = state["objects"][cheese]["pos"]
    eef = state["eef_pos"]
    current_offset = [current_obj[i] - eef[i] for i in range(3)]
    if (
        abs(current_offset[2] - offset[2]) > 0.05
        or hypot(
            current_offset[0] - offset[0],
            current_offset[1] - offset[1]
        ) > 0.045
    ):
        return {"status": "failed", "reason": "Object slipped during transport"}

    target = policy_target(state, basket, region)
    release = [
        target[0] - current_offset[0],
        target[1] - current_offset[1],
        target[2] + 0.045 - current_offset[2]
    ]
    if not policy_move(sdk, release, grasp_yaw, "close", 35, 0.008):
        return {"status": "failed", "reason": "Could not reach release pose"}

    if not policy_grip(sdk, "open", 15):
        return {"status": "failed", "reason": "Step budget exhausted"}

    policy_move(
        sdk, [release[0], release[1], transit_z],
        grasp_yaw, "open", 25
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
