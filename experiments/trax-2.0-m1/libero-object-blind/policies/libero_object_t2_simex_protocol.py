def policy_move(sdk, pos, yaw, gripper, limit=45, tol=0.012):
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


def resolve_entity(entities, base):
    if base + "_1" in entities:
        return base + "_1"
    if base in entities:
        return base
    matches = []
    for name in entities:
        if name.startswith(base + "_"):
            matches.append(name)
    if len(matches) == 1:
        return matches[0]
    return None


def resolve_region(state, basket):
    for name in (
        basket + "_contain_region",
        basket + "_container_region",
        basket + "_inside_region",
    ):
        if name in state["sites"]:
            return name
    matches = []
    for name in state["sites"]:
        if name.startswith(basket + "_") and "contain" in name:
            matches.append(name)
    if len(matches) == 1:
        return matches[0]
    return None


def basket_pose(state, basket):
    if basket in state["objects"]:
        return list(state["objects"][basket]["pos"])
    return list(state["fixtures"][basket]["pos"])


def basket_target(state, basket, region):
    if region is not None:
        return list(state["sites"][region])
    pos = basket_pose(state, basket)
    return [pos[0], pos[1], pos[2] + 0.075]


def nearest_grasp_yaw(yaw, current):
    while yaw - current > pi / 2:
        yaw -= pi
    while yaw - current < -pi / 2:
        yaw += pi
    return yaw


def run_policy(sdk):
    state = sdk.get_state()
    target = resolve_entity(state["objects"], "salad_dressing")
    basket = resolve_entity(state["objects"], "basket")
    if basket is None:
        basket = resolve_entity(state["fixtures"], "basket")
    if target is None or basket is None:
        return {
            "status": "failed",
            "reason": "Required object identities are missing or ambiguous",
        }

    region = resolve_region(state, basket)
    grounding = {
        "object": target,
        "basket": basket,
        "region": region,
        "initial_object_pos": list(state["objects"][target]["pos"]),
        "initial_destination": basket_target(state, basket, region),
    }
    grasped = False
    offset = None

    for attempt in range(3):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 175:
            break

        pos = list(state["objects"][target]["pos"])
        destination = basket_target(state, basket, region)
        grasp_yaw = state["objects"][target]["yaw"]
        if attempt == 1:
            grasp_yaw += pi / 2
        height_offset = 0.045
        if attempt == 2:
            height_offset = 0.035

        clearance = max(
            state["eef_pos"][2], pos[2] + 0.18, destination[2] + 0.17
        )
        policy_grip(sdk, "open", 8)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        raised = policy_move(
            sdk, [eef[0], eef[1], clearance],
            grasp_yaw, "open", 30
        )

        if not raised:
            state = sdk.get_state()
            eef = list(state["eef_pos"])
            pos = list(state["objects"][target]["pos"])
            current_yaw = state["eef_yaw"]
            grasp_yaw = nearest_grasp_yaw(grasp_yaw, current_yaw)
            safe_height = max(pos[2] + 0.15, clearance)
            if eef[2] < pos[2] + 0.12:
                policy_move(
                    sdk, [eef[0], eef[1], safe_height],
                    current_yaw, "open", 20, 0.018
                )
            state = sdk.get_state()
            if state["eef_pos"][2] < pos[2] + 0.10:
                continue
            if not policy_move(
                sdk, [pos[0], pos[1], safe_height],
                grasp_yaw, "open", 55, 0.015
            ):
                continue
        elif not policy_move(
            sdk, [pos[0], pos[1], clearance],
            grasp_yaw, "open", 40
        ):
            continue

        state = sdk.get_state()
        pos = list(state["objects"][target]["pos"])
        approach = [pos[0], pos[1], pos[2] + 0.095]
        if not policy_move(
            sdk, approach, grasp_yaw, "open", 25, 0.008
        ):
            continue

        state = sdk.get_state()
        pos = list(state["objects"][target]["pos"])
        policy_move(
            sdk, [pos[0], pos[1], pos[2] + height_offset],
            grasp_yaw, "open", 30, 0.006
        )

        state = sdk.get_state()
        before = list(state["objects"][target]["pos"])
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - before[0], eef[1] - before[1]) > 0.015
            or abs(eef[2] - before[2] - height_offset) > 0.018
        ):
            continue

        policy_grip(sdk, "close", 18)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.13],
            grasp_yaw, "close", 35, 0.009
        )
        state = sdk.get_state()
        lifted = state["objects"][target]["pos"]
        eef = state["eef_pos"]
        if (
            lifted[2] - before[2] > 0.055
            and hypot(lifted[0] - eef[0], lifted[1] - eef[1]) < 0.065
        ):
            offset = [lifted[i] - eef[i] for i in range(3)]
            grasped = True
            break

    if not grasped:
        return {
            "status": "failed",
            "reason": "Could not verify a lifted grasp",
            "grounding": grounding,
        }

    state = sdk.get_state()
    destination = basket_target(state, basket, region)
    eef = state["eef_pos"]
    travel_height = max(eef[2], destination[2] + 0.20 - offset[2])
    if not policy_move(
        sdk, [eef[0], eef[1], travel_height],
        grasp_yaw, "close", 30
    ):
        return {"status": "failed", "reason": "Could not reach clearance"}

    state = sdk.get_state()
    destination = basket_target(state, basket, region)
    if not policy_move(
        sdk,
        [destination[0] - offset[0], destination[1] - offset[1], travel_height],
        grasp_yaw, "close", 55
    ):
        return {"status": "failed", "reason": "Could not reach above basket"}

    state = sdk.get_state()
    pos = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    current_offset = [pos[i] - eef[i] for i in range(3)]
    if any(abs(current_offset[i] - offset[i]) > 0.045 for i in range(3)):
        return {"status": "failed", "reason": "Object slipped during transport"}

    offset = current_offset
    destination = basket_target(state, basket, region)
    placement = [
        destination[0] - offset[0],
        destination[1] - offset[1],
        destination[2] + 0.045 - offset[2],
    ]
    if not policy_move(sdk, placement, grasp_yaw, "close", 40, 0.012):
        return {"status": "failed", "reason": "Could not reach release position"}

    state = sdk.get_state()
    pos = state["objects"][target]["pos"]
    destination = basket_target(state, basket, region)
    if hypot(pos[0] - destination[0], pos[1] - destination[1]) > 0.055:
        return {"status": "failed", "reason": "Object is not centered"}
    if state["max_steps"] - state["steps_used"] < 40:
        return {"status": "failed", "reason": "Insufficient steps to release safely"}

    policy_grip(sdk, "open", 12)
    eef = sdk.get_state()["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.16],
        grasp_yaw, "open", 25
    )
    policy_grip(sdk, "open", 12)

    state = sdk.get_state()
    pos = state["objects"][target]["pos"]
    destination = basket_target(state, basket, region)
    base = basket_pose(state, basket)
    centered = hypot(pos[0] - destination[0], pos[1] - destination[1]) < 0.075
    settled = base[2] + 0.025 < pos[2] < destination[2] + 0.13
    released = (
        state["gripper_width"] > 0.05
        and state["eef_pos"][2] - pos[2] > 0.09
    )
    if centered and settled and released:
        return {"status": "done", "grounding": grounding}
    return {
        "status": "failed",
        "reason": "Placement could not be verified",
        "grounding": grounding,
    }
