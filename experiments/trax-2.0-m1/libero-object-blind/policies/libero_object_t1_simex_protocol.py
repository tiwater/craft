def policy_remaining(sdk):
    state = sdk.get_state()
    return max(0, min(450, state["max_steps"]) - state["steps_used"])


def policy_move(sdk, pos, yaw, grip, limit=50, tol=0.01):
    budget = min(limit, policy_remaining(sdk))
    if budget <= 0:
        return False
    sdk.move_to(pos, yaw=yaw, gripper=grip, max_steps=budget, tol=tol)
    actual = sdk.get_state()["eef_pos"]
    return sqrt(sum((actual[i] - pos[i]) ** 2 for i in range(3))) <= max(0.012, 1.5 * tol)


def policy_grip(sdk, action, steps=12):
    budget = min(steps, policy_remaining(sdk))
    if budget <= 0:
        return False
    sdk.set_gripper(action, steps=budget)
    return True


def policy_names(mapping, stem):
    return sorted([
        name for name in mapping
        if name == stem or name.startswith(stem + "_")
    ])


def policy_angle(angle, reference):
    while angle - reference > pi / 2:
        angle -= pi
    while angle - reference < -pi / 2:
        angle += pi
    return angle


def policy_target(state, source, basket, region):
    if region is not None:
        return list(state["sites"][region])
    pos = list(state[source][basket]["pos"])
    return [pos[0], pos[1], pos[2] + 0.075]


def run_policy(sdk):
    state = sdk.get_state()
    cheeses = policy_names(state["objects"], "cream_cheese")
    baskets = policy_names(state["objects"], "basket")
    source = "objects"
    if len(baskets) == 0:
        baskets = policy_names(state["fixtures"], "basket")
        source = "fixtures"
    if len(cheeses) != 1 or len(baskets) != 1:
        return {
            "status": "failed",
            "reason": "Ambiguous object grounding",
            "cheese_candidates": cheeses,
            "basket_candidates": baskets
        }

    cheese = cheeses[0]
    basket = baskets[0]
    region = None
    exact = basket + "_contain_region"
    candidates = [
        name for name in state["sites"]
        if name.startswith(basket + "_") and "contain" in name
    ]
    if exact in state["sites"]:
        region = exact
    elif len(candidates) == 1:
        region = candidates[0]
    elif len(candidates) > 1:
        return {
            "status": "failed",
            "reason": "Ambiguous basket containment site",
            "sites": candidates
        }

    grounding = {
        "object": cheese,
        "object_initial_pos": list(state["objects"][cheese]["pos"]),
        "basket": basket,
        "basket_source": source,
        "region": region,
        "initial_target": policy_target(state, source, basket, region),
        "object_sites": [
            name for name in state["sites"]
            if name.startswith(cheese + "_")
        ]
    }

    attempts = []
    held = False
    grasp_yaw = state["eef_yaw"]
    clearance = 0.0

    for attempt in range(2):
        if policy_remaining(sdk) < 175:
            break

        state = sdk.get_state()
        pos = list(state["objects"][cheese]["pos"])
        target = policy_target(state, source, basket, region)
        clearance = max(pos[2] + 0.17, target[2] + 0.14)
        for name in state["objects"]:
            clearance = max(clearance, state["objects"][name]["pos"][2] + 0.11)

        eef = list(state["eef_pos"])
        if eef[2] < clearance - 0.012:
            if not policy_move(
                sdk, [eef[0], eef[1], clearance],
                state["eef_yaw"], "open", 45, 0.012
            ):
                attempts.append({"attempt": attempt, "reason": "Clearance missed"})
                continue

        policy_grip(sdk, "open", 8)
        state = sdk.get_state()
        pos = list(state["objects"][cheese]["pos"])
        grasp_yaw = policy_angle(
            state["objects"][cheese]["yaw"] + (0.0 if attempt == 0 else pi / 2),
            state["eef_yaw"]
        )

        if not policy_move(
            sdk, [pos[0], pos[1], clearance],
            grasp_yaw, "open", 55, 0.008
        ):
            attempts.append({"attempt": attempt, "reason": "Approach missed"})
            continue

        state = sdk.get_state()
        pos = list(state["objects"][cheese]["pos"])
        if not policy_move(
            sdk, [pos[0], pos[1], pos[2] + 0.070],
            grasp_yaw, "open", 35, 0.008
        ):
            attempts.append({"attempt": attempt, "reason": "Pregrasp missed"})
            continue

        state = sdk.get_state()
        pos = list(state["objects"][cheese]["pos"])
        grasp_height = 0.032 if attempt == 0 else 0.037
        policy_move(
            sdk, [pos[0], pos[1], pos[2] + grasp_height],
            grasp_yaw, "open", 28, 0.005
        )

        state = sdk.get_state()
        baseline = list(state["objects"][cheese]["pos"])
        contact = list(state["eef_pos"])
        if (
            hypot(contact[0] - baseline[0], contact[1] - baseline[1]) > 0.015
            or contact[2] - baseline[2] > 0.052
            or contact[2] - baseline[2] < 0.012
        ):
            attempts.append({
                "attempt": attempt,
                "reason": "Contact pose missed",
                "eef_pos": contact,
                "object_pos": baseline
            })
            continue

        policy_grip(sdk, "close", 18)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.09],
            grasp_yaw, "close", 35, 0.008
        )

        state = sdk.get_state()
        lifted = list(state["objects"][cheese]["pos"])
        eef = list(state["eef_pos"])
        held = (
            lifted[2] - baseline[2] > 0.045
            and hypot(lifted[0] - eef[0], lifted[1] - eef[1]) < 0.055
            and abs(lifted[2] - eef[2]) < 0.10
            and state["gripper_width"] > 0.004
        )
        attempts.append({
            "attempt": attempt,
            "grasp_yaw": grasp_yaw,
            "contact_offset": [contact[i] - baseline[i] for i in range(3)],
            "lift": lifted[2] - baseline[2],
            "gripper_width": state["gripper_width"]
        })
        if held:
            break

    if not held:
        return {
            "status": "failed",
            "reason": "Could not verify lifted cream cheese",
            "grounding": grounding,
            "attempts": attempts
        }

    state = sdk.get_state()
    obj = list(state["objects"][cheese]["pos"])
    eef = list(state["eef_pos"])
    offset = [obj[i] - eef[i] for i in range(3)]
    target = policy_target(state, source, basket, region)
    transit_z = max(clearance, target[2] + 0.14 - offset[2], eef[2])
    if not policy_move(
        sdk, [eef[0], eef[1], transit_z],
        grasp_yaw, "close", 40, 0.012
    ):
        return {"status": "failed", "reason": "Transit clearance missed", "grounding": grounding}

    state = sdk.get_state()
    obj = list(state["objects"][cheese]["pos"])
    eef = list(state["eef_pos"])
    offset = [obj[i] - eef[i] for i in range(3)]
    target = policy_target(state, source, basket, region)
    if not policy_move(
        sdk, [target[0] - offset[0], target[1] - offset[1], transit_z],
        grasp_yaw, "close", min(65, max(0, policy_remaining(sdk) - 65)), 0.008
    ):
        return {"status": "failed", "reason": "Basket approach missed", "grounding": grounding}

    state = sdk.get_state()
    obj = list(state["objects"][cheese]["pos"])
    eef = list(state["eef_pos"])
    target = policy_target(state, source, basket, region)
    if (
        hypot(obj[0] - eef[0], obj[1] - eef[1]) > 0.07
        or abs(obj[2] - eef[2]) > 0.11
    ):
        return {"status": "failed", "reason": "Object slipped", "grounding": grounding}

    offset = [obj[i] - eef[i] for i in range(3)]
    release = [
        target[0] - offset[0],
        target[1] - offset[1],
        target[2] + 0.025 - offset[2]
    ]
    if policy_remaining(sdk) < 35:
        return {"status": "failed", "reason": "Insufficient placement budget"}

    policy_move(
        sdk, release, grasp_yaw, "close",
        min(45, policy_remaining(sdk) - 25), 0.007
    )
    state = sdk.get_state()
    obj = list(state["objects"][cheese]["pos"])
    target = policy_target(state, source, basket, region)
    if (
        hypot(obj[0] - target[0], obj[1] - target[1]) > 0.025
        or obj[2] > target[2] + 0.07
        or obj[2] < target[2] - 0.06
    ):
        return {
            "status": "failed",
            "reason": "Release alignment missed",
            "object_pos": obj,
            "target": target,
            "grounding": grounding
        }

    policy_grip(sdk, "open", 10)
    state = sdk.get_state()
    eef = list(state["eef_pos"])
    policy_move(
        sdk, [eef[0], eef[1], max(transit_z, eef[2] + 0.10)],
        grasp_yaw, "open", min(25, max(0, policy_remaining(sdk) - 8)), 0.015
    )
    policy_grip(sdk, "open", 8)

    state = sdk.get_state()
    final_pos = list(state["objects"][cheese]["pos"])
    target = policy_target(state, source, basket, region)
    basket_pos = state[source][basket]["pos"]
    inside = (
        hypot(final_pos[0] - target[0], final_pos[1] - target[1]) < 0.06
        and basket_pos[2] - 0.015 < final_pos[2] < target[2] + 0.065
        and state["gripper_width"] > 0.045
        and state["eef_pos"][2] - final_pos[2] > 0.075
    )
    return {
        "status": "done" if inside else "failed",
        "reason": "Placement verified by live poses" if inside else "Placement not verified",
        "grounding": grounding,
        "attempts": attempts,
        "final_object_pos": final_pos,
        "final_target": target
    }
