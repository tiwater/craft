def policy_remaining(sdk):
    state = sdk.get_state()
    return max(0, min(450, state["max_steps"]) - state["steps_used"])


def policy_move(sdk, pos, yaw, grip, steps, tol=0.012):
    budget = min(steps, policy_remaining(sdk))
    if budget > 0:
        return sdk.move_to(pos, yaw=yaw, gripper=grip,
                           max_steps=budget, tol=tol)
    return {"reached": False}


def policy_grip(sdk, action, steps):
    budget = min(steps, policy_remaining(sdk))
    if budget > 0:
        return sdk.set_gripper(action, steps=budget)
    return {"gripper_width": 0.0, "grasped_hint": False}


def policy_resolve(mapping, label):
    if label + "_1" in mapping:
        return label + "_1"
    if label in mapping:
        return label
    matches = []
    for name in mapping:
        if name.startswith(label + "_"):
            matches.append(name)
    if len(matches) == 1:
        return matches[0]
    return None


def policy_destination(state, basket, collection):
    sites = state["sites"]
    for suffix in ["_contain", "_contain_region", "_inside", "_inside_region"]:
        name = basket + suffix
        if name in sites:
            return list(sites[name]), name

    candidates = []
    for name in sites:
        if name.startswith(basket + "_"):
            if "contain" in name or "inside" in name:
                candidates.append(name)
    if len(candidates) == 1:
        name = candidates[0]
        return list(sites[name]), name

    pos = state[collection][basket]["pos"]
    return [pos[0], pos[1], pos[2] + 0.072], "basket_pose_offset"


def run_policy(sdk):
    state = sdk.get_state()
    ketchup = policy_resolve(state["objects"], "ketchup")
    basket = policy_resolve(state["objects"], "basket")
    collection = "objects"
    if basket is None:
        basket = policy_resolve(state["fixtures"], "basket")
        collection = "fixtures"
    if ketchup is None or basket is None:
        return {
            "status": "failed",
            "reason": "Could not uniquely ground ketchup and basket",
            "objects": list(state["objects"]),
            "fixtures": list(state["fixtures"]),
            "sites": list(state["sites"])
        }

    destination, destination_site = policy_destination(state, basket, collection)
    grounding = {
        "object": ketchup,
        "object_initial_pos": list(state["objects"][ketchup]["pos"]),
        "basket": basket,
        "basket_collection": collection,
        "destination_site": destination_site,
        "destination_initial_pos": list(destination)
    }

    grasped = False
    offset = [0.0, 0.0, 0.0]
    carry_yaw = state["eef_yaw"]
    attempts = []

    for attempt in range(3):
        if policy_remaining(sdk) < 185:
            break

        state = sdk.get_state()
        obj = list(state["objects"][ketchup]["pos"])
        destination, destination_site = policy_destination(state, basket, collection)
        yaw = state["objects"][ketchup]["yaw"]
        if attempt == 1:
            yaw += pi / 2.0
        while yaw - state["eef_yaw"] > pi / 2.0:
            yaw -= pi
        while yaw - state["eef_yaw"] < -pi / 2.0:
            yaw += pi

        clearance = max(obj[2] + 0.18, destination[2] + 0.20)
        eef = state["eef_pos"]
        policy_move(sdk, [eef[0], eef[1], clearance],
                    state["eef_yaw"], "open", 25)
        policy_grip(sdk, "open", 6)
        approach = policy_move(
            sdk, [obj[0], obj[1], clearance], yaw, "open", 40
        )
        if not approach["reached"]:
            attempts.append({"stage": "approach", "result": approach})
            continue

        state = sdk.get_state()
        obj = list(state["objects"][ketchup]["pos"])
        height_offset = 0.045
        if attempt == 2:
            height_offset = 0.025
        target = [obj[0], obj[1], obj[2] + height_offset]
        descent = policy_move(sdk, target, yaw, "open", 35, 0.012)

        state = sdk.get_state()
        obj = list(state["objects"][ketchup]["pos"])
        eef = state["eef_pos"]
        horizontal_error = hypot(obj[0] - eef[0], obj[1] - eef[1])
        height = eef[2] - obj[2]
        if horizontal_error > 0.022 or height < 0.005 or height > 0.070:
            attempts.append({
                "stage": "descent",
                "target": target,
                "eef_pos": list(eef),
                "object_pos": obj,
                "result": descent
            })
            continue

        grip = policy_grip(sdk, "close", 14)
        before = sdk.get_state()
        initial_obj = list(before["objects"][ketchup]["pos"])
        eef = before["eef_pos"]
        policy_move(sdk, [eef[0], eef[1], eef[2] + 0.14],
                    yaw, "close", 30)

        after = sdk.get_state()
        lifted_obj = after["objects"][ketchup]["pos"]
        lifted_eef = after["eef_pos"]
        lift = lifted_obj[2] - initial_obj[2]
        attempts.append({
            "stage": "lift",
            "lift_m": lift,
            "gripper_width": after["gripper_width"],
            "grasped_hint": grip["grasped_hint"]
        })
        if (
            lift > 0.065
            and hypot(lifted_obj[0] - lifted_eef[0],
                      lifted_obj[1] - lifted_eef[1]) < 0.075
        ):
            grasped = True
            offset = [lifted_obj[i] - lifted_eef[i] for i in range(3)]
            carry_yaw = after["eef_yaw"]
            break
        policy_grip(sdk, "open", 6)

    if not grasped:
        return {
            "status": "failed",
            "reason": "Could not verify a lifted grasp",
            "grounding": grounding,
            "attempts": attempts,
            "steps_used": sdk.get_state()["steps_used"]
        }

    state = sdk.get_state()
    destination, destination_site = policy_destination(state, basket, collection)
    eef = state["eef_pos"]
    travel_z = max(eef[2], destination[2] + 0.23 - offset[2])
    policy_move(sdk, [eef[0], eef[1], travel_z],
                carry_yaw, "close", 25)
    crossing = policy_move(
        sdk,
        [destination[0] - offset[0], destination[1] - offset[1], travel_z],
        carry_yaw, "close", 55
    )
    if not crossing["reached"]:
        return {"status": "failed", "reason": "Basket approach incomplete",
                "grounding": grounding}

    state = sdk.get_state()
    obj = state["objects"][ketchup]["pos"]
    eef = state["eef_pos"]
    current_offset = [obj[i] - eef[i] for i in range(3)]
    if sqrt(sum((current_offset[i] - offset[i]) ** 2 for i in range(3))) > 0.055:
        return {"status": "failed", "reason": "Object slipped during transport",
                "grounding": grounding}

    destination, destination_site = policy_destination(state, basket, collection)
    placement = [
        destination[0] - current_offset[0],
        destination[1] - current_offset[1],
        destination[2] + 0.045 - current_offset[2]
    ]
    policy_move(sdk, placement, carry_yaw, "close", 35, 0.01)

    state = sdk.get_state()
    obj = state["objects"][ketchup]["pos"]
    destination, destination_site = policy_destination(state, basket, collection)
    if (
        hypot(obj[0] - destination[0], obj[1] - destination[1]) > 0.045
        or obj[2] > destination[2] + 0.10
        or policy_remaining(sdk) < 20
    ):
        return {"status": "failed", "reason": "Placement incomplete",
                "grounding": grounding}

    policy_grip(sdk, "open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(sdk, [eef[0], eef[1], eef[2] + 0.16],
                carry_yaw, "open", 22)
    budget = min(20, policy_remaining(sdk))
    if budget > 0:
        sdk.step_raw([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0], steps=budget)

    state = sdk.get_state()
    obj = state["objects"][ketchup]["pos"]
    destination, destination_site = policy_destination(state, basket, collection)
    yaw = state[collection][basket]["yaw"]
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
        "reason": "Placement verified from live poses" if placed
                  else "Object did not settle inside basket",
        "grounding": grounding,
        "object_final_pos": list(obj),
        "destination_final_pos": list(destination),
        "attempts": attempts,
        "steps_used": state["steps_used"]
    }
