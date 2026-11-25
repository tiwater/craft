def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def move(position, yaw, grip, steps, tolerance=0.02):
        available = remaining()
        if available <= 0:
            return False
        sdk.move_to(
            position,
            yaw=yaw,
            gripper=grip,
            max_steps=min(steps, available),
            tol=tolerance,
        )
        actual = sdk.get_state()["eef_pos"]
        return sqrt(sum((actual[i] - position[i]) ** 2 for i in range(3))) <= tolerance

    def gripper(action, steps):
        if remaining() > 0:
            sdk.set_gripper(action, steps=min(steps, remaining()))

    def nearest_yaw(target, current):
        while target - current > pi / 2:
            target -= pi
        while target - current < -pi / 2:
            target += pi
        return target

    state = sdk.get_state()
    milk_names = [
        name for name in state["objects"]
        if name == "milk" or name.startswith("milk_")
    ]
    basket_names = [
        name for name in state["objects"]
        if name == "basket" or name.startswith("basket_")
    ]
    basket_fixtures = [
        name for name in state["fixtures"]
        if name == "basket" or name.startswith("basket_")
    ]
    if len(milk_names) != 1:
        return {"status": "failed", "reason": "Milk selection is missing or ambiguous"}
    milk = milk_names[0]
    if len(basket_names) == 1:
        basket = basket_names[0]
        basket_group = "objects"
    elif len(basket_names) == 0 and len(basket_fixtures) == 1:
        basket = basket_fixtures[0]
        basket_group = "fixtures"
    else:
        return {"status": "failed", "reason": "Basket selection is missing or ambiguous"}

    exact_site = basket + "_contain_region"
    target_site = None
    if exact_site in state["sites"]:
        target_site = exact_site
    else:
        candidates = [
            name for name in state["sites"]
            if name.startswith(basket + "_") and "contain" in name
        ]
        if len(candidates) == 1:
            target_site = candidates[0]
        elif len(candidates) > 1:
            return {"status": "failed", "reason": "Basket containment region is ambiguous"}

    def destination(current):
        if target_site is not None:
            return list(current["sites"][target_site])
        position = current[basket_group][basket]["pos"]
        return [position[0], position[1], position[2] + 0.07]

    grounding = {
        "milk": milk,
        "basket": basket,
        "target_site": target_site,
        "initial_milk_position": list(state["objects"][milk]["pos"]),
        "initial_target_position": destination(state),
    }
    grasped = False
    carry_yaw = state["eef_yaw"]

    for attempt in range(2):
        if remaining() < 200:
            break
        state = sdk.get_state()
        position = list(state["objects"][milk]["pos"])
        original_z = position[2]
        yaw = state["objects"][milk]["yaw"] + attempt * pi / 2
        yaw = nearest_yaw(yaw, state["eef_yaw"])
        clearance = max(position[2] + 0.16, destination(state)[2] + 0.17)
        gripper("open", 8)
        eef = sdk.get_state()["eef_pos"]

        if eef[2] < clearance - 0.025:
            if not move([eef[0], eef[1], clearance], state["eef_yaw"], "open", 45, 0.025):
                return {
                    "status": "failed",
                    "reason": "Approach clearance was not reached",
                    "grounding": grounding,
                }

        state = sdk.get_state()
        position = list(state["objects"][milk]["pos"])
        approach_z = max(clearance, state["eef_pos"][2])
        if not move([position[0], position[1], approach_z], yaw, "open", 65, 0.025):
            return {
                "status": "failed",
                "reason": "Milk approach was not reached",
                "grounding": grounding,
            }

        state = sdk.get_state()
        position = list(state["objects"][milk]["pos"])
        grasp_height = position[2] + 0.015 + attempt * 0.015
        if not move([position[0], position[1], grasp_height], yaw, "open", 55, 0.015):
            continue

        gripper("close", 16)
        state = sdk.get_state()
        eef = state["eef_pos"]
        position = state["objects"][milk]["pos"]
        before_offset = [position[i] - eef[i] for i in range(3)]
        move([eef[0], eef[1], eef[2] + 0.14], yaw, "close", 45, 0.02)
        state = sdk.get_state()
        position = state["objects"][milk]["pos"]
        eef = state["eef_pos"]
        after_offset = [position[i] - eef[i] for i in range(3)]
        offset_error = sqrt(sum(
            (after_offset[i] - before_offset[i]) ** 2 for i in range(3)
        ))
        if position[2] - original_z > 0.065 and offset_error < 0.045:
            grasped = True
            carry_yaw = yaw
            break

    if not grasped:
        return {
            "status": "failed",
            "reason": "Could not verify milk grasp",
            "grounding": grounding,
        }

    state = sdk.get_state()
    position = state["objects"][milk]["pos"]
    eef = state["eef_pos"]
    offset = [position[i] - eef[i] for i in range(3)]
    target = destination(state)
    transit_z = max(eef[2], target[2] + 0.20 - offset[2])
    if not move([eef[0], eef[1], transit_z], carry_yaw, "close", 35, 0.025):
        return {"status": "failed", "reason": "Transport clearance was not reached", "grounding": grounding}

    if not move(
        [target[0] - offset[0], target[1] - offset[1], transit_z],
        carry_yaw, "close", 65, 0.02,
    ):
        return {"status": "failed", "reason": "Basket approach was not reached", "grounding": grounding}

    state = sdk.get_state()
    eef = state["eef_pos"]
    position = state["objects"][milk]["pos"]
    current_offset = [position[i] - eef[i] for i in range(3)]
    if sqrt(sum((current_offset[i] - offset[i]) ** 2 for i in range(3))) > 0.05:
        return {"status": "failed", "reason": "Milk slipped during transport", "grounding": grounding}
    if remaining() < 70:
        return {"status": "failed", "reason": "Insufficient placement budget", "grounding": grounding}

    target = destination(state)
    release = [
        target[0] - current_offset[0],
        target[1] - current_offset[1],
        target[2] + 0.045 - current_offset[2],
    ]
    if not move(release, carry_yaw, "close", 40, 0.018):
        return {"status": "failed", "reason": "Placement height was not reached", "grounding": grounding}

    state = sdk.get_state()
    position = state["objects"][milk]["pos"]
    target = destination(state)
    if hypot(position[0] - target[0], position[1] - target[1]) > 0.045:
        return {"status": "failed", "reason": "Milk is not centered over basket", "grounding": grounding}

    gripper("open", 12)
    eef = sdk.get_state()["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.16], carry_yaw, "open", 30, 0.025)
    if remaining() > 0:
        sdk.step_raw(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
            steps=min(18, remaining()),
        )

    state = sdk.get_state()
    position = state["objects"][milk]["pos"]
    target = destination(state)
    horizontal_error = hypot(position[0] - target[0], position[1] - target[1])
    vertical_error = position[2] - target[2]
    released = state["eef_pos"][2] - position[2] > 0.10
    if horizontal_error < 0.075 and -0.06 < vertical_error < 0.10 and released:
        return {"status": "done", "grounding": grounding}
    return {
        "status": "failed",
        "reason": "Final placement could not be verified",
        "grounding": grounding,
        "milk_position": list(position),
        "target_position": target,
    }
