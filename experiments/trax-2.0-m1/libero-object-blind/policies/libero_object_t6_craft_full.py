def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(450, state["max_steps"]) - state["steps_used"])


def move(sdk, position, yaw, action, steps, tolerance=0.012):
    budget = min(steps, remaining_steps(sdk))
    if budget > 0:
        return sdk.move_to(
            position, yaw=yaw, gripper=action,
            max_steps=budget, tol=tolerance
        )
    return {"reached": False}


def grip(sdk, action, steps):
    budget = min(steps, remaining_steps(sdk))
    if budget > 0:
        return sdk.set_gripper(action, steps=budget)
    return {"gripper_width": 0.0, "grasped_hint": False}


def scene_entities(state):
    entities = {}
    for name in state["fixtures"]:
        entities[name] = state["fixtures"][name]
    for name in state["objects"]:
        entities[name] = state["objects"][name]
    return entities


def basket_target(state, basket):
    exact = basket + "_contain_region"
    if exact in state["sites"]:
        return list(state["sites"][exact])
    matches = [
        name for name in state["sites"]
        if name.startswith(basket + "_") and "contain" in name
    ]
    if len(matches) == 1:
        return list(state["sites"][matches[0]])
    position = scene_entities(state)[basket]["pos"]
    return [position[0], position[1], position[2] + 0.075]


def run_policy(sdk):
    state = sdk.get_state()
    language = state["task_language"].lower()
    entities = scene_entities(state)
    butters = [name for name in state["objects"] if "butter" in name.lower()]
    baskets = [name for name in entities if "basket" in name.lower()]

    if "butter" not in language or "basket" not in language:
        return {"status": "failed", "reason": "Task does not identify butter and basket"}
    if len(butters) != 1 or len(baskets) != 1:
        return {
            "status": "failed",
            "reason": "Task referents cannot be uniquely grounded in live entities"
        }

    butter = butters[0]
    basket = baskets[0]
    held = False
    offset = [0.0, 0.0, 0.0]
    grasp_yaw = state["eef_yaw"]

    for attempt in range(2):
        if remaining_steps(sdk) < 255:
            break

        state = sdk.get_state()
        position = list(state["objects"][butter]["pos"])
        object_yaw = state["objects"][butter]["yaw"]
        desired_yaw = object_yaw + (pi / 2.0 if attempt == 0 else 0.0)
        current_yaw = state["eef_yaw"]
        grasp_yaw = current_yaw + 0.5 * atan2(
            sin(2.0 * (desired_yaw - current_yaw)),
            cos(2.0 * (desired_yaw - current_yaw))
        )

        grip(sdk, "open", 16)
        state = sdk.get_state()
        eef = state["eef_pos"]
        approach_height = max(position[2] + 0.115, eef[2])
        if eef[2] < position[2] + 0.075:
            move(
                sdk, [eef[0], eef[1], approach_height],
                grasp_yaw, "open", 16
            )

        move(
            sdk, [position[0], position[1], approach_height],
            grasp_yaw, "open", 30
        )

        state = sdk.get_state()
        position = list(state["objects"][butter]["pos"])
        move(
            sdk, [position[0], position[1], position[2] + 0.035],
            grasp_yaw, "open", 20, 0.009
        )

        state = sdk.get_state()
        position = list(state["objects"][butter]["pos"])
        grasp_height = position[2] - (0.006 if attempt == 0 else 0.010)
        move(
            sdk, [position[0], position[1], grasp_height],
            grasp_yaw, "open", 22, 0.004
        )

        before = list(sdk.get_state()["objects"][butter]["pos"])
        grip(sdk, "close", 20)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move(
            sdk, [eef[0], eef[1], eef[2] + 0.095],
            grasp_yaw, "close", 25, 0.010
        )

        state = sdk.get_state()
        lifted = state["objects"][butter]["pos"]
        eef = state["eef_pos"]
        held = (
            lifted[2] - before[2] > 0.030
            and hypot(lifted[0] - eef[0], lifted[1] - eef[1]) < 0.065
            and state["gripper_width"] > 0.008
        )
        if held:
            offset = [lifted[index] - eef[index] for index in range(3)]
            break

    if not held:
        return {
            "status": "failed",
            "reason": "Lift did not verify a retained butter grasp",
            "steps_used": sdk.get_state()["steps_used"]
        }

    state = sdk.get_state()
    target = basket_target(state, basket)
    entities = scene_entities(state)
    obstacle_height = max(
        [entities[name]["pos"][2] for name in entities]
        + [target[2]]
    )
    clearance = max(
        state["eef_pos"][2],
        obstacle_height + 0.12 - offset[2],
        target[2] + 0.13 - offset[2]
    )
    eef = state["eef_pos"]
    move(
        sdk, [eef[0], eef[1], clearance],
        grasp_yaw, "close", 22
    )
    move(
        sdk,
        [target[0] - offset[0], target[1] - offset[1], clearance],
        grasp_yaw, "close", 55, 0.012
    )

    state = sdk.get_state()
    position = state["objects"][butter]["pos"]
    eef = state["eef_pos"]
    if any(
        abs(position[index] - eef[index] - offset[index]) > 0.045
        for index in range(3)
    ):
        return {"status": "failed", "reason": "Butter slipped during transport"}

    target = basket_target(state, basket)
    move(
        sdk,
        [
            target[0] - offset[0],
            target[1] - offset[1],
            target[2] + 0.025 - offset[2]
        ],
        grasp_yaw, "close", 32, 0.010
    )

    state = sdk.get_state()
    position = state["objects"][butter]["pos"]
    target = basket_target(state, basket)
    if hypot(position[0] - target[0], position[1] - target[1]) > 0.045:
        return {"status": "failed", "reason": "Butter is not aligned over basket"}
    if remaining_steps(sdk) < 18:
        return {"status": "failed", "reason": "Insufficient release budget"}

    grip(sdk, "open", 18)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        sdk, [eef[0], eef[1], eef[2] + 0.12],
        grasp_yaw, "open", 20
    )

    budget = min(12, remaining_steps(sdk))
    if budget > 0:
        sdk.step_raw([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0], steps=budget)

    state = sdk.get_state()
    target = basket_target(state, basket)
    position = state["objects"][butter]["pos"]
    base = scene_entities(state)[basket]["pos"]
    placed = (
        hypot(position[0] - target[0], position[1] - target[1]) < 0.055
        and base[2] - 0.01 <= position[2] <= target[2] + 0.065
        and state["eef_pos"][2] - position[2] > 0.07
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Placement verified from live poses"
        if placed else "Final placement could not be verified",
        "steps_used": state["steps_used"]
    }
