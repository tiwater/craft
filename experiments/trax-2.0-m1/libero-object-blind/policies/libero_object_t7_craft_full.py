def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(450, state["max_steps"]) - state["steps_used"])


def move(sdk, position, yaw, action, steps, tolerance=0.015):
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


def distance(a, b):
    return sqrt(sum((a[index] - b[index]) ** 2 for index in range(3)))


def resolve_instance(state, category):
    candidates = []
    for group in ("objects", "fixtures"):
        for name in state[group]:
            if name == category or name.startswith(category + "_"):
                if name not in candidates:
                    candidates.append(name)
    if len(candidates) == 1:
        return candidates[0]

    language = state["task_language"].lower()
    explicit = []
    for name in candidates:
        if name.lower() in language:
            explicit.append(name)
    if len(explicit) == 1:
        return explicit[0]
    return None


def entity_position(state, name):
    if name in state["objects"]:
        return list(state["objects"][name]["pos"])
    return list(state["fixtures"][name]["pos"])


def destination(state, basket):
    key = basket + "_contain_region"
    if key in state["sites"]:
        return list(state["sites"][key])
    return None


def clearance_height(state, milk, basket):
    start = entity_position(state, milk)
    target = destination(state, basket)
    height = max(start[2] + 0.18, target[2] + 0.17)
    dx = target[0] - start[0]
    dy = target[1] - start[1]
    length_squared = max(dx * dx + dy * dy, 0.000001)
    for group in ("objects", "fixtures"):
        for name in state[group]:
            if name != milk and name != basket:
                point = state[group][name]["pos"]
                fraction = max(
                    0.0, min(
                        1.0,
                        ((point[0] - start[0]) * dx
                         + (point[1] - start[1]) * dy) / length_squared
                    )
                )
                separation = hypot(
                    point[0] - start[0] - fraction * dx,
                    point[1] - start[1] - fraction * dy
                )
                if separation < 0.12:
                    height = max(height, point[2] + 0.16)
    return height


def run_policy(sdk):
    state = sdk.get_state()
    milk = resolve_instance(state, "milk")
    basket = resolve_instance(state, "basket")
    if milk is None or basket is None or milk not in state["objects"]:
        return {"status": "failed", "reason": "Missing or ambiguous task instances"}
    if destination(state, basket) is None:
        return {"status": "failed", "reason": "Basket containment site missing"}

    # The task's spatial relation is placement inside this basket's own region.
    # Use the successful measured world yaw, independently of the milk's yaw.
    yaw = 0.0
    grasped = False

    for attempt in range(2):
        if remaining_steps(sdk) < 275:
            break

        state = sdk.get_state()
        height = clearance_height(state, milk, basket)
        grip(sdk, "open", 6)
        eef = sdk.get_state()["eef_pos"]
        move(sdk, [eef[0], eef[1], height], yaw, "open", 20, 0.025)

        state = sdk.get_state()
        point = entity_position(state, milk)
        move(
            sdk, [point[0], point[1], height],
            yaw, "open", 38, 0.018
        )

        state = sdk.get_state()
        point = entity_position(state, milk)
        reference_z = point[2]
        move(sdk, point, yaw, "open", 35, 0.012)

        # Contact can prevent exact convergence. Never gate closing on reached.
        state = sdk.get_state()
        point = entity_position(state, milk)
        eef = state["eef_pos"]
        if hypot(point[0] - eef[0], point[1] - eef[1]) > 0.02:
            move(sdk, point, yaw, "open", 8, 0.015)

        grip(sdk, "close", 12)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        point_before = entity_position(state, milk)
        move(
            sdk, [eef[0], eef[1], eef[2] + 0.11],
            yaw, "close", 26, 0.015
        )

        state = sdk.get_state()
        point = entity_position(state, milk)
        rise = point[2] - reference_z
        coupled_rise = point[2] - point_before[2]
        if (
            rise >= 0.025
            and coupled_rise >= 0.02
            and state["gripper_width"] > 0.015
            and distance(point, state["eef_pos"]) < 0.13
        ):
            grasped = True
            break

        if attempt == 0 and remaining_steps(sdk) >= 295:
            if rise > 0.015:
                eef = state["eef_pos"]
                move(
                    sdk, [eef[0], eef[1], eef[2] - rise],
                    yaw, "close", 18, 0.02
                )
            grip(sdk, "open", 6)

    if not grasped:
        return {"status": "failed", "reason": "Milk did not follow the grasp lift"}

    state = sdk.get_state()
    point = entity_position(state, milk)
    eef = state["eef_pos"]
    offset = [point[index] - eef[index] for index in range(3)]
    target = destination(state, basket)
    height = max(
        clearance_height(state, milk, basket),
        target[2] + 0.16 - offset[2]
    )
    move(sdk, [eef[0], eef[1], height], yaw, "close", 24, 0.02)

    state = sdk.get_state()
    target = destination(state, basket)
    move(
        sdk, [target[0] - offset[0], target[1] - offset[1], height],
        yaw, "close", 58, 0.015
    )

    state = sdk.get_state()
    point = entity_position(state, milk)
    eef = state["eef_pos"]
    target = destination(state, basket)
    if distance(point, eef) > 0.14 or point[2] < reference_z + 0.02:
        return {"status": "failed", "reason": "Milk lost during transport"}

    offset = [point[index] - eef[index] for index in range(3)]
    if hypot(point[0] - target[0], point[1] - target[1]) > 0.025:
        move(
            sdk,
            [target[0] - offset[0], target[1] - offset[1], height],
            yaw, "close", 16, 0.012
        )

    state = sdk.get_state()
    point = entity_position(state, milk)
    eef = state["eef_pos"]
    target = destination(state, basket)
    offset = [point[index] - eef[index] for index in range(3)]
    move(
        sdk,
        [
            target[0] - offset[0],
            target[1] - offset[1],
            target[2] + 0.025 - offset[2]
        ],
        yaw, "close", 30, 0.015
    )

    state = sdk.get_state()
    point = entity_position(state, milk)
    target = destination(state, basket)
    if hypot(point[0] - target[0], point[1] - target[1]) > 0.055:
        return {"status": "failed", "reason": "Milk outside basket release footprint"}

    grip(sdk, "open", 10)
    eef = sdk.get_state()["eef_pos"]
    move(
        sdk, [eef[0], eef[1], eef[2] + 0.15],
        yaw, "open", 22, 0.02
    )
    budget = min(12, remaining_steps(sdk))
    if budget > 0:
        sdk.step_raw([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0], steps=budget)

    state = sdk.get_state()
    point = entity_position(state, milk)
    target = destination(state, basket)
    if (
        hypot(point[0] - target[0], point[1] - target[1]) < 0.065
        and -0.06 < point[2] - target[2] < 0.10
        and state["eef_pos"][2] - point[2] > 0.10
        and state["gripper_width"] > 0.04
    ):
        return {
            "status": "done",
            "verification": "Live milk pose inside basket region after release"
        }
    return {"status": "failed", "reason": "Final placement not verified"}
