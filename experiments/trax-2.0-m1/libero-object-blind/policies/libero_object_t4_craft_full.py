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
        sdk.set_gripper(action, steps=budget)


def distance(a, b):
    return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def equivalent_yaw(yaw, reference):
    while yaw - reference > pi / 2.0:
        yaw -= pi
    while yaw - reference < -pi / 2.0:
        yaw += pi
    return yaw


def destination(state, basket):
    for name in state["sites"]:
        if basket in name and "contain" in name:
            return list(state["sites"][name])
    if basket in state["objects"]:
        position = state["objects"][basket]["pos"]
    else:
        position = state["fixtures"][basket]["pos"]
    return [position[0], position[1], position[2] + 0.072]


def free_angle(state, target, previous):
    center = state["objects"][target]["pos"]
    neighbors = []
    for group in ["objects", "fixtures"]:
        for name in state[group]:
            if name != target:
                neighbors.append(state[group][name]["pos"])

    best_angle = 0.0
    best_clearance = -1.0
    for index in range(72):
        angle = 2.0 * pi * index / 72.0
        distinct = True
        for used in previous:
            if cos(angle - used) > cos(pi / 3.0):
                distinct = False
        if distinct:
            x = center[0] + 0.04 * cos(angle)
            y = center[1] + 0.04 * sin(angle)
            clearance = 1000.0
            for neighbor in neighbors:
                clearance = min(
                    clearance,
                    hypot(x - neighbor[0], y - neighbor[1])
                )
            if clearance > best_clearance:
                best_clearance = clearance
                best_angle = angle
    return best_angle


def overhead_height(state, target, basket):
    height = max(
        state["objects"][target]["pos"][2] + 0.16,
        destination(state, basket)[2] + 0.16
    )
    for group in ["objects", "fixtures"]:
        for name in state[group]:
            height = max(height, state[group][name]["pos"][2] + 0.16)
    return height


def lift_verified(state, target, baseline_z):
    obj = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    return (
        obj[2] - baseline_z >= 0.03
        and hypot(obj[0] - eef[0], obj[1] - eef[1]) <= 0.08
        and distance(obj, eef) <= 0.12
    )


def run_policy(sdk):
    state = sdk.get_state()
    target = None
    basket = None
    for name in state["objects"]:
        if "ketchup" in name:
            target = name
        if "basket" in name:
            basket = name
    if basket is None:
        for name in state["fixtures"]:
            if "basket" in name:
                basket = name
    if target is None or basket is None:
        return {"status": "failed", "reason": "Required scene entities unavailable"}

    previous = []
    grasped = False
    baseline_z = state["objects"][target]["pos"][2]
    carry_yaw = state["eef_yaw"]
    offset = [0.0, 0.0, 0.0]

    # The ketchup probes had no successful grasp. These are measured
    # candidates, not validated successes; every attempt requires a live lift.
    for attempt in range(3):
        if remaining_steps(sdk) < 215:
            break

        state = sdk.get_state()
        angle = free_angle(state, target, previous)
        previous.append(angle)

        # First two attempts use the measured radial geometry and its
        # perpendicular jaw alignment. The last uses the verified cross-task
        # center geometry, with orientation still selected in world space.
        radius = 0.04
        z_offset = 0.02
        jaw_relative = pi / 2.0
        if attempt == 2:
            radius = 0.0
            z_offset = 0.0
            jaw_relative = 0.0

        yaw = equivalent_yaw(angle + jaw_relative, state["eef_yaw"])
        height = overhead_height(state, target, basket)
        eef = state["eef_pos"]
        move(
            sdk, [eef[0], eef[1], height],
            state["eef_yaw"], "open", 18
        )
        grip(sdk, "open", 4)

        state = sdk.get_state()
        obj = state["objects"][target]["pos"]
        x = obj[0] + radius * cos(angle)
        y = obj[1] + radius * sin(angle)
        move(sdk, [x, y, height], yaw, "open", 25)

        state = sdk.get_state()
        obj = list(state["objects"][target]["pos"])
        baseline_z = obj[2]
        contact = [
            obj[0] + radius * cos(angle),
            obj[1] + radius * sin(angle),
            obj[2] + z_offset
        ]
        move(sdk, contact, yaw, "open", 27, 0.008)

        state = sdk.get_state()
        # Contact can prevent numerical convergence. A nearby stalled EEF
        # must still close; move_to["reached"] is not a grasp prerequisite.
        if distance(state["eef_pos"], contact) > 0.065:
            continue

        baseline_z = state["objects"][target]["pos"][2]
        grip(sdk, "close", 10)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move(
            sdk, [eef[0], eef[1], eef[2] + 0.12],
            yaw, "close", 24
        )
        state = sdk.get_state()
        if lift_verified(state, target, baseline_z):
            grasped = True
            carry_yaw = state["eef_yaw"]
            offset = [
                state["objects"][target]["pos"][i] - state["eef_pos"][i]
                for i in range(3)
            ]
            break
        grip(sdk, "open", 4)

    if not grasped:
        return {
            "status": "failed",
            "reason": "No candidate produced a verified 0.03m lift",
            "steps_used": sdk.get_state()["steps_used"]
        }

    state = sdk.get_state()
    goal = destination(state, basket)
    eef = state["eef_pos"]
    travel_z = max(
        eef[2], overhead_height(state, target, basket) - offset[2],
        goal[2] + 0.18 - offset[2]
    )
    move(sdk, [eef[0], eef[1], travel_z], carry_yaw, "close", 20)

    state = sdk.get_state()
    if not lift_verified(state, target, baseline_z):
        return {"status": "failed", "reason": "Lift lost before transport"}
    goal = destination(state, basket)
    move(
        sdk, [goal[0] - offset[0], goal[1] - offset[1], travel_z],
        carry_yaw, "close", 45
    )

    state = sdk.get_state()
    if not lift_verified(state, target, baseline_z):
        return {"status": "failed", "reason": "Lift lost during transport"}
    obj = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    offset = [obj[i] - eef[i] for i in range(3)]
    goal = destination(state, basket)

    if hypot(obj[0] - goal[0], obj[1] - goal[1]) > 0.035:
        move(
            sdk, [goal[0] - offset[0], goal[1] - offset[1], eef[2]],
            carry_yaw, "close", 12
        )

    state = sdk.get_state()
    obj = state["objects"][target]["pos"]
    goal = destination(state, basket)
    if hypot(obj[0] - goal[0], obj[1] - goal[1]) > 0.045:
        return {"status": "failed", "reason": "Basket alignment incomplete"}

    eef = state["eef_pos"]
    offset = [obj[i] - eef[i] for i in range(3)]
    move(
        sdk,
        [
            goal[0] - offset[0],
            goal[1] - offset[1],
            goal[2] + 0.035 - offset[2]
        ],
        carry_yaw, "close", 28, 0.01
    )
    state = sdk.get_state()
    obj = state["objects"][target]["pos"]
    goal = destination(state, basket)
    if (
        hypot(obj[0] - goal[0], obj[1] - goal[1]) > 0.045
        or obj[2] > goal[2] + 0.10
        or remaining_steps(sdk) < 16
    ):
        return {"status": "failed", "reason": "Release pose incomplete"}

    grip(sdk, "open", 8)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        sdk, [eef[0], eef[1], eef[2] + 0.15],
        carry_yaw, "open", 16
    )
    budget = min(12, remaining_steps(sdk))
    if budget > 0:
        sdk.step_raw(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
            steps=budget
        )

    state = sdk.get_state()
    obj = state["objects"][target]["pos"]
    goal = destination(state, basket)
    placed = (
        hypot(obj[0] - goal[0], obj[1] - goal[1]) < 0.075
        and goal[2] - 0.07 < obj[2] < goal[2] + 0.10
        and distance(obj, state["eef_pos"]) > 0.10
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Placement verified from live poses" if placed
                  else "Released object did not settle inside basket",
        "steps_used": state["steps_used"]
    }
