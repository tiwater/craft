def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return min(450, state["max_steps"]) - state["steps_used"]

    def move(pos, yaw, grip, steps, tol=0.012):
        budget = remaining()
        if budget > 0:
            return sdk.move_to(
                pos, yaw=yaw, gripper=grip,
                max_steps=min(steps, budget), tol=tol
            )
        return None

    def grip(action, steps):
        budget = remaining()
        if budget > 0:
            sdk.set_gripper(action, steps=min(steps, budget))

    def destination(state, basket, region):
        if region is not None:
            return list(state["sites"][region])
        for collection in ["objects", "fixtures"]:
            if basket in state[collection]:
                pos = list(state[collection][basket]["pos"])
                return [pos[0], pos[1], pos[2] + 0.072]
        return None

    state = sdk.get_state()
    language = state["task_language"].lower()
    cheeses = []
    baskets = []
    for name in state["objects"]:
        if "cream_cheese" in name:
            cheeses.append(name)
    for collection in ["objects", "fixtures"]:
        for name in state[collection]:
            if "basket" in name and name not in baskets:
                baskets.append(name)

    if not cheeses or not baskets:
        return {"status": "failed", "reason": "Required entities unavailable"}

    # The stated task identifies the cheese by type, with the basket as destination.
    # Do not silently select the last instance if the scene is ambiguous.
    cheese = cheeses[0] if len(cheeses) == 1 else None
    basket = baskets[0] if len(baskets) == 1 else None
    for name in cheeses:
        if name in language:
            cheese = name
    for name in baskets:
        if name in language:
            basket = name
    if cheese is None or basket is None:
        return {"status": "failed", "reason": "Ambiguous task entity grounding"}

    region = None
    region_distance = float("inf")
    basket_pos = None
    for collection in ["objects", "fixtures"]:
        if basket in state[collection]:
            basket_pos = state[collection][basket]["pos"]
    for name in state["sites"]:
        if basket in name and "contain" in name:
            site = state["sites"][name]
            distance = hypot(site[0] - basket_pos[0], site[1] - basket_pos[1])
            if distance < region_distance:
                region = name
                region_distance = distance

    # Empirical winning orientation is world yaw zero, independent of object yaw.
    yaw = 0.0
    while yaw - state["eef_yaw"] > pi:
        yaw -= 2.0 * pi
    while yaw - state["eef_yaw"] < -pi:
        yaw += 2.0 * pi

    held = False
    for attempt in range(2):
        if remaining() < 240:
            break
        state = sdk.get_state()
        pos = list(state["objects"][cheese]["pos"])
        target = destination(state, basket, region)
        clearance = max(pos[2] + 0.17, target[2] + 0.14)
        for collection in ["objects", "fixtures"]:
            for name in state[collection]:
                obstacle = state[collection][name]["pos"]
                clearance = max(clearance, obstacle[2] + 0.12)

        eef = state["eef_pos"]
        grip("open", 8)
        move(
            [eef[0], eef[1], max(eef[2], clearance)],
            yaw, "open", 22
        )
        state = sdk.get_state()
        pos = list(state["objects"][cheese]["pos"])
        move([pos[0], pos[1], clearance], yaw, "open", 32)

        # Use the measured center grasp with zero height offset.
        # Bounded moves may report non-convergence; still execute the grasp.
        state = sdk.get_state()
        pos = list(state["objects"][cheese]["pos"])
        move(pos, yaw, "open", 44, 0.004)
        state = sdk.get_state()
        live_pos = list(state["objects"][cheese]["pos"])
        if hypot(
            live_pos[0] - state["eef_pos"][0],
            live_pos[1] - state["eef_pos"][1]
        ) > 0.012:
            move(live_pos, yaw, "open", 10, 0.005)

        base_z = sdk.get_state()["objects"][cheese]["pos"][2]
        grip("close", 16)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move(
            [eef[0], eef[1], max(base_z + 0.12, eef[2] + 0.10)],
            yaw, "close", 28, 0.012
        )
        state = sdk.get_state()
        lifted = state["objects"][cheese]["pos"]
        eef = state["eef_pos"]
        held = (
            lifted[2] - base_z > 0.03
            and hypot(lifted[0] - eef[0], lifted[1] - eef[1]) < 0.065
            and abs(lifted[2] - eef[2]) < 0.12
        )
        if held:
            break

    if not held:
        return {"status": "failed", "reason": "No measured object lift"}

    state = sdk.get_state()
    target = destination(state, basket, region)
    obj = state["objects"][cheese]["pos"]
    eef = state["eef_pos"]
    offset = [obj[index] - eef[index] for index in range(3)]
    transit_z = max(clearance, target[2] + 0.15 - offset[2])
    move([eef[0], eef[1], transit_z], yaw, "close", 22)
    move(
        [target[0] - offset[0], target[1] - offset[1], transit_z],
        yaw, "close", 52, 0.01
    )

    state = sdk.get_state()
    target = destination(state, basket, region)
    obj = state["objects"][cheese]["pos"]
    eef = state["eef_pos"]
    if obj[2] < base_z + 0.025:
        return {"status": "failed", "reason": "Object slipped during transport"}

    offset = [obj[index] - eef[index] for index in range(3)]
    release = [
        target[0] - offset[0],
        target[1] - offset[1],
        target[2] + 0.025 - offset[2]
    ]
    move(release, yaw, "close", 34, 0.008)

    state = sdk.get_state()
    target = destination(state, basket, region)
    obj = state["objects"][cheese]["pos"]
    if hypot(obj[0] - target[0], obj[1] - target[1]) > 0.035:
        eef = state["eef_pos"]
        correction = [
            eef[0] + target[0] - obj[0],
            eef[1] + target[1] - obj[1],
            eef[2]
        ]
        move(correction, yaw, "close", 16, 0.008)

    state = sdk.get_state()
    obj = state["objects"][cheese]["pos"]
    target = destination(state, basket, region)
    if hypot(obj[0] - target[0], obj[1] - target[1]) > 0.05:
        return {"status": "failed", "reason": "Release not aligned with basket"}

    grip("open", 14)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], max(transit_z, eef[2] + 0.10)], yaw, "open", 22)
    grip("open", 10)

    state = sdk.get_state()
    obj = state["objects"][cheese]["pos"]
    target = destination(state, basket, region)
    for collection in ["objects", "fixtures"]:
        if basket in state[collection]:
            basket_pos = state[collection][basket]["pos"]
    inside = (
        hypot(obj[0] - target[0], obj[1] - target[1]) < 0.065
        and basket_pos[2] - 0.015 < obj[2] < target[2] + 0.065
        and state["gripper_width"] > 0.045
    )
    return {
        "status": "done" if inside else "failed",
        "reason": "Placement verified by live poses" if inside else "Placement not verified"
    }
