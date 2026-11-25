def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, grip, steps, tol=0.015):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.move_to(pos, yaw=yaw, gripper=grip,
                        max_steps=budget, tol=tol)
        return sdk.get_state()

    def grip(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def destination(state, basket, source):
        center = state[source][basket]["pos"]
        candidates = []
        for name in state["sites"]:
            if basket in name and ("contain" in name or "inside" in name):
                candidates.append(name)
        if len(candidates) > 0:
            return list(state["sites"][sorted(candidates)[0]])
        return [center[0], center[1], center[2] + 0.07]

    def nearest_yaw(target, current):
        while target - current > pi / 2.0:
            target -= pi
        while target - current < -pi / 2.0:
            target += pi
        return target

    state = sdk.get_state()
    ketchup = None
    basket = None
    basket_source = "objects"
    for name in state["objects"]:
        if "ketchup" in name:
            ketchup = name
        if "basket" in name:
            basket = name
    if basket is None:
        for name in state["fixtures"]:
            if "basket" in name:
                basket = name
                basket_source = "fixtures"
    if ketchup is None or basket is None:
        return {"status": "failed", "reason": "Required scene entities unavailable"}

    held = False
    offset = [0.0, 0.0, 0.0]
    carry_yaw = state["eef_yaw"]
    heights = [0.055, 0.025, 0.075]

    for attempt in range(3):
        if remaining() < 155:
            break
        state = sdk.get_state()
        obj = list(state["objects"][ketchup]["pos"])
        yaw = nearest_yaw(
            state["objects"][ketchup]["yaw"] +
            (pi / 2.0 if attempt != 1 else 0.0),
            state["eef_yaw"]
        )
        eef = state["eef_pos"]
        clearance = max(eef[2], obj[2] + 0.17)

        # Retreat before opening so a failed low approach cannot sweep objects.
        move([eef[0], eef[1], clearance],
             state["eef_yaw"], "close" if attempt > 0 else "open", 20)
        grip("open", 6)
        move([obj[0], obj[1], clearance], yaw, "open", 32)

        state = sdk.get_state()
        obj = list(state["objects"][ketchup]["pos"])
        eef = state["eef_pos"]
        if hypot(eef[0] - obj[0], eef[1] - obj[1]) > 0.035:
            continue

        target = [obj[0], obj[1], obj[2] + heights[attempt]]
        move([target[0], target[1], target[2] + 0.045],
             yaw, "open", 18)
        state = move(target, yaw, "open", 25, 0.012)

        # Contact may prevent move_to from reporting reached. Judge the
        # physical approach instead of discarding a usable grasp pose.
        obj = list(state["objects"][ketchup]["pos"])
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - obj[0], eef[1] - obj[1]) > 0.03
            or abs(eef[2] - target[2]) > 0.045
        ):
            continue

        baseline = obj[2]
        grip("close", 14)
        state = sdk.get_state()
        eef = state["eef_pos"]
        state = move([eef[0], eef[1], eef[2] + 0.11],
                     yaw, "close", 25)
        obj = state["objects"][ketchup]["pos"]
        eef = state["eef_pos"]
        if (
            obj[2] > baseline + 0.045
            and hypot(obj[0] - eef[0], obj[1] - eef[1]) < 0.085
        ):
            first_offset = [obj[i] - eef[i] for i in range(3)]
            first_height = obj[2]
            state = move([eef[0], eef[1], eef[2] + 0.045],
                         state["eef_yaw"], "close", 12)
            obj = state["objects"][ketchup]["pos"]
            eef = state["eef_pos"]
            offset = [obj[i] - eef[i] for i in range(3)]
            if (
                obj[2] > first_height + 0.015
                and sqrt(sum(
                    (offset[i] - first_offset[i]) ** 2
                    for i in range(3)
                )) < 0.035
            ):
                held = True
                carry_yaw = state["eef_yaw"]
                break

    if not held:
        return {
            "status": "failed",
            "reason": "No verified object-following grasp",
            "steps_used": sdk.get_state()["steps_used"]
        }

    state = sdk.get_state()
    dest = destination(state, basket, basket_source)
    eef = state["eef_pos"]
    travel_z = max(eef[2], dest[2] + 0.19 - offset[2])
    move([eef[0], eef[1], travel_z], carry_yaw, "close", 20)
    state = move(
        [dest[0] - offset[0], dest[1] - offset[1], travel_z],
        carry_yaw, "close", 45
    )

    obj = state["objects"][ketchup]["pos"]
    eef = state["eef_pos"]
    live_offset = [obj[i] - eef[i] for i in range(3)]
    if sqrt(sum(
        (live_offset[i] - offset[i]) ** 2 for i in range(3)
    )) > 0.05:
        return {"status": "failed", "reason": "Grasp lost during transfer"}

    for correction in range(2):
        if remaining() < 30:
            break
        state = sdk.get_state()
        dest = destination(state, basket, basket_source)
        obj = state["objects"][ketchup]["pos"]
        eef = state["eef_pos"]
        error_x = dest[0] - obj[0]
        error_y = dest[1] - obj[1]
        if hypot(error_x, error_y) < 0.018:
            break
        move([eef[0] + error_x, eef[1] + error_y, eef[2]],
             carry_yaw, "close", 15, 0.01)

    state = sdk.get_state()
    dest = destination(state, basket, basket_source)
    obj = state["objects"][ketchup]["pos"]
    if hypot(obj[0] - dest[0], obj[1] - dest[1]) > 0.04:
        return {"status": "failed", "reason": "Basket alignment unsuccessful"}

    eef = state["eef_pos"]
    release_height = dest[2] + 0.065
    state = move(
        [eef[0] + dest[0] - obj[0],
         eef[1] + dest[1] - obj[1],
         eef[2] + release_height - obj[2]],
        carry_yaw, "close", min(25, max(0, remaining() - 28)), 0.012
    )
    obj = state["objects"][ketchup]["pos"]
    dest = destination(state, basket, basket_source)
    if hypot(obj[0] - dest[0], obj[1] - dest[1]) > 0.045:
        return {"status": "failed", "reason": "Object displaced during lowering"}

    grip("open", 10)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.14],
         carry_yaw, "open", min(16, max(0, remaining() - 8)))
    budget = min(12, remaining())
    if budget > 0:
        sdk.step_raw([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
                     steps=budget)

    state = sdk.get_state()
    dest = destination(state, basket, basket_source)
    obj = state["objects"][ketchup]["pos"]
    yaw = state[basket_source][basket]["yaw"]
    dx = obj[0] - dest[0]
    dy = obj[1] - dest[1]
    local_x = cos(yaw) * dx + sin(yaw) * dy
    local_y = -sin(yaw) * dx + cos(yaw) * dy
    placed = (
        abs(local_x) < 0.08
        and abs(local_y) < 0.08
        and dest[2] - 0.075 < obj[2] < dest[2] + 0.10
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Released object settled inside basket" if placed
                  else "Final object pose outside basket region",
        "steps_used": state["steps_used"]
    }
