def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, state["max_steps"] - state["steps_used"])

    def move(pos, yaw, action, steps, reserve=0, tol=0.012):
        budget = min(steps, max(0, remaining() - reserve))
        if budget <= 0:
            return False
        result = sdk.move_to(
            pos, yaw=yaw, gripper=action, max_steps=budget, tol=tol
        )
        return result["reached"]

    def grip(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            return sdk.set_gripper(action, steps=budget)
        return {"gripper_width": 0.0, "grasped_hint": False}

    def nearest_yaw(yaw, current):
        while yaw - current > pi / 2:
            yaw -= pi
        while yaw - current < -pi / 2:
            yaw += pi
        return yaw

    state = sdk.get_state()
    pudding = None
    basket = None
    basket_group = "objects"
    for name in state["objects"]:
        if "chocolate_pudding" in name:
            pudding = name
        if "basket" in name:
            basket = name
    if basket is None:
        for name in state["fixtures"]:
            if "basket" in name:
                basket = name
                basket_group = "fixtures"
    if pudding is None or basket is None:
        return {"status": "failed", "reason": "Required entities unavailable"}

    def destination(state):
        preferred = basket + "_contain_region"
        if preferred in state["sites"]:
            return list(state["sites"][preferred])
        for name in state["sites"]:
            if basket in name and "contain" in name:
                return list(state["sites"][name])
        pos = state[basket_group][basket]["pos"]
        return [pos[0], pos[1], pos[2] + 0.06]

    held = False
    grasp_yaw = state["eef_yaw"]

    # Start with the deeper, object-aligned grasp that previously lifted
    # successfully, preserving enough time for transport and release.
    for attempt in range(2):
        if remaining() < 205:
            break
        state = sdk.get_state()
        obj = list(state["objects"][pudding]["pos"])
        eef = list(state["eef_pos"])
        grasp_yaw = nearest_yaw(
            state["objects"][pudding]["yaw"] + attempt * pi / 2,
            state["eef_yaw"]
        )
        approach_z = max(obj[2] + 0.13, eef[2])
        grip("open", 5)
        if eef[2] < obj[2] + 0.10:
            move(
                [eef[0], eef[1], approach_z],
                state["eef_yaw"], "open", 18, 170
            )
        move(
            [obj[0], obj[1], approach_z],
            grasp_yaw, "open", 38, 170
        )
        state = sdk.get_state()
        obj = list(state["objects"][pudding]["pos"])
        eef = state["eef_pos"]
        if hypot(eef[0] - obj[0], eef[1] - obj[1]) > 0.025:
            move(
                [obj[0], obj[1], approach_z],
                grasp_yaw, "open", 16, 160
            )
        state = sdk.get_state()
        obj = list(state["objects"][pudding]["pos"])
        eef = state["eef_pos"]
        if hypot(eef[0] - obj[0], eef[1] - obj[1]) > 0.035:
            continue

        move(
            [obj[0], obj[1], obj[2] + 0.045],
            grasp_yaw, "open", 20, 155, 0.009
        )
        state = sdk.get_state()
        obj = list(state["objects"][pudding]["pos"])
        move(
            [obj[0], obj[1], obj[2] + (0.005 if attempt == 0 else -0.005)],
            grasp_yaw, "open", 20, 145, 0.006
        )
        grip("close", 10)
        state = sdk.get_state()
        closed_obj = list(state["objects"][pudding]["pos"])
        closed_eef = list(state["eef_pos"])
        move(
            [closed_eef[0], closed_eef[1], closed_eef[2] + 0.10],
            grasp_yaw, "close", 25, 120, 0.012
        )
        state = sdk.get_state()
        obj = state["objects"][pudding]["pos"]
        eef = state["eef_pos"]
        offset_change = sqrt(sum(
            ((obj[i] - eef[i]) - (closed_obj[i] - closed_eef[i])) ** 2
            for i in range(3)
        ))
        held = (
            obj[2] - closed_obj[2] > 0.035
            and eef[2] - closed_eef[2] > 0.04
            and offset_change < 0.035
            and hypot(obj[0] - eef[0], obj[1] - eef[1]) < 0.07
        )
        if held:
            break
        grip("open", 5)

    if not held:
        return {"status": "failed", "reason": "No verified grasp"}

    state = sdk.get_state()
    obj = state["objects"][pudding]["pos"]
    eef = state["eef_pos"]
    offset = [obj[i] - eef[i] for i in range(3)]
    target = destination(state)
    basket_pos = state[basket_group][basket]["pos"]
    carry_z = max(
        eef[2],
        target[2] + 0.12 - offset[2],
        basket_pos[2] + 0.17 - offset[2]
    )
    move(
        [eef[0], eef[1], carry_z],
        grasp_yaw, "close", 25, 85
    )

    # Use measured object alignment throughout transport, with a fixed
    # release reserve instead of consuming the horizon on grasp retries.
    for correction in range(4):
        state = sdk.get_state()
        obj = state["objects"][pudding]["pos"]
        eef = state["eef_pos"]
        target = destination(state)
        current_offset = [obj[i] - eef[i] for i in range(3)]
        if sqrt(sum(
            (current_offset[i] - offset[i]) ** 2 for i in range(3)
        )) > 0.055:
            return {"status": "failed", "reason": "Object slipped during transport"}
        if hypot(obj[0] - target[0], obj[1] - target[1]) < 0.018:
            break
        if remaining() <= 55:
            break
        move(
            [
                eef[0] + target[0] - obj[0],
                eef[1] + target[1] - obj[1],
                carry_z
            ],
            grasp_yaw, "close",
            55 if correction == 0 else 22,
            55, 0.010
        )

    state = sdk.get_state()
    obj = state["objects"][pudding]["pos"]
    eef = state["eef_pos"]
    target = destination(state)
    if hypot(obj[0] - target[0], obj[1] - target[1]) > 0.035:
        return {"status": "failed", "reason": "Basket alignment not achieved"}
    if remaining() < 24:
        return {"status": "failed", "reason": "Insufficient release budget"}

    offset = [obj[i] - eef[i] for i in range(3)]
    move(
        [
            target[0] - offset[0],
            target[1] - offset[1],
            target[2] + 0.04 - offset[2]
        ],
        grasp_yaw, "close", 30, 24, 0.010
    )
    state = sdk.get_state()
    obj = state["objects"][pudding]["pos"]
    target = destination(state)
    if hypot(obj[0] - target[0], obj[1] - target[1]) > 0.04:
        return {"status": "failed", "reason": "Object moved outside basket opening"}

    grip("open", 8)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.12],
        grasp_yaw, "open", 20, 6
    )
    grip("open", min(12, remaining()))

    state = sdk.get_state()
    obj = state["objects"][pudding]["pos"]
    target = destination(state)
    yaw = state[basket_group][basket]["yaw"]
    dx = obj[0] - target[0]
    dy = obj[1] - target[1]
    local_x = cos(yaw) * dx + sin(yaw) * dy
    local_y = -sin(yaw) * dx + cos(yaw) * dy
    placed = (
        abs(local_x) < 0.07
        and abs(local_y) < 0.07
        and target[2] - 0.09 < obj[2] < target[2] + 0.065
        and state["eef_pos"][2] - obj[2] > 0.08
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Placement verified" if placed else "Placement not verified"
    }
