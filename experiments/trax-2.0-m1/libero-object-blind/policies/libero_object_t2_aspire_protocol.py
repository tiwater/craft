def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, state["max_steps"] - state["steps_used"])

    def move(pos, yaw, grip, steps=40, tol=0.012):
        budget = min(steps, remaining())
        if budget <= 0:
            return False
        result = sdk.move_to(
            pos, yaw=yaw, gripper=grip, max_steps=budget, tol=tol
        )
        return result["reached"]

    def grip(action, steps=12):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def basket_pose(state, name):
        if name in state["objects"]:
            return list(state["objects"][name]["pos"])
        return list(state["fixtures"][name]["pos"])

    def destination(state, name):
        exact = name + "_contain_region"
        if exact in state["sites"]:
            return list(state["sites"][exact])
        for site in state["sites"]:
            if name in site and "contain" in site:
                return list(state["sites"][site])
        pos = basket_pose(state, name)
        return [pos[0], pos[1], pos[2] + 0.075]

    state = sdk.get_state()
    target = None
    basket = None
    for name in state["objects"]:
        if "salad_dressing" in name:
            target = name
        if "basket" in name:
            basket = name
    if basket is None:
        for name in state["fixtures"]:
            if "basket" in name:
                basket = name
    if target is None or basket is None:
        return {"status": "failed", "reason": "Required scene entities are missing"}

    # A blocked descent is not proof of a failed grasp. The fingers extend
    # below the reported end-effector position, so grasp above the body origin.
    grasped = False
    offset = None
    heights = [0.060, 0.040, 0.080]
    for attempt in range(3):
        if remaining() < 155:
            break
        state = sdk.get_state()
        pos = list(state["objects"][target]["pos"])
        yaw = state["objects"][target]["yaw"]
        if attempt == 1:
            yaw += pi / 2

        grip("open", 7)
        eef = list(sdk.get_state()["eef_pos"])
        approach_z = max(eef[2], pos[2] + 0.155)
        move([eef[0], eef[1], approach_z], yaw, "open", 24)
        move([pos[0], pos[1], approach_z], yaw, "open", 36)

        state = sdk.get_state()
        pos = list(state["objects"][target]["pos"])
        eef = state["eef_pos"]
        if hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.035:
            continue

        move(
            [pos[0], pos[1], pos[2] + heights[attempt]],
            yaw, "open", 32, 0.009
        )
        state = sdk.get_state()
        pos = list(state["objects"][target]["pos"])
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.030
            or eef[2] - pos[2] > 0.115
        ):
            continue

        before = list(pos)
        grip("close", 16)
        eef = list(sdk.get_state()["eef_pos"])
        move([eef[0], eef[1], eef[2] + 0.105], yaw, "close", 28)
        state = sdk.get_state()
        pos = state["objects"][target]["pos"]
        eef = state["eef_pos"]
        if (
            pos[2] - before[2] > 0.045
            and hypot(pos[0] - eef[0], pos[1] - eef[1]) < 0.075
        ):
            grasped = True
            offset = [pos[i] - eef[i] for i in range(3)]
            break

    if not grasped:
        return {"status": "failed", "reason": "No verified lifted grasp"}

    state = sdk.get_state()
    dest = destination(state, basket)
    base = basket_pose(state, basket)
    eef = list(state["eef_pos"])
    travel_z = max(
        eef[2], dest[2] + 0.145 - offset[2],
        base[2] + 0.205 - offset[2]
    )
    move([eef[0], eef[1], travel_z], yaw, "close", 28)
    state = sdk.get_state()
    if state["eef_pos"][2] < travel_z - 0.035:
        return {"status": "failed", "reason": "Insufficient transport clearance"}

    move(
        [dest[0] - offset[0], dest[1] - offset[1], travel_z],
        yaw, "close", 55, 0.012
    )
    state = sdk.get_state()
    pos = list(state["objects"][target]["pos"])
    eef = list(state["eef_pos"])
    current_offset = [pos[i] - eef[i] for i in range(3)]
    if (
        abs(current_offset[2] - offset[2]) > 0.055
        or hypot(
            current_offset[0] - offset[0],
            current_offset[1] - offset[1]
        ) > 0.045
    ):
        return {"status": "failed", "reason": "Grasp lost during transport"}

    offset = current_offset
    dest = destination(state, basket)
    move(
        [dest[0] - offset[0], dest[1] - offset[1], travel_z],
        yaw, "close", 20, 0.008
    )
    state = sdk.get_state()
    pos = state["objects"][target]["pos"]
    if hypot(pos[0] - dest[0], pos[1] - dest[1]) > 0.045:
        return {"status": "failed", "reason": "Could not center object over basket"}

    # Lower conservatively; contact with the basket may prevent exact arrival.
    release_z = dest[2] + 0.060 - offset[2]
    move(
        [dest[0] - offset[0], dest[1] - offset[1], release_z],
        yaw, "close", 32, 0.012
    )
    state = sdk.get_state()
    pos = state["objects"][target]["pos"]
    dest = destination(state, basket)
    if hypot(pos[0] - dest[0], pos[1] - dest[1]) > 0.055:
        return {"status": "failed", "reason": "Object moved outside basket opening"}

    if remaining() < 20:
        return {"status": "failed", "reason": "Insufficient time to release and settle"}
    grip("open", 12)
    eef = list(sdk.get_state()["eef_pos"])
    move([eef[0], eef[1], eef[2] + 0.14], yaw, "open", 24)
    grip("open", 18)

    state = sdk.get_state()
    pos = state["objects"][target]["pos"]
    dest = destination(state, basket)
    base = basket_pose(state, basket)
    centered = hypot(pos[0] - dest[0], pos[1] - dest[1]) < 0.075
    inside_height = base[2] - 0.015 < pos[2] < dest[2] + 0.13
    separated = state["eef_pos"][2] - pos[2] > 0.10
    if centered and inside_height and separated:
        return {"status": "done"}
    return {"status": "failed", "reason": "Final placement could not be verified"}
