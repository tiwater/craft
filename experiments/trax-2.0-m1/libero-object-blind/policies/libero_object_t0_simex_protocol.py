def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return min(450, state["max_steps"]) - state["steps_used"]

    def move(pos, yaw, grip, limit=50, tol=0.012):
        budget = remaining()
        if budget <= 0:
            return False
        result = sdk.move_to(
            pos, yaw=yaw, gripper=grip,
            max_steps=min(limit, budget), tol=tol
        )
        return result["reached"]

    def gripper(action, steps=12):
        budget = remaining()
        if budget > 0:
            return sdk.set_gripper(action, steps=min(steps, budget))
        return {}

    def select_name(mapping, base):
        if base + "_1" in mapping:
            return base + "_1"
        if base in mapping:
            return base
        matches = []
        for name in mapping:
            if name.startswith(base + "_"):
                matches.append(name)
        if len(matches) == 1:
            return matches[0]
        return None

    def basket_target(state, name, kind):
        sites = state["sites"]
        for suffix in ["_contain_region", "_inside_region"]:
            key = name + suffix
            if key in sites:
                return list(sites[key]), key
        matches = []
        for key in sites:
            if key.startswith(name + "_") and "contain" in key:
                matches.append(key)
        if len(matches) == 1:
            return list(sites[matches[0]]), matches[0]
        pos = state[kind][name]["pos"]
        return [pos[0], pos[1], pos[2] + 0.07], "basket_pose"

    def nearest_grasp_yaw(yaw, current):
        while yaw - current > pi / 2:
            yaw = yaw - pi
        while yaw - current < -pi / 2:
            yaw = yaw + pi
        return yaw

    state = sdk.get_state()
    soup_name = select_name(state["objects"], "alphabet_soup")
    basket_name = select_name(state["objects"], "basket")
    basket_kind = "objects"
    if basket_name is None:
        basket_name = select_name(state["fixtures"], "basket")
        basket_kind = "fixtures"
    if soup_name is None or basket_name is None:
        return {"status": "failed", "reason": "Required grounding is unavailable or ambiguous"}

    held = False
    carry_yaw = state["eef_yaw"]
    target, target_site = basket_target(state, basket_name, basket_kind)

    for attempt in range(3):
        if remaining() < 160:
            break

        state = sdk.get_state()
        soup = list(state["objects"][soup_name]["pos"])
        target, target_site = basket_target(state, basket_name, basket_kind)
        yaw = nearest_grasp_yaw(
            state["objects"][soup_name]["yaw"] + attempt * pi / 2,
            state["eef_yaw"]
        )
        clearance = max(soup[2] + 0.19, target[2] + 0.19)
        for name in state["objects"]:
            clearance = max(clearance, state["objects"][name]["pos"][2] + 0.14)

        hand = state["eef_pos"]
        if not move([hand[0], hand[1], clearance], state["eef_yaw"], None, 40):
            continue
        gripper("open")

        if not move([soup[0], soup[1], clearance], yaw, "open", 50):
            continue

        state = sdk.get_state()
        soup = list(state["objects"][soup_name]["pos"])
        grasp_height = soup[2] + (0.008 if attempt != 1 else -0.004)
        if not move([soup[0], soup[1], grasp_height], yaw, "open", 45, 0.007):
            continue

        gripper("close", 16)
        before = sdk.get_state()
        original = list(before["objects"][soup_name]["pos"])
        hand = before["eef_pos"]
        move([hand[0], hand[1], clearance], yaw, "close", 45)

        after = sdk.get_state()
        lifted = after["objects"][soup_name]["pos"]
        hand = after["eef_pos"]
        if (
            lifted[2] - original[2] > 0.065
            and abs(lifted[0] - hand[0]) < 0.065
            and abs(lifted[1] - hand[1]) < 0.065
            and abs(lifted[2] - hand[2]) < 0.12
        ):
            held = True
            carry_yaw = yaw
            break
        gripper("open", 8)

    if not held:
        return {
            "status": "failed",
            "reason": "Could not verify a lifted grasp",
            "object": soup_name,
            "basket": basket_name,
            "target_site": target_site,
        }

    state = sdk.get_state()
    soup = state["objects"][soup_name]["pos"]
    hand = state["eef_pos"]
    offset = [soup[i] - hand[i] for i in range(3)]
    target, target_site = basket_target(state, basket_name, basket_kind)
    transit_height = max(hand[2], target[2] + 0.20 - offset[2])

    if not move(
        [target[0] - offset[0], target[1] - offset[1], transit_height],
        carry_yaw, "close", 65
    ):
        return {"status": "failed", "reason": "Basket approach was not reached"}

    state = sdk.get_state()
    soup = state["objects"][soup_name]["pos"]
    hand = state["eef_pos"]
    if (
        abs((soup[0] - hand[0]) - offset[0]) > 0.045
        or abs((soup[1] - hand[1]) - offset[1]) > 0.045
        or abs((soup[2] - hand[2]) - offset[2]) > 0.055
    ):
        return {"status": "failed", "reason": "Object slipped during transport"}

    target, target_site = basket_target(state, basket_name, basket_kind)
    release = [
        target[0] - offset[0],
        target[1] - offset[1],
        target[2] + 0.065 - offset[2],
    ]
    if not move(release, carry_yaw, "close", 45, 0.008):
        return {"status": "failed", "reason": "Release position was not reached"}

    gripper("open", 15)
    state = sdk.get_state()
    hand = state["eef_pos"]
    move([hand[0], hand[1], hand[2] + 0.16], carry_yaw, "open", 30)

    budget = remaining()
    if budget > 0:
        sdk.step_raw([0, 0, 0, 0, 0, 0, -1], steps=min(25, budget))

    state = sdk.get_state()
    soup = state["objects"][soup_name]["pos"]
    target, target_site = basket_target(state, basket_name, basket_kind)
    basket = state[basket_kind][basket_name]["pos"]
    placed = (
        hypot(soup[0] - target[0], soup[1] - target[1]) < 0.075
        and basket[2] + 0.015 < soup[2] < target[2] + 0.065
        and state["eef_pos"][2] - soup[2] > 0.10
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Placement verified by live pose" if placed else "Placement not verified",
        "object": soup_name,
        "basket": basket_name,
        "target_site": target_site,
        "target": target,
        "object_pos": list(soup),
        "steps_used": state["steps_used"],
    }
