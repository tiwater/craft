def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, grip, limit=40, tol=0.015):
        budget = remaining()
        if budget > 0:
            sdk.move_to(
                pos, yaw=yaw, gripper=grip,
                max_steps=min(limit, budget), tol=tol
            )
        return sdk.get_state()

    def grip(action, steps=12):
        budget = remaining()
        if budget > 0:
            sdk.set_gripper(action, steps=min(steps, budget))
        return sdk.get_state()

    def nearest_yaw(yaw, reference):
        while yaw - reference > pi / 2:
            yaw -= pi
        while yaw - reference < -pi / 2:
            yaw += pi
        return yaw

    def basket_pose(state, name):
        if name in state["objects"]:
            return list(state["objects"][name]["pos"])
        return list(state["fixtures"][name]["pos"])

    def target_pose(state, name):
        preferred = name + "_contain_region"
        if preferred in state["sites"]:
            return list(state["sites"][preferred])
        for site in state["sites"]:
            if "basket" in site and "contain" in site:
                return list(state["sites"][site])
        pos = basket_pose(state, name)
        return [pos[0], pos[1], pos[2] + 0.07]

    state = sdk.get_state()
    soup_name = None
    basket_name = None
    for name in state["objects"]:
        if "alphabet_soup" in name:
            soup_name = name
        if "basket" in name:
            basket_name = name
    if basket_name is None:
        for name in state["fixtures"]:
            if "basket" in name:
                basket_name = name
    if soup_name is None or basket_name is None:
        return {"status": "failed", "reason": "Required scene poses unavailable"}

    held = False
    carry_yaw = state["eef_yaw"]
    offset = [0.0, 0.0, 0.0]

    # Use measured progress, not a strict controller convergence flag.
    # Grasp slightly above the object origin to avoid table contact.
    for attempt in range(3):
        if remaining() < 165:
            break
        state = sdk.get_state()
        soup = list(state["objects"][soup_name]["pos"])
        start_z = soup[2]
        yaw = nearest_yaw(
            state["objects"][soup_name]["yaw"] + (attempt % 2) * pi / 2,
            state["eef_yaw"]
        )
        approach_z = soup[2] + 0.15
        hand = state["eef_pos"]
        if hand[2] < approach_z - 0.025:
            state = move(
                [hand[0], hand[1], approach_z],
                state["eef_yaw"], "close", 25
            )
        grip("open", 8)
        state = move([soup[0], soup[1], approach_z], yaw, "open", 40)
        hand = state["eef_pos"]
        if hypot(hand[0] - soup[0], hand[1] - soup[1]) > 0.04:
            continue

        soup = list(state["objects"][soup_name]["pos"])
        heights = [0.025, 0.012, 0.04]
        grasp_z = soup[2] + heights[attempt]
        state = move(
            [soup[0], soup[1], grasp_z], yaw, "open", 35, 0.012
        )
        soup = list(state["objects"][soup_name]["pos"])
        hand = state["eef_pos"]

        if (
            hypot(hand[0] - soup[0], hand[1] - soup[1]) > 0.018
            or hand[2] - soup[2] > 0.065
        ):
            state = move(
                [soup[0], soup[1], grasp_z], yaw, "open", 15, 0.014
            )
            soup = list(state["objects"][soup_name]["pos"])
            hand = state["eef_pos"]

        if (
            hypot(hand[0] - soup[0], hand[1] - soup[1]) > 0.03
            or hand[2] - soup[2] > 0.075
            or hand[2] - soup[2] < -0.035
        ):
            continue

        state = grip("close", 16)
        before = list(state["objects"][soup_name]["pos"])
        hand = list(state["eef_pos"])
        state = move(
            [hand[0], hand[1], hand[2] + 0.11],
            yaw, "close", 30, 0.018
        )
        soup = state["objects"][soup_name]["pos"]
        hand = state["eef_pos"]
        if (
            soup[2] - before[2] > 0.045
            and soup[2] - start_z > 0.045
            and hypot(soup[0] - hand[0], soup[1] - hand[1]) < 0.07
            and abs(soup[2] - hand[2]) < 0.13
        ):
            held = True
            carry_yaw = yaw
            offset = [soup[i] - hand[i] for i in range(3)]
            break
        grip("open", 6)

    if not held:
        return {
            "status": "failed",
            "reason": "No object lift detected",
            "steps_used": sdk.get_state()["steps_used"]
        }

    state = sdk.get_state()
    target = target_pose(state, basket_name)
    basket = basket_pose(state, basket_name)
    hand = state["eef_pos"]
    transit_z = max(
        hand[2],
        target[2] + 0.16 - offset[2],
        basket[2] + 0.22 - offset[2]
    )
    state = move(
        [hand[0], hand[1], transit_z], carry_yaw, "close", 25
    )
    state = move(
        [target[0] - offset[0], target[1] - offset[1], transit_z],
        carry_yaw, "close", 50
    )

    soup = state["objects"][soup_name]["pos"]
    hand = state["eef_pos"]
    if any(abs(soup[i] - hand[i] - offset[i]) > 0.055 for i in range(3)):
        return {"status": "failed", "reason": "Grasp lost during transport"}

    target = target_pose(state, basket_name)
    if hypot(soup[0] - target[0], soup[1] - target[1]) > 0.025:
        state = move(
            [target[0] - offset[0], target[1] - offset[1], transit_z],
            carry_yaw, "close", 20, 0.01
        )
    soup = state["objects"][soup_name]["pos"]
    if hypot(soup[0] - target[0], soup[1] - target[1]) > 0.045:
        return {"status": "failed", "reason": "Basket alignment not achieved"}

    state = move(
        [
            target[0] - offset[0],
            target[1] - offset[1],
            target[2] + 0.065 - offset[2]
        ],
        carry_yaw, "close", 35, 0.012
    )
    soup = state["objects"][soup_name]["pos"]
    if (
        hypot(soup[0] - target[0], soup[1] - target[1]) > 0.045
        or soup[2] > target[2] + 0.12
    ):
        return {"status": "failed", "reason": "Release pose not achieved"}

    state = grip("open", 12)
    hand = state["eef_pos"]
    move(
        [hand[0], hand[1], hand[2] + 0.15],
        carry_yaw, "open", 25
    )
    budget = remaining()
    if budget > 0:
        sdk.step_raw([0, 0, 0, 0, 0, 0, -1], steps=min(20, budget))

    state = sdk.get_state()
    soup = state["objects"][soup_name]["pos"]
    target = target_pose(state, basket_name)
    basket = basket_pose(state, basket_name)
    placed = (
        hypot(soup[0] - target[0], soup[1] - target[1]) < 0.075
        and basket[2] + 0.015 < soup[2] < target[2] + 0.065
        and state["eef_pos"][2] - soup[2] > 0.10
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Placement verified" if placed else "Placement not verified",
        "steps_used": state["steps_used"]
    }
