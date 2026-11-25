def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return min(450, state["max_steps"]) - state["steps_used"]

    def distance(a, b):
        return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    def nearest_grasp_yaw(yaw, reference):
        while yaw - reference > pi / 2:
            yaw = yaw - pi
        while yaw - reference < -pi / 2:
            yaw = yaw + pi
        return yaw

    def move(pos, yaw, grip, limit=50, tol=0.012):
        budget = remaining()
        if budget <= 0:
            return False
        result = sdk.move_to(
            pos,
            yaw=yaw,
            gripper=grip,
            max_steps=min(limit, budget),
            tol=tol,
        )
        return result["reached"] or distance(
            sdk.get_state()["eef_pos"], pos
        ) <= tol

    def gripper(action, steps=12):
        budget = remaining()
        if budget > 0:
            return sdk.set_gripper(action, steps=min(steps, budget))
        return {}

    def basket_pose(state, name):
        if name in state["objects"]:
            return state["objects"][name]["pos"]
        return state["fixtures"][name]["pos"]

    def basket_target(state, basket_name):
        sites = state["sites"]
        preferred = basket_name + "_contain_region"
        if preferred in sites:
            return list(sites[preferred])
        for name in sites:
            if "basket" in name and "contain_region" in name:
                return list(sites[name])
        pos = basket_pose(state, basket_name)
        return [pos[0], pos[1], pos[2] + 0.07]

    def clearance_height(state, soup_name, basket_name):
        soup = state["objects"][soup_name]["pos"]
        target = basket_target(state, basket_name)
        height = max(soup[2] + 0.19, target[2] + 0.19)
        for name in state["objects"]:
            height = max(height, state["objects"][name]["pos"][2] + 0.14)
        return height

    def lifted_grasp(state, soup_name, original, hand_before):
        soup = state["objects"][soup_name]["pos"]
        hand = state["eef_pos"]
        object_rise = soup[2] - original[2]
        hand_rise = hand[2] - hand_before[2]
        return (
            object_rise > 0.035
            and hand_rise > 0.04
            and abs(object_rise - hand_rise) < 0.035
            and hypot(soup[0] - hand[0], soup[1] - hand[1]) < 0.065
            and abs(soup[2] - hand[2]) < 0.12
        )

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
        return {"status": "failed", "reason": "Required objects unavailable"}

    held = False
    carry_yaw = state["eef_yaw"]
    stage = "approach"

    for attempt in range(3):
        if remaining() < 175:
            break

        state = sdk.get_state()
        clearance = clearance_height(state, soup_name, basket_name)
        hand = state["eef_pos"]

        # Lift before rotating; a parallel gripper is invariant to a half turn.
        stage = "clearance"
        if not move(
            [hand[0], hand[1], max(hand[2], clearance)],
            state["eef_yaw"], None, 35, 0.018
        ):
            continue
        gripper("open", 8)

        state = sdk.get_state()
        soup = list(state["objects"][soup_name]["pos"])
        desired_yaw = state["objects"][soup_name]["yaw"]
        if attempt == 1:
            desired_yaw = desired_yaw + pi / 2
        yaw = nearest_grasp_yaw(desired_yaw, state["eef_yaw"])

        stage = "alignment"
        if not move([soup[0], soup[1], clearance], yaw, "open", 45):
            continue

        # Use the achieved wrist orientation, avoiding repeated yaw wrap commands.
        state = sdk.get_state()
        yaw = state["eef_yaw"]
        soup = list(state["objects"][soup_name]["pos"])
        stage = "descent"
        if not move(
            [soup[0], soup[1], soup[2] + 0.075],
            yaw, "open", 30, 0.010
        ):
            continue

        state = sdk.get_state()
        soup = list(state["objects"][soup_name]["pos"])
        height_offset = 0.008
        if attempt == 2:
            height_offset = -0.004
        grasp = [soup[0], soup[1], soup[2] + height_offset]
        if not move(grasp, yaw, "open", 28, 0.009):
            state = sdk.get_state()
            hand = state["eef_pos"]
            if (
                hypot(hand[0] - grasp[0], hand[1] - grasp[1]) > 0.012
                or abs(hand[2] - grasp[2]) > 0.018
            ):
                continue

        gripper("close", 16)
        before = sdk.get_state()
        original = list(before["objects"][soup_name]["pos"])
        hand_before = list(before["eef_pos"])
        stage = "lift verification"
        move(
            [hand_before[0], hand_before[1], hand_before[2] + 0.085],
            yaw, "close", 28, 0.010
        )
        after = sdk.get_state()
        if lifted_grasp(after, soup_name, original, hand_before):
            held = True
            carry_yaw = after["eef_yaw"]
            break
        gripper("open", 8)

    if not held:
        return {
            "status": "failed",
            "reason": "Could not verify a lifted grasp",
            "stage": stage,
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    soup = state["objects"][soup_name]["pos"]
    hand = state["eef_pos"]
    offset = [soup[i] - hand[i] for i in range(3)]
    target = basket_target(state, basket_name)
    transit_height = max(
        hand[2],
        clearance_height(state, soup_name, basket_name),
        target[2] + 0.20 - offset[2],
    )

    if not move(
        [hand[0], hand[1], transit_height],
        carry_yaw, "close", 30, 0.015
    ):
        return {"status": "failed", "reason": "Transport clearance not reached"}

    destination = [
        target[0] - offset[0],
        target[1] - offset[1],
        transit_height,
    ]
    if not move(destination, carry_yaw, "close", 60):
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

    offset = [soup[i] - hand[i] for i in range(3)]
    target = basket_target(state, basket_name)
    release = [
        target[0] - offset[0],
        target[1] - offset[1],
        target[2] + 0.065 - offset[2],
    ]
    if not move(release, carry_yaw, "close", 40, 0.010):
        return {"status": "failed", "reason": "Release position was not reached"}

    gripper("open", 12)
    state = sdk.get_state()
    hand = state["eef_pos"]
    move(
        [hand[0], hand[1], hand[2] + 0.16],
        carry_yaw, "open", 28,
    )

    budget = remaining()
    if budget > 0:
        sdk.step_raw([0, 0, 0, 0, 0, 0, -1], steps=min(20, budget))

    state = sdk.get_state()
    soup = state["objects"][soup_name]["pos"]
    target = basket_target(state, basket_name)
    basket = basket_pose(state, basket_name)
    placed = (
        hypot(soup[0] - target[0], soup[1] - target[1]) < 0.075
        and basket[2] + 0.015 < soup[2] < target[2] + 0.065
        and state["eef_pos"][2] - soup[2] > 0.10
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Placement verified by live pose" if placed else "Placement not verified",
        "steps_used": state["steps_used"],
    }
