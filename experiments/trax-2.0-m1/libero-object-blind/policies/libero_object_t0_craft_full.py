def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return min(450, state["max_steps"]) - state["steps_used"]

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
        return result["reached"]

    def gripper(action, steps=12):
        budget = remaining()
        if budget > 0:
            return sdk.set_gripper(action, steps=min(steps, budget))
        return {}

    def basket_target(state, basket_name):
        sites = state["sites"]
        preferred = basket_name + "_contain_region"
        if preferred in sites:
            return list(sites[preferred])
        for name in sites:
            if "basket" in name and "contain_region" in name:
                return list(sites[name])
        pos = state["objects"][basket_name]["pos"]
        return [pos[0], pos[1], pos[2] + 0.07]

    def nearby(pos, horizontal, vertical):
        hand = sdk.get_state()["eef_pos"]
        return (
            hypot(hand[0] - pos[0], hand[1] - pos[1]) <= horizontal
            and abs(hand[2] - pos[2]) <= vertical
        )

    def equivalent_yaw(yaw, current):
        delta = atan2(sin(yaw - current), cos(yaw - current))
        if delta > pi / 2:
            delta = delta - pi
        if delta < -pi / 2:
            delta = delta + pi
        return current + delta

    state = sdk.get_state()
    soup_names = []
    basket_names = []
    for name in state["objects"]:
        if "alphabet_soup" in name:
            soup_names.append(name)
        if "basket" in name:
            basket_names.append(name)

    if len(soup_names) != 1 or len(basket_names) != 1:
        return {"status": "failed", "reason": "Required object instances ambiguous or unavailable"}

    soup_name = soup_names[0]
    basket_name = basket_names[0]
    held = False
    carry_yaw = state["eef_yaw"]

    for attempt in range(3):
        if remaining() < 160:
            break

        state = sdk.get_state()
        soup = list(state["objects"][soup_name]["pos"])
        target = basket_target(state, basket_name)
        yaw = state["objects"][soup_name]["yaw"] + attempt * pi / 2
        clearance = max(soup[2] + 0.19, target[2] + 0.19)
        for name in state["objects"]:
            clearance = max(clearance, state["objects"][name]["pos"][2] + 0.14)

        eef = state["eef_pos"]
        raised = [eef[0], eef[1], clearance]
        if not move(raised, state["eef_yaw"], None, 40):
            if not nearby(raised, 0.035, 0.04):
                continue
        gripper("open")

        approach = [soup[0], soup[1], clearance]
        if not move(approach, yaw, "open", 50):
            state = sdk.get_state()
            yaw = equivalent_yaw(yaw, state["eef_yaw"])
            if not nearby(approach, 0.035, 0.04):
                move(approach, yaw, "open", 20, 0.02)
                if not nearby(approach, 0.035, 0.04):
                    continue

        state = sdk.get_state()
        soup = list(state["objects"][soup_name]["pos"])
        grasp_height = soup[2] + (0.008 if attempt != 1 else -0.004)
        contact = [soup[0], soup[1], grasp_height]
        reached = move(contact, yaw, "open", 45, 0.007)

        if not reached:
            state = sdk.get_state()
            soup = list(state["objects"][soup_name]["pos"])
            hand = state["eef_pos"]
            yaw = equivalent_yaw(yaw, state["eef_yaw"])
            contact = [
                soup[0],
                soup[1],
                soup[2] + (0.008 if attempt != 1 else -0.004),
            ]
            if hypot(hand[0] - soup[0], hand[1] - soup[1]) > 0.02:
                move(
                    [soup[0], soup[1], max(hand[2], soup[2] + 0.075)],
                    yaw,
                    "open",
                    12,
                    0.012,
                )
            move(contact, yaw, "open", 18, 0.012)
            state = sdk.get_state()
            soup = state["objects"][soup_name]["pos"]
            hand = state["eef_pos"]
            if (
                hypot(hand[0] - soup[0], hand[1] - soup[1]) > 0.025
                or hand[2] - soup[2] > 0.075
                or hand[2] - soup[2] < -0.025
            ):
                continue

        gripper("close", 16)
        before = sdk.get_state()
        original = list(before["objects"][soup_name]["pos"])
        eef = before["eef_pos"]
        move([eef[0], eef[1], clearance], yaw, "close", 45)

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
            carry_yaw = after["eef_yaw"]
            break
        gripper("open", 8)

    if not held:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    soup = state["objects"][soup_name]["pos"]
    hand = state["eef_pos"]
    target = basket_target(state, basket_name)
    offset = [soup[i] - hand[i] for i in range(3)]
    transit_height = max(hand[2], target[2] + 0.17 - offset[2])
    destination = [
        target[0] - offset[0],
        target[1] - offset[1],
        transit_height,
    ]
    move(
        destination,
        carry_yaw,
        "close",
        max(1, min(38, remaining() - 48)),
        0.025,
    )

    state = sdk.get_state()
    soup = state["objects"][soup_name]["pos"]
    hand = state["eef_pos"]
    target = basket_target(state, basket_name)
    if (
        hypot(soup[0] - hand[0], soup[1] - hand[1]) > 0.09
        or abs(soup[2] - hand[2]) > 0.14
    ):
        return {"status": "failed", "reason": "Object lost during transport"}

    horizontal_error = hypot(soup[0] - target[0], soup[1] - target[1])
    if horizontal_error > 0.035 and remaining() > 48:
        move(
            [
                hand[0] + target[0] - soup[0],
                hand[1] + target[1] - soup[1],
                hand[2],
            ],
            carry_yaw,
            "close",
            min(12, remaining() - 48),
            0.02,
        )

    state = sdk.get_state()
    soup = state["objects"][soup_name]["pos"]
    hand = state["eef_pos"]
    target = basket_target(state, basket_name)
    if hypot(soup[0] - target[0], soup[1] - target[1]) > 0.055:
        return {"status": "failed", "reason": "Object remains outside basket opening"}

    offset = [soup[i] - hand[i] for i in range(3)]
    release = [
        target[0] - offset[0],
        target[1] - offset[1],
        target[2] + 0.025 - offset[2],
    ]
    move(
        release,
        carry_yaw,
        "close",
        max(1, min(32, remaining() - 24)),
        0.018,
    )

    state = sdk.get_state()
    soup = state["objects"][soup_name]["pos"]
    hand = state["eef_pos"]
    target = basket_target(state, basket_name)
    if (
        hypot(soup[0] - target[0], soup[1] - target[1]) > 0.025
        and remaining() > 30
    ):
        move(
            [
                hand[0] + target[0] - soup[0],
                hand[1] + target[1] - soup[1],
                hand[2],
            ],
            carry_yaw,
            "close",
            min(8, remaining() - 24),
            0.015,
        )

    state = sdk.get_state()
    soup = state["objects"][soup_name]["pos"]
    target = basket_target(state, basket_name)
    if hypot(soup[0] - target[0], soup[1] - target[1]) > 0.05:
        return {"status": "failed", "reason": "Release alignment outside basket opening"}

    gripper("open", 10)
    state = sdk.get_state()
    hand = state["eef_pos"]
    move(
        [hand[0], hand[1], hand[2] + 0.16],
        carry_yaw,
        "open",
        max(1, min(20, remaining() - 6)),
        0.025,
    )
    budget = remaining()
    if budget > 0:
        sdk.step_raw([0, 0, 0, 0, 0, 0, -1], steps=min(12, budget))

    state = sdk.get_state()
    soup = state["objects"][soup_name]["pos"]
    target = basket_target(state, basket_name)
    basket = state["objects"][basket_name]["pos"]
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
