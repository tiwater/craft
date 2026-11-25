def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, state["max_steps"] - state["steps_used"])

    def move(pos, yaw, grip, steps=45, tol=0.015):
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
            return sdk.set_gripper(action, steps=budget)
        return {}

    def select_object(state, token):
        exact = token + "_1"
        if exact in state["objects"]:
            return exact
        matches = [
            name for name in state["objects"]
            if name == token or name.startswith(token + "_")
        ]
        if len(matches) == 1:
            return matches[0]
        return None

    def basket_target(state):
        if basket in state["objects"]:
            base = state["objects"][basket]["pos"]
        else:
            base = state["fixtures"][basket]["pos"]
        for suffix in ["contain_region", "contain", "inside_region", "inside"]:
            name = basket + "_" + suffix
            if name in state["sites"]:
                point = state["sites"][name]
                if hypot(point[0] - base[0], point[1] - base[1]) < 0.12:
                    return list(point)
        candidates = []
        for name in state["sites"]:
            if name.startswith(basket + "_") and (
                "contain" in name or "inside" in name
            ):
                point = state["sites"][name]
                if hypot(point[0] - base[0], point[1] - base[1]) < 0.12:
                    candidates.append(name)
        if len(candidates) == 1:
            return list(state["sites"][candidates[0]])
        return [base[0], base[1], base[2] + 0.075]

    def nearest_yaw(yaw, current):
        while yaw - current > pi / 2:
            yaw -= pi
        while yaw - current < -pi / 2:
            yaw += pi
        return yaw

    state = sdk.get_state()
    sauce = select_object(state, "bbq_sauce")
    basket = select_object(state, "basket")
    if basket is None:
        matches = [
            name for name in state["fixtures"]
            if name == "basket" or name.startswith("basket_")
        ]
        if len(matches) == 1:
            basket = matches[0]
    if sauce is None or basket is None:
        return {"status": "failed", "reason": "Object grounding is ambiguous"}

    held = False
    yaw = state["eef_yaw"]
    for attempt in range(3):
        if remaining() < 150:
            break
        state = sdk.get_state()
        position = list(state["objects"][sauce]["pos"])
        target = basket_target(state)
        yaw = nearest_yaw(
            state["objects"][sauce]["yaw"] + (pi / 2 if attempt != 1 else 0),
            state["eef_yaw"]
        )
        clearance = max(position[2] + 0.17, target[2] + 0.16)
        grip("open", 8)
        state = sdk.get_state()
        if state["eef_pos"][2] < clearance - 0.015:
            move(
                [state["eef_pos"][0], state["eef_pos"][1], clearance],
                yaw, "open", 30
            )
        if not move(
            [position[0], position[1], clearance], yaw, "open", 40
        ):
            continue

        state = sdk.get_state()
        position = list(state["objects"][sauce]["pos"])
        height_offset = 0.035 if attempt < 2 else 0.015
        move(
            [position[0], position[1], position[2] + height_offset],
            yaw, "open", 40, 0.009
        )
        state = sdk.get_state()
        position = list(state["objects"][sauce]["pos"])
        if hypot(
            state["eef_pos"][0] - position[0],
            state["eef_pos"][1] - position[1]
        ) > 0.025:
            continue
        if abs(state["eef_pos"][2] - position[2] - height_offset) > 0.025:
            continue

        before = list(position)
        grip("close", 16)
        state = sdk.get_state()
        move(
            [
                state["eef_pos"][0],
                state["eef_pos"][1],
                max(state["eef_pos"][2] + 0.13, target[2] + 0.16)
            ],
            yaw, "close", 35
        )
        state = sdk.get_state()
        position = state["objects"][sauce]["pos"]
        separation = sqrt(sum(
            (state["eef_pos"][axis] - position[axis]) ** 2
            for axis in range(3)
        ))
        if position[2] - before[2] > 0.06 and separation < 0.13:
            held = True
            break

    if not held:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    target = basket_target(state)
    position = state["objects"][sauce]["pos"]
    offset = [
        state["eef_pos"][axis] - position[axis] for axis in range(3)
    ]
    height = max(state["eef_pos"][2], target[2] + 0.15 + offset[2])
    move(
        [state["eef_pos"][0], state["eef_pos"][1], height],
        yaw, "close", 25
    )
    move(
        [target[0] + offset[0], target[1] + offset[1], height],
        yaw, "close", 55, 0.018
    )

    for correction in range(2):
        state = sdk.get_state()
        target = basket_target(state)
        position = state["objects"][sauce]["pos"]
        if position[2] < target[2] + 0.04:
            return {"status": "failed", "reason": "Grasp lost during transfer"}
        error = hypot(position[0] - target[0], position[1] - target[1])
        if error < 0.025:
            break
        if remaining() < 65:
            break
        move(
            [
                state["eef_pos"][0] + target[0] - position[0],
                state["eef_pos"][1] + target[1] - position[1],
                state["eef_pos"][2]
            ],
            yaw, "close", 25, 0.012
        )

    state = sdk.get_state()
    target = basket_target(state)
    position = state["objects"][sauce]["pos"]
    if hypot(position[0] - target[0], position[1] - target[1]) > 0.045:
        return {"status": "failed", "reason": "Object not centered above basket"}
    if remaining() < 40:
        return {"status": "failed", "reason": "Insufficient release budget"}

    move(
        [
            state["eef_pos"][0] + target[0] - position[0],
            state["eef_pos"][1] + target[1] - position[1],
            state["eef_pos"][2] + target[2] + 0.065 - position[2]
        ],
        yaw, "close", min(30, remaining() - 30), 0.015
    )
    state = sdk.get_state()
    position = state["objects"][sauce]["pos"]
    target = basket_target(state)
    if hypot(position[0] - target[0], position[1] - target[1]) > 0.05:
        return {"status": "failed", "reason": "Placement alignment lost"}

    grip("open", 12)
    state = sdk.get_state()
    move(
        [state["eef_pos"][0], state["eef_pos"][1], state["eef_pos"][2] + 0.12],
        yaw, "open", min(25, max(1, remaining() - 15))
    )
    budget = min(20, remaining())
    if budget > 0:
        sdk.step_raw([0, 0, 0, 0, 0, 0, -1], steps=budget)

    state = sdk.get_state()
    target = basket_target(state)
    position = state["objects"][sauce]["pos"]
    if basket in state["objects"]:
        base = state["objects"][basket]["pos"]
    else:
        base = state["fixtures"][basket]["pos"]
    centered = hypot(position[0] - target[0], position[1] - target[1]) < 0.085
    settled = base[2] + 0.025 < position[2] < target[2] + 0.09
    if centered and settled:
        return {"status": "done"}
    return {"status": "failed", "reason": "Final placement could not be verified"}
