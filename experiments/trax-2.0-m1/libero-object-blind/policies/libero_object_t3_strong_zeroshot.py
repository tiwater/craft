def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, state["max_steps"] - state["steps_used"])

    def move(position, yaw, grip, limit=55, tolerance=0.012):
        budget = min(limit, remaining())
        if budget <= 0:
            return False
        result = sdk.move_to(
            position,
            yaw=yaw,
            gripper=grip,
            max_steps=budget,
            tol=tolerance
        )
        return result["reached"]

    def gripper(action, steps=12):
        budget = min(steps, remaining())
        if budget > 0:
            return sdk.set_gripper(action, steps=budget)
        return {}

    def basket_target(state, basket):
        for name in state["sites"]:
            if basket in name and "contain" in name:
                return list(state["sites"][name])
        position = state["objects"][basket]["pos"]
        return [position[0], position[1], position[2] + 0.075]

    state = sdk.get_state()
    sauce = None
    basket = None
    for name in state["objects"]:
        if "bbq" in name and "sauce" in name:
            sauce = name
        if "basket" in name:
            basket = name
    if sauce is None or basket is None:
        return {"status": "failed", "reason": "Required objects unavailable"}

    held = False
    grasp_yaw = state["objects"][sauce]["yaw"]

    for attempt in range(3):
        if remaining() < 135:
            break

        state = sdk.get_state()
        position = list(state["objects"][sauce]["pos"])
        target = basket_target(state, basket)
        grasp_yaw = state["objects"][sauce]["yaw"] + attempt * pi / 2
        while grasp_yaw - state["eef_yaw"] > pi:
            grasp_yaw -= 2 * pi
        while grasp_yaw - state["eef_yaw"] < -pi:
            grasp_yaw += 2 * pi

        clearance = max(
            position[2] + 0.19,
            target[2] + 0.20,
            state["eef_pos"][2]
        )
        gripper("open", 8)
        if state["eef_pos"][2] < clearance - 0.025:
            move(
                [state["eef_pos"][0], state["eef_pos"][1], clearance],
                grasp_yaw, "open", 35
            )

        if not move(
            [position[0], position[1], clearance],
            grasp_yaw, "open", 50
        ):
            continue

        state = sdk.get_state()
        position = list(state["objects"][sauce]["pos"])
        grip_height = position[2] + (0.012 if attempt != 2 else 0.025)
        if not move(
            [position[0], position[1], grip_height],
            grasp_yaw, "open", 45, 0.007
        ):
            continue

        before = list(sdk.get_state()["objects"][sauce]["pos"])
        gripper("close", 15)
        state = sdk.get_state()
        lift = [
            state["eef_pos"][0],
            state["eef_pos"][1],
            max(state["eef_pos"][2] + 0.16, target[2] + 0.20)
        ]
        move(lift, grasp_yaw, "close", 45)

        state = sdk.get_state()
        after = state["objects"][sauce]["pos"]
        separation = sqrt(sum(
            (state["eef_pos"][axis] - after[axis]) ** 2
            for axis in range(3)
        ))
        if after[2] - before[2] > 0.065 and separation < 0.13:
            held = True
            break

    if not held:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    position = state["objects"][sauce]["pos"]
    offset = [
        state["eef_pos"][axis] - position[axis]
        for axis in range(3)
    ]
    target = basket_target(state, basket)
    transit_height = max(state["eef_pos"][2], target[2] + 0.22 + offset[2])

    if not move(
        [state["eef_pos"][0], state["eef_pos"][1], transit_height],
        grasp_yaw, "close", 30
    ):
        return {"status": "failed", "reason": "Unable to reach safe transfer height"}

    if not move(
        [target[0] + offset[0], target[1] + offset[1], transit_height],
        grasp_yaw, "close", 55, 0.009
    ):
        return {"status": "failed", "reason": "Unable to reach basket"}

    state = sdk.get_state()
    target = basket_target(state, basket)
    position = state["objects"][sauce]["pos"]
    if position[2] < target[2] + 0.06:
        return {"status": "failed", "reason": "Grasp lost during transfer"}

    offset = [
        state["eef_pos"][axis] - position[axis]
        for axis in range(3)
    ]
    release_position = [
        target[0] + offset[0],
        target[1] + offset[1],
        target[2] + 0.085 + offset[2]
    ]
    if not move(release_position, grasp_yaw, "close", 40, 0.008):
        return {"status": "failed", "reason": "Unable to reach release position"}

    state = sdk.get_state()
    position = state["objects"][sauce]["pos"]
    target = basket_target(state, basket)
    if hypot(position[0] - target[0], position[1] - target[1]) > 0.045:
        return {"status": "failed", "reason": "Object not centered above basket"}

    if remaining() < 25:
        return {"status": "failed", "reason": "Insufficient time to release and settle"}

    gripper("open", 12)
    state = sdk.get_state()
    move(
        [state["eef_pos"][0], state["eef_pos"][1], state["eef_pos"][2] + 0.15],
        grasp_yaw, "open", min(30, max(1, remaining() - 12))
    )
    budget = min(20, remaining())
    if budget > 0:
        sdk.step_raw([0, 0, 0, 0, 0, 0, -1], steps=budget)

    state = sdk.get_state()
    target = basket_target(state, basket)
    position = state["objects"][sauce]["pos"]
    basket_position = state["objects"][basket]["pos"]
    centered = hypot(position[0] - target[0], position[1] - target[1]) < 0.085
    settled_height = basket_position[2] + 0.025 < position[2] < target[2] + 0.09

    if centered and settled_height:
        return {"status": "done"}
    return {"status": "failed", "reason": "Final placement could not be verified"}
