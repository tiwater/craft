def run_policy(sdk):
    # Grasp above the bottle center to avoid palm contact with its top.
    # Use measured motion and object tracking instead of strict reached flags.
    def remaining():
        state = sdk.get_state()
        return max(0, state["max_steps"] - state["steps_used"])

    def move(position, yaw, grip, steps=35, tolerance=0.012):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.move_to(
                position, yaw=yaw, gripper=grip,
                max_steps=budget, tol=tolerance
            )

    def actuate(action, steps=10):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def target(state, basket):
        for name in state["sites"]:
            if basket in name and "contain" in name:
                return list(state["sites"][name])
        for name in state["sites"]:
            if "basket" in name and "contain" in name:
                return list(state["sites"][name])
        if basket in state["objects"]:
            position = state["objects"][basket]["pos"]
        else:
            position = state["fixtures"][basket]["pos"]
        return [position[0], position[1], position[2] + 0.075]

    def nearest_yaw(yaw, current):
        while yaw - current > pi / 2:
            yaw -= pi
        while yaw - current < -pi / 2:
            yaw += pi
        return yaw

    state = sdk.get_state()
    sauce = None
    basket = None
    for name in state["objects"]:
        if "bbq" in name and "sauce" in name:
            sauce = name
        if "basket" in name:
            basket = name
    if basket is None:
        for name in state["fixtures"]:
            if "basket" in name:
                basket = name
    if sauce is None or basket is None:
        return {"status": "failed", "reason": "Required scene entities unavailable"}

    held = False
    yaw = state["eef_yaw"]
    heights = [0.055, 0.035, 0.075]

    for attempt in range(3):
        if remaining() < 165:
            break

        state = sdk.get_state()
        position = list(state["objects"][sauce]["pos"])
        yaw = nearest_yaw(
            state["objects"][sauce]["yaw"] + (pi / 2 if attempt != 1 else 0),
            state["eef_yaw"]
        )
        actuate("open", 8)

        state = sdk.get_state()
        approach_z = max(position[2] + 0.18, state["eef_pos"][2])
        move(
            [state["eef_pos"][0], state["eef_pos"][1], approach_z],
            state["eef_yaw"], "open", 20
        )
        move([position[0], position[1], approach_z], yaw, "open", 35)

        state = sdk.get_state()
        position = list(state["objects"][sauce]["pos"])
        if hypot(
            state["eef_pos"][0] - position[0],
            state["eef_pos"][1] - position[1]
        ) > 0.025:
            continue

        grasp = [position[0], position[1], position[2] + heights[attempt]]
        move(grasp, yaw, "open", 30, 0.009)
        state = sdk.get_state()
        position = list(state["objects"][sauce]["pos"])
        if hypot(
            state["eef_pos"][0] - position[0],
            state["eef_pos"][1] - position[1]
        ) > 0.025:
            continue
        if abs(state["eef_pos"][2] - grasp[2]) > 0.035:
            continue

        before = list(position)
        actuate("close", 14)
        state = sdk.get_state()
        lift = [
            state["eef_pos"][0],
            state["eef_pos"][1],
            state["eef_pos"][2] + 0.11
        ]
        move(lift, yaw, "close", 28)
        state = sdk.get_state()
        position = state["objects"][sauce]["pos"]
        separation = sqrt(sum(
            (state["eef_pos"][axis] - position[axis]) ** 2
            for axis in range(3)
        ))
        if position[2] - before[2] > 0.045 and separation < 0.16:
            held = True
            break

    if not held:
        return {"status": "failed", "reason": "No verified lifted grasp"}

    state = sdk.get_state()
    destination = target(state, basket)
    position = state["objects"][sauce]["pos"]
    offset = [
        state["eef_pos"][axis] - position[axis]
        for axis in range(3)
    ]
    transit_z = max(
        state["eef_pos"][2],
        destination[2] + 0.14 + offset[2]
    )
    move(
        [state["eef_pos"][0], state["eef_pos"][1], transit_z],
        yaw, "close", 24
    )

    # Split the transfer and refresh the object-to-hand offset at the destination.
    state = sdk.get_state()
    destination = target(state, basket)
    endpoint = [
        destination[0] + offset[0],
        destination[1] + offset[1],
        transit_z
    ]
    midpoint = [
        (state["eef_pos"][0] + endpoint[0]) / 2,
        (state["eef_pos"][1] + endpoint[1]) / 2,
        transit_z
    ]
    move(midpoint, yaw, "close", 28)
    move(endpoint, yaw, "close", min(40, max(0, remaining() - 55)))

    for correction in range(2):
        state = sdk.get_state()
        position = state["objects"][sauce]["pos"]
        destination = target(state, basket)
        horizontal_error = hypot(
            position[0] - destination[0],
            position[1] - destination[1]
        )
        separation = sqrt(sum(
            (state["eef_pos"][axis] - position[axis]) ** 2
            for axis in range(3)
        ))
        if separation > 0.18:
            return {"status": "failed", "reason": "Grasp lost during transfer"}
        if horizontal_error < 0.025:
            break
        if remaining() < 55:
            break
        move(
            [
                state["eef_pos"][0] + destination[0] - position[0],
                state["eef_pos"][1] + destination[1] - position[1],
                state["eef_pos"][2]
            ],
            yaw, "close", min(25, remaining() - 35), 0.012
        )

    state = sdk.get_state()
    position = state["objects"][sauce]["pos"]
    destination = target(state, basket)
    if hypot(position[0] - destination[0], position[1] - destination[1]) > 0.055:
        return {"status": "failed", "reason": "Could not center bottle over basket"}
    if remaining() < 24:
        return {"status": "failed", "reason": "Insufficient release budget"}

    offset_z = state["eef_pos"][2] - position[2]
    move(
        [
            state["eef_pos"][0] + destination[0] - position[0],
            state["eef_pos"][1] + destination[1] - position[1],
            destination[2] + 0.045 + offset_z
        ],
        yaw, "close", min(28, remaining() - 24), 0.012
    )
    actuate("open", 10)
    state = sdk.get_state()
    move(
        [state["eef_pos"][0], state["eef_pos"][1], state["eef_pos"][2] + 0.12],
        yaw, "open", min(20, max(0, remaining() - 12))
    )
    budget = min(18, remaining())
    if budget > 0:
        sdk.step_raw([0, 0, 0, 0, 0, 0, -1], steps=budget)

    state = sdk.get_state()
    position = state["objects"][sauce]["pos"]
    destination = target(state, basket)
    centered = hypot(
        position[0] - destination[0],
        position[1] - destination[1]
    ) < 0.075
    settled = destination[2] - 0.09 < position[2] < destination[2] + 0.09
    if centered and settled:
        return {"status": "done"}
    return {"status": "failed", "reason": "Final placement could not be verified"}
