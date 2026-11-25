def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, grip, budget, tol=0.012):
        steps = min(budget, remaining())
        if steps <= 0:
            return False
        result = sdk.move_to(
            pos, yaw=yaw, gripper=grip, max_steps=steps, tol=tol
        )
        return result["reached"]

    def gripper(action, budget):
        steps = min(budget, remaining())
        if steps > 0:
            return sdk.set_gripper(action, steps=steps)
        return {"gripper_width": 0.0, "grasped_hint": False}

    def basket_center(state, basket):
        preferred = basket + "_contain_region"
        if preferred in state["sites"]:
            return list(state["sites"][preferred])
        for name in sorted(state["sites"]):
            if basket in name and "contain" in name:
                return list(state["sites"][name])
        pos = state["objects"][basket]["pos"]
        return [pos[0], pos[1], pos[2] + 0.075]

    def nearest_yaw(angle, reference):
        while angle - reference > pi / 2.0:
            angle -= pi
        while angle - reference < -pi / 2.0:
            angle += pi
        return angle

    state = sdk.get_state()
    tomato = None
    basket = None
    for name in sorted(state["objects"]):
        if "tomato_sauce" in name:
            tomato = name
        if "basket" in name:
            basket = name
    if tomato is None or basket is None:
        return {"status": "failed", "reason": "Required objects unavailable"}

    held = False
    offset = [0.0, 0.0, 0.0]
    grasp_yaw = state["eef_yaw"]

    for attempt in range(3):
        if remaining() < 170:
            break

        state = sdk.get_state()
        obj = list(state["objects"][tomato]["pos"])
        center = basket_center(state, basket)
        eef = state["eef_pos"]

        # Retreat vertically before changing orientation near clutter.
        clearance = max(eef[2], obj[2] + 0.15, center[2] + 0.15)
        gripper("open", 6)
        move([eef[0], eef[1], clearance], state["eef_yaw"], "open", 22)

        state = sdk.get_state()
        angle = state["objects"][tomato]["yaw"]
        if attempt == 1:
            angle += pi / 2.0
        grasp_yaw = nearest_yaw(angle, state["eef_yaw"])
        obj = list(state["objects"][tomato]["pos"])
        move([obj[0], obj[1], clearance], grasp_yaw, "open", 35)

        state = sdk.get_state()
        obj = list(state["objects"][tomato]["pos"])
        eef = state["eef_pos"]
        if hypot(eef[0] - obj[0], eef[1] - obj[1]) > 0.025:
            continue

        # A low pose may be contact-limited even when the fingers surround
        # the object. Judge the live geometry rather than reached alone.
        height_offset = 0.025
        if attempt == 1:
            height_offset = 0.035
        if attempt == 2:
            height_offset = 0.008
        move(
            [obj[0], obj[1], obj[2] + height_offset],
            grasp_yaw,
            "open",
            30,
            0.012,
        )

        state = sdk.get_state()
        obj = list(state["objects"][tomato]["pos"])
        eef = state["eef_pos"]
        lateral_error = hypot(eef[0] - obj[0], eef[1] - obj[1])
        if lateral_error > 0.018:
            move(
                [obj[0], obj[1], max(eef[2], obj[2] + height_offset)],
                grasp_yaw,
                "open",
                12,
            )
            state = sdk.get_state()
            obj = list(state["objects"][tomato]["pos"])
            eef = state["eef_pos"]

        if (
            hypot(eef[0] - obj[0], eef[1] - obj[1]) > 0.028
            or eef[2] - obj[2] > 0.075
            or eef[2] - obj[2] < -0.035
        ):
            continue

        before = list(obj)
        gripper("close", 15)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move(
            [eef[0], eef[1], eef[2] + 0.105],
            grasp_yaw,
            "close",
            27,
        )

        state = sdk.get_state()
        lifted = state["objects"][tomato]["pos"]
        eef = state["eef_pos"]
        if (
            lifted[2] - before[2] > 0.045
            and hypot(lifted[0] - eef[0], lifted[1] - eef[1]) < 0.065
            and state["gripper_width"] > 0.01
        ):
            held = True
            offset = [lifted[i] - eef[i] for i in range(3)]
            break
        gripper("open", 6)

    if not held:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    center = basket_center(state, basket)
    eef = state["eef_pos"]
    transit_z = max(eef[2], center[2] + 0.18 - offset[2])
    move([eef[0], eef[1], transit_z], grasp_yaw, "close", 25)

    state = sdk.get_state()
    if state["objects"][tomato]["pos"][2] < center[2] + 0.11:
        return {"status": "failed", "reason": "Insufficient transfer clearance"}

    move(
        [center[0] - offset[0], center[1] - offset[1], transit_z],
        grasp_yaw,
        "close",
        55,
    )
    state = sdk.get_state()
    obj = state["objects"][tomato]["pos"]
    eef = state["eef_pos"]
    measured_offset = [obj[i] - eef[i] for i in range(3)]
    if (
        abs(measured_offset[2] - offset[2]) > 0.045
        or hypot(
            measured_offset[0] - offset[0],
            measured_offset[1] - offset[1],
        ) > 0.045
    ):
        return {"status": "failed", "reason": "Object slipped during transfer"}

    center = basket_center(state, basket)
    if hypot(obj[0] - center[0], obj[1] - center[1]) > 0.025:
        move(
            [
                center[0] - measured_offset[0],
                center[1] - measured_offset[1],
                eef[2],
            ],
            grasp_yaw,
            "close",
            18,
            0.01,
        )

    if remaining() < 48:
        return {"status": "failed", "reason": "Insufficient release budget"}

    state = sdk.get_state()
    center = basket_center(state, basket)
    obj = state["objects"][tomato]["pos"]
    eef = state["eef_pos"]
    measured_offset = [obj[i] - eef[i] for i in range(3)]
    move(
        [
            center[0] - measured_offset[0],
            center[1] - measured_offset[1],
            center[2] + 0.035 - measured_offset[2],
        ],
        grasp_yaw,
        "close",
        min(35, remaining() - 28),
        0.01,
    )

    state = sdk.get_state()
    center = basket_center(state, basket)
    obj = state["objects"][tomato]["pos"]
    if (
        hypot(obj[0] - center[0], obj[1] - center[1]) > 0.04
        or obj[2] > center[2] + 0.11
        or obj[2] < center[2] - 0.065
    ):
        return {"status": "failed", "reason": "Unsafe release geometry"}

    gripper("open", 10)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.15], grasp_yaw, "open", 22)
    gripper("open", 10)

    state = sdk.get_state()
    obj = state["objects"][tomato]["pos"]
    center = basket_center(state, basket)
    basket_yaw = state["objects"][basket]["yaw"]
    dx = obj[0] - center[0]
    dy = obj[1] - center[1]
    local_x = cos(basket_yaw) * dx + sin(basket_yaw) * dy
    local_y = -sin(basket_yaw) * dx + cos(basket_yaw) * dy
    placed = (
        abs(local_x) < 0.075
        and abs(local_y) < 0.075
        and center[2] - 0.065 < obj[2] < center[2] + 0.07
        and state["gripper_width"] > 0.04
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Placement verified geometrically" if placed else "Placement could not be verified",
        "steps_used": state["steps_used"],
    }
