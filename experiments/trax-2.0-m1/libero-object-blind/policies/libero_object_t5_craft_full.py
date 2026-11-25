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
        sites = state["sites"]
        preferred = basket + "_contain_region"
        if preferred in sites:
            return list(sites[preferred])
        for name in sites:
            if basket in name and "contain" in name:
                return list(sites[name])
        pos = state["objects"][basket]["pos"]
        return [pos[0], pos[1], pos[2] + 0.075]

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
    grasp_yaw = state["objects"][tomato]["yaw"]

    for attempt in range(3):
        if remaining() < 175:
            break

        state = sdk.get_state()
        obj = list(state["objects"][tomato]["pos"])
        center = basket_center(state, basket)
        grasp_yaw = state["objects"][tomato]["yaw"]
        if attempt == 1:
            grasp_yaw += pi / 2.0
        grasp_yaw = atan2(sin(grasp_yaw), cos(grasp_yaw))

        clearance = max(
            state["eef_pos"][2], obj[2] + 0.16, center[2] + 0.19
        )
        eef = state["eef_pos"]
        if eef[2] < clearance - 0.02:
            move([eef[0], eef[1], clearance], grasp_yaw, "open", 25)
        gripper("open", 8)

        approached = move(
            [obj[0], obj[1], clearance], grasp_yaw, "open", 40
        )
        if not approached:
            state = sdk.get_state()
            obj = list(state["objects"][tomato]["pos"])
            eef = state["eef_pos"]
            aligned = hypot(eef[0] - obj[0], eef[1] - obj[1]) < 0.025
            above = eef[2] >= obj[2] + 0.09
            if not (aligned and above):
                if remaining() < 190:
                    continue
                move(
                    [obj[0], obj[1], clearance],
                    grasp_yaw, "open", 18, 0.018,
                )
                state = sdk.get_state()
                obj = list(state["objects"][tomato]["pos"])
                eef = state["eef_pos"]
                if (
                    hypot(eef[0] - obj[0], eef[1] - obj[1]) >= 0.03
                    or eef[2] < obj[2] + 0.08
                ):
                    continue

        state = sdk.get_state()
        obj = list(state["objects"][tomato]["pos"])
        grasp_height = obj[2] + (0.008 if attempt != 2 else -0.005)
        contacted = move(
            [obj[0], obj[1], grasp_height],
            grasp_yaw,
            "open",
            38,
            0.008,
        )
        if not contacted:
            state = sdk.get_state()
            obj = list(state["objects"][tomato]["pos"])
            eef = state["eef_pos"]
            lateral_error = hypot(eef[0] - obj[0], eef[1] - obj[1])
            height_error = eef[2] - obj[2]

            # A blocked descent can still leave the fingers around the object.
            # Recenter on its live pose before deciding whether to actuate.
            if (
                lateral_error < 0.055
                and -0.02 < height_error < 0.10
                and remaining() >= 165
            ):
                move(
                    [obj[0], obj[1], obj[2] + 0.025],
                    grasp_yaw, "open", 12, 0.012,
                )
                state = sdk.get_state()
                obj = list(state["objects"][tomato]["pos"])
                eef = state["eef_pos"]
                lateral_error = hypot(eef[0] - obj[0], eef[1] - obj[1])
                height_error = eef[2] - obj[2]

            if not (
                lateral_error < 0.035
                and -0.02 < height_error < 0.07
            ):
                continue

        before = list(sdk.get_state()["objects"][tomato]["pos"])
        gripper("close", 14)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move(
            [eef[0], eef[1], max(eef[2] + 0.12, center[2] + 0.16)],
            grasp_yaw,
            "close",
            35,
        )
        state = sdk.get_state()
        lifted = state["objects"][tomato]["pos"]
        eef = state["eef_pos"]
        if (
            lifted[2] - before[2] > 0.045
            and hypot(lifted[0] - eef[0], lifted[1] - eef[1]) < 0.065
            and state["gripper_width"] > 0.012
        ):
            held = True
            offset = [lifted[i] - eef[i] for i in range(3)]
            break
        gripper("open", 8)

    if not held:
        return {"status": "failed", "reason": "Could not verify a lifted grasp"}

    state = sdk.get_state()
    center = basket_center(state, basket)
    eef = state["eef_pos"]
    transit_z = max(eef[2], center[2] + 0.18 - offset[2])

    if not move(
        [eef[0], eef[1], transit_z], grasp_yaw, "close", 25
    ):
        return {"status": "failed", "reason": "Could not reach safe transfer height"}

    target = [
        center[0] - offset[0],
        center[1] - offset[1],
        transit_z,
    ]
    if not move(target, grasp_yaw, "close", 60):
        return {"status": "failed", "reason": "Could not reach basket"}

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
    release = [
        center[0] - measured_offset[0],
        center[1] - measured_offset[1],
        center[2] + 0.035 - measured_offset[2],
    ]
    if remaining() < 45:
        return {"status": "failed", "reason": "Insufficient steps for controlled release"}
    if not move(release, grasp_yaw, "close", 35, 0.008):
        return {"status": "failed", "reason": "Could not reach release pose"}

    gripper("open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.15], grasp_yaw, "open", 25)
    gripper("open", 15)

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
