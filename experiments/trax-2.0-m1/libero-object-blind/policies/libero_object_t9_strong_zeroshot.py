def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return min(450, state["max_steps"]) - state["steps_used"]

    def move(pos, yaw, grip, limit=45, tol=0.012):
        budget = remaining()
        if budget < 1:
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
        if budget < 1:
            return {}
        return sdk.set_gripper(action, steps=min(steps, budget))

    def settle(grip, steps):
        budget = remaining()
        if budget > 0:
            sdk.step_raw(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, grip],
                steps=min(steps, budget),
            )

    def destination(state, basket):
        sites = state["sites"]
        preferred = basket + "_contain_region"
        if preferred in sites:
            return list(sites[preferred])
        for name in sites:
            if basket in name and "contain" in name:
                return list(sites[name])
        pos = state["objects"][basket]["pos"]
        return [pos[0], pos[1], pos[2] + 0.07]

    state = sdk.get_state()
    juice = None
    basket = None
    for name in state["objects"]:
        if "orange_juice" in name:
            juice = name
        if "basket" in name:
            basket = name

    if juice is None or basket is None:
        return {"status": "failed", "reason": "Required objects unavailable"}

    held = False
    grasp_yaw = state["objects"][juice]["yaw"]

    for attempt in range(2):
        if remaining() < 155:
            break

        state = sdk.get_state()
        obj = state["objects"][juice]
        start = list(obj["pos"])
        target = destination(state, basket)
        grasp_yaw = obj["yaw"] + attempt * pi / 2.0
        while grasp_yaw - state["eef_yaw"] > pi / 2.0:
            grasp_yaw -= pi
        while grasp_yaw - state["eef_yaw"] < -pi / 2.0:
            grasp_yaw += pi

        clearance = max(start[2] + 0.19, target[2] + 0.22)
        for name in state["objects"]:
            clearance = max(
                clearance, state["objects"][name]["pos"][2] + 0.16
            )

        gripper("open", 10)
        eef = sdk.get_state()["eef_pos"]
        if not move(
            [eef[0], eef[1], max(eef[2], clearance)],
            grasp_yaw, -1.0, 35
        ):
            continue

        if not move(
            [start[0], start[1], clearance], grasp_yaw, -1.0, 45
        ):
            continue

        state = sdk.get_state()
        start = list(state["objects"][juice]["pos"])
        grasp_height = start[2] + (0.025 if attempt == 0 else 0.01)
        if not move(
            [start[0], start[1], grasp_height],
            grasp_yaw, -1.0, 45, 0.008
        ):
            continue

        gripper("close", 15)
        state = sdk.get_state()
        before_lift = list(state["objects"][juice]["pos"])
        eef = state["eef_pos"]
        move(
            [eef[0], eef[1], clearance],
            grasp_yaw, 1.0, 45, 0.012
        )

        state = sdk.get_state()
        lifted = state["objects"][juice]["pos"]
        eef = state["eef_pos"]
        held = (
            lifted[2] - before_lift[2] > 0.055
            and hypot(lifted[0] - eef[0], lifted[1] - eef[1]) < 0.07
            and abs(lifted[2] - eef[2]) < 0.14
        )
        if held:
            break
        gripper("open", 8)

    if not held:
        return {"status": "failed", "reason": "Could not verify a grasp"}

    state = sdk.get_state()
    obj = state["objects"][juice]["pos"]
    eef = state["eef_pos"]
    offset = [obj[i] - eef[i] for i in range(3)]
    target = destination(state, basket)
    carry_height = max(eef[2], target[2] + 0.22 - offset[2])
    above = [
        target[0] - offset[0],
        target[1] - offset[1],
        carry_height,
    ]

    if not move(above, grasp_yaw, 1.0, 60, 0.012):
        return {"status": "failed", "reason": "Could not reach basket"}

    state = sdk.get_state()
    obj = state["objects"][juice]["pos"]
    eef = state["eef_pos"]
    if abs((obj[2] - eef[2]) - offset[2]) > 0.06:
        return {"status": "failed", "reason": "Object slipped during transport"}

    target = destination(state, basket)
    release = [
        eef[0] + target[0] - obj[0],
        eef[1] + target[1] - obj[1],
        eef[2] + target[2] + 0.075 - obj[2],
    ]
    if remaining() < 35:
        return {"status": "failed", "reason": "Insufficient release budget"}

    if not move(
        release, grasp_yaw, 1.0, min(40, remaining() - 28), 0.01
    ):
        return {"status": "failed", "reason": "Could not reach release pose"}

    state = sdk.get_state()
    obj = state["objects"][juice]["pos"]
    target = destination(state, basket)
    if hypot(obj[0] - target[0], obj[1] - target[1]) > 0.055:
        return {"status": "failed", "reason": "Object not centered over basket"}

    gripper("open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.14],
        grasp_yaw, -1.0, min(25, max(1, remaining() - 12))
    )
    settle(-1.0, 18)

    state = sdk.get_state()
    obj = state["objects"][juice]["pos"]
    target = destination(state, basket)
    placed = (
        hypot(obj[0] - target[0], obj[1] - target[1]) < 0.075
        and target[2] - 0.07 < obj[2] < target[2] + 0.10
        and state["gripper_width"] > 0.04
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Released in basket" if placed else "Placement not verified",
        "steps_used": state["steps_used"],
    }
