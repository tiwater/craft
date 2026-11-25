def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, action, steps, tol=0.008):
        budget = remaining()
        if budget <= 0:
            return False
        result = sdk.move_to(
            pos, yaw=yaw, gripper=action,
            max_steps=min(steps, budget), tol=tol
        )
        return result["reached"]

    def grip(action, steps):
        budget = remaining()
        if budget > 0:
            sdk.set_gripper(action, steps=min(steps, budget))

    def nearest_yaw(angle, reference):
        while angle - reference > pi / 2:
            angle -= pi
        while angle - reference < -pi / 2:
            angle += pi
        return angle

    def distance(a, b):
        return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    state = sdk.get_state()
    cheese = None
    basket = None
    basket_group = None
    for name in state["objects"]:
        if "cream_cheese" in name:
            cheese = name
            break
    for group in ["objects", "fixtures"]:
        for name in state[group]:
            if "basket" in name and basket is None:
                basket = name
                basket_group = group

    if cheese is None or basket is None:
        return {"status": "failed", "reason": "Required entities unavailable"}

    region = None
    for name in state["sites"]:
        if basket in name and "contain" in name:
            region = name
            break
    if region is None:
        for name in state["sites"]:
            if "basket" in name and "contain" in name:
                region = name
                break

    def destination(state):
        if region is not None:
            return list(state["sites"][region])
        pos = state[basket_group][basket]["pos"]
        return [pos[0], pos[1], pos[2] + 0.04]

    # The previous grasps produced no object motion. Probe near the
    # object's center instead of assuming a large fingertip offset.
    trials = [
        [0.0, 0.008],
        [pi / 2, 0.008],
        [0.0, 0.028],
        [pi / 2, 0.028]
    ]
    held = False
    offset = None
    yaw = state["eef_yaw"]

    for trial in trials:
        if remaining() < 155:
            break

        state = sdk.get_state()
        pos = list(state["objects"][cheese]["pos"])
        eef = list(state["eef_pos"])
        yaw = nearest_yaw(
            state["objects"][cheese]["yaw"] + trial[0],
            state["eef_yaw"]
        )
        approach_z = max(pos[2] + 0.12, eef[2])
        grip("open", 5)

        if eef[2] < pos[2] + 0.09:
            move(
                [eef[0], eef[1], approach_z],
                state["eef_yaw"], "open", 15
            )

        move([pos[0], pos[1], approach_z], yaw, "open", 25)
        state = sdk.get_state()
        pos = list(state["objects"][cheese]["pos"])
        eef = list(state["eef_pos"])
        if hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.018:
            continue

        move(
            [pos[0], pos[1], pos[2] + 0.045],
            yaw, "open", 15, 0.005
        )
        state = sdk.get_state()
        pos = list(state["objects"][cheese]["pos"])
        move(
            [pos[0], pos[1], pos[2] + trial[1]],
            yaw, "open", 18, 0.004
        )

        state = sdk.get_state()
        pos = list(state["objects"][cheese]["pos"])
        eef = list(state["eef_pos"])
        if hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.022:
            continue
        if eef[2] - pos[2] > trial[1] + 0.025:
            continue

        baseline = list(pos)
        grip("close", 10)
        state = sdk.get_state()
        start_eef = list(state["eef_pos"])
        start_obj = list(state["objects"][cheese]["pos"])
        move(
            [start_eef[0], start_eef[1], start_eef[2] + 0.09],
            yaw, "close", 20, 0.006
        )

        state = sdk.get_state()
        pos = list(state["objects"][cheese]["pos"])
        eef = list(state["eef_pos"])
        hand_rise = eef[2] - start_eef[2]
        object_rise = pos[2] - start_obj[2]
        held = (
            pos[2] - baseline[2] > 0.035
            and hand_rise > 0.04
            and abs(object_rise - hand_rise) < 0.025
            and hypot(pos[0] - eef[0], pos[1] - eef[1]) < 0.06
        )
        if held:
            offset = [pos[i] - eef[i] for i in range(3)]
            break

    if not held:
        grip("open", 5)
        return {"status": "failed", "reason": "No verified grasp"}

    state = sdk.get_state()
    target = destination(state)
    basket_pos = state[basket_group][basket]["pos"]
    object_clearance = max(target[2] + 0.16, basket_pos[2] + 0.22)
    for name in state["objects"]:
        if name != cheese and name != basket:
            object_clearance = max(
                object_clearance, state["objects"][name]["pos"][2] + 0.10
            )

    eef = list(state["eef_pos"])
    transit_z = max(eef[2], object_clearance - offset[2])
    move([eef[0], eef[1], transit_z], yaw, "close", 25)

    state = sdk.get_state()
    if state["eef_pos"][2] < transit_z - 0.025:
        return {"status": "failed", "reason": "Transit clearance unavailable"}

    target = destination(state)
    move(
        [target[0] - offset[0], target[1] - offset[1], transit_z],
        yaw, "close", 35, 0.007
    )

    for correction in range(2):
        state = sdk.get_state()
        pos = list(state["objects"][cheese]["pos"])
        eef = list(state["eef_pos"])
        live_offset = [pos[i] - eef[i] for i in range(3)]
        if distance(live_offset, offset) > 0.04:
            return {"status": "failed", "reason": "Grasp lost during transport"}

        target = destination(state)
        if hypot(pos[0] - target[0], pos[1] - target[1]) < 0.012:
            break
        if remaining() < 55:
            break
        move(
            [
                eef[0] + target[0] - pos[0],
                eef[1] + target[1] - pos[1],
                eef[2]
            ],
            yaw, "close", 12, 0.005
        )

    state = sdk.get_state()
    target = destination(state)
    basket_pos = state[basket_group][basket]["pos"]
    pos = list(state["objects"][cheese]["pos"])
    eef = list(state["eef_pos"])
    if hypot(pos[0] - target[0], pos[1] - target[1]) > 0.025:
        return {"status": "failed", "reason": "Basket alignment unavailable"}
    if remaining() < 35:
        return {"status": "failed", "reason": "Insufficient release budget"}

    live_offset = [pos[i] - eef[i] for i in range(3)]
    release_z = max(target[2] + 0.055, basket_pos[2] + 0.11)
    move(
        [
            target[0] - live_offset[0],
            target[1] - live_offset[1],
            release_z - live_offset[2]
        ],
        yaw, "close", min(22, remaining() - 30), 0.007
    )

    state = sdk.get_state()
    target = destination(state)
    pos = list(state["objects"][cheese]["pos"])
    eef = list(state["eef_pos"])
    live_offset = [pos[i] - eef[i] for i in range(3)]
    if distance(live_offset, offset) > 0.045:
        return {"status": "failed", "reason": "Grasp lost before release"}
    if hypot(pos[0] - target[0], pos[1] - target[1]) > 0.025:
        return {"status": "failed", "reason": "Release alignment failed"}

    grip("open", 10)
    state = sdk.get_state()
    eef = list(state["eef_pos"])
    move(
        [eef[0], eef[1], eef[2] + 0.14],
        yaw, "open", min(18, max(1, remaining() - 8))
    )
    grip("open", min(15, remaining()))

    state = sdk.get_state()
    pos = list(state["objects"][cheese]["pos"])
    target = destination(state)
    basket_pos = state[basket_group][basket]["pos"]
    inside = (
        hypot(pos[0] - target[0], pos[1] - target[1]) < 0.055
        and basket_pos[2] - 0.025 < pos[2]
        and pos[2] < max(target[2] + 0.065, basket_pos[2] + 0.10)
        and state["gripper_width"] > 0.045
        and distance(pos, state["eef_pos"]) > 0.09
    )
    return {
        "status": "done" if inside else "failed",
        "reason": "Placement verified" if inside else "Placement not verified"
    }
