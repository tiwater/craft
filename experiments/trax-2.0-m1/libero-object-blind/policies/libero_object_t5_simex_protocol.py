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

    def select(names, stem):
        if stem + "_1" in names:
            return stem + "_1"
        if stem in names:
            return stem
        matches = [name for name in names if name.startswith(stem + "_")]
        if len(matches) == 1:
            return matches[0]
        return None

    def basket_center(state, basket):
        sites = state["sites"]
        for suffix in ["_contain_region", "_basket_contain_region"]:
            name = basket + suffix
            if name in sites:
                return list(sites[name]), name
        matches = [
            name for name in sites
            if name.startswith(basket + "_") and "contain" in name
        ]
        if len(matches) == 1:
            return list(sites[matches[0]]), matches[0]
        if basket in state["objects"]:
            pos = state["objects"][basket]["pos"]
        else:
            pos = state["fixtures"][basket]["pos"]
        return [pos[0], pos[1], pos[2] + 0.075], "pose_offset"

    state = sdk.get_state()
    tomato = select(state["objects"], "tomato_sauce")
    basket = select(state["objects"], "basket")
    if basket is None:
        basket = select(state["fixtures"], "basket")
    if tomato is None or basket is None:
        return {
            "status": "failed",
            "reason": "Required object identities unavailable or ambiguous",
        }

    center, region = basket_center(state, basket)
    grounding = {
        "object": tomato,
        "object_pos": list(state["objects"][tomato]["pos"]),
        "basket": basket,
        "region": region,
        "target": list(center),
    }
    held = False
    offset = [0.0, 0.0, 0.0]
    grasp_yaw = state["objects"][tomato]["yaw"]
    descent_failed = False
    attempts = []

    for attempt in range(4):
        if remaining() < 175:
            break
        state = sdk.get_state()
        obj = list(state["objects"][tomato]["pos"])
        center, region = basket_center(state, basket)
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
        if not move([obj[0], obj[1], clearance], grasp_yaw, "open", 40):
            continue

        state = sdk.get_state()
        obj = list(state["objects"][tomato]["pos"])
        grasp_height = obj[2] + (0.008 if attempt != 2 else -0.005)
        if descent_failed:
            if not move(
                [obj[0], obj[1], obj[2] + 0.075],
                grasp_yaw, "open", 20, 0.008
            ):
                continue
            state = sdk.get_state()
            obj = list(state["objects"][tomato]["pos"])
            grasp_height = obj[2] + 0.018

        reached = move(
            [obj[0], obj[1], grasp_height],
            grasp_yaw, "open", 38, 0.008
        )
        if not reached:
            descent_failed = True
            state = sdk.get_state()
            obj = list(state["objects"][tomato]["pos"])
            eef = list(state["eef_pos"])
            unsafe = (
                hypot(eef[0] - obj[0], eef[1] - obj[1]) > 0.012
                or eef[2] - obj[2] > 0.04
                or eef[2] < obj[2] - 0.015
            )
            if unsafe and remaining() >= 235:
                recovery_z = max(eef[2] + 0.045, obj[2] + 0.10)
                move(
                    [eef[0], eef[1], recovery_z],
                    grasp_yaw, "open", 16
                )
                state = sdk.get_state()
                obj = list(state["objects"][tomato]["pos"])
                move(
                    [obj[0], obj[1], recovery_z],
                    grasp_yaw, "open", 20, 0.005
                )
                state = sdk.get_state()
                obj = list(state["objects"][tomato]["pos"])
                move(
                    [obj[0], obj[1], obj[2] + 0.025],
                    grasp_yaw, "open", 25, 0.006
                )
                state = sdk.get_state()
                obj = list(state["objects"][tomato]["pos"])
                eef = list(state["eef_pos"])
                unsafe = (
                    hypot(eef[0] - obj[0], eef[1] - obj[1]) > 0.012
                    or eef[2] - obj[2] > 0.04
                    or eef[2] < obj[2] - 0.015
                )
            if unsafe:
                attempts.append({
                    "attempt": attempt,
                    "object_pos": list(obj),
                    "eef_pos": list(eef),
                    "reason": "Descent geometry rejected",
                })
                continue

        before = list(sdk.get_state()["objects"][tomato]["pos"])
        gripper("close", 14)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move(
            [eef[0], eef[1], max(eef[2] + 0.12, center[2] + 0.16)],
            grasp_yaw, "close", 35
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
        attempts.append({
            "attempt": attempt,
            "object_pos": list(lifted),
            "eef_pos": list(eef),
            "lift": lifted[2] - before[2],
            "gripper_width": state["gripper_width"],
        })
        gripper("open", 8)

    if not held:
        return {
            "status": "failed",
            "reason": "Could not verify a lifted grasp",
            "grounding": grounding,
            "attempts": attempts,
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    center, region = basket_center(state, basket)
    eef = state["eef_pos"]
    transit_z = max(eef[2], center[2] + 0.18 - offset[2])
    if not move([eef[0], eef[1], transit_z], grasp_yaw, "close", 25):
        return {"status": "failed", "reason": "Could not reach safe transfer height"}

    target = [center[0] - offset[0], center[1] - offset[1], transit_z]
    reached = move(target, grasp_yaw, "close", 60)
    state = sdk.get_state()
    obj = state["objects"][tomato]["pos"]
    eef = state["eef_pos"]
    measured_offset = [obj[i] - eef[i] for i in range(3)]
    if (
        abs(measured_offset[2] - offset[2]) > 0.045
        or hypot(
            measured_offset[0] - offset[0],
            measured_offset[1] - offset[1]
        ) > 0.045
    ):
        return {"status": "failed", "reason": "Object slipped during transfer"}

    center, region = basket_center(state, basket)
    alignment_error = hypot(obj[0] - center[0], obj[1] - center[1])
    if not reached or alignment_error > 0.012:
        if remaining() < 85:
            return {"status": "failed", "reason": "Insufficient steps for alignment"}
        target = [
            eef[0] + center[0] - obj[0],
            eef[1] + center[1] - obj[1],
            max(eef[2], center[2] + 0.18 - measured_offset[2]),
        ]
        move(target, grasp_yaw, "close", 25, 0.006)

    state = sdk.get_state()
    center, region = basket_center(state, basket)
    obj = state["objects"][tomato]["pos"]
    eef = state["eef_pos"]
    if hypot(obj[0] - center[0], obj[1] - center[1]) > 0.025:
        return {"status": "failed", "reason": "Basket alignment could not be verified"}
    measured_offset = [obj[i] - eef[i] for i in range(3)]
    release = [
        center[0] - measured_offset[0],
        center[1] - measured_offset[1],
        center[2] + 0.035 - measured_offset[2],
    ]
    if remaining() < 70:
        return {"status": "failed", "reason": "Insufficient steps for controlled release"}

    move(release, grasp_yaw, "close", 35, 0.008)
    state = sdk.get_state()
    center, region = basket_center(state, basket)
    obj = state["objects"][tomato]["pos"]
    eef = state["eef_pos"]
    horizontal_error = hypot(obj[0] - center[0], obj[1] - center[1])
    if horizontal_error > 0.012 and remaining() >= 60:
        release = [
            eef[0] + center[0] - obj[0],
            eef[1] + center[1] - obj[1],
            eef[2],
        ]
        move(release, grasp_yaw, "close", 15, 0.006)
        state = sdk.get_state()
        center, region = basket_center(state, basket)
        obj = state["objects"][tomato]["pos"]

    if (
        hypot(obj[0] - center[0], obj[1] - center[1]) > 0.025
        or obj[2] < center[2] - 0.04
        or obj[2] > center[2] + 0.065
    ):
        return {"status": "failed", "reason": "Unsafe release geometry"}

    gripper("open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.15], grasp_yaw, "open", 25)
    gripper("open", 15)

    state = sdk.get_state()
    obj = state["objects"][tomato]["pos"]
    center, region = basket_center(state, basket)
    if basket in state["objects"]:
        basket_yaw = state["objects"][basket]["yaw"]
    else:
        basket_yaw = state["fixtures"][basket]["yaw"]
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
        "grounding": grounding,
        "final_region": region,
        "final_target": list(center),
        "release_target": release,
        "final_object_pos": list(obj),
        "attempts": attempts,
        "steps_used": state["steps_used"],
    }
