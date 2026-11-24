def policy_move(sdk, pos, yaw, grip, limit=45, tol=0.012):
    state = sdk.get_state()
    remaining = min(450, state["max_steps"]) - state["steps_used"]
    if remaining < 2:
        return False
    result = sdk.move_to(
        pos, yaw=yaw, gripper=grip,
        max_steps=min(limit, remaining), tol=tol
    )
    return result["reached"]


def policy_gripper(sdk, action, steps=12):
    state = sdk.get_state()
    remaining = min(450, state["max_steps"]) - state["steps_used"]
    if remaining < 1:
        return {}
    return sdk.set_gripper(action, steps=min(steps, remaining))


def policy_distance(a, b):
    return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def run_policy(sdk):
    state = sdk.get_state()
    objects = state["objects"]
    sites = state["sites"]
    fixtures = state["fixtures"]

    bowls = [name for name in objects if "black_bowl" in name]
    plates = [name for name in objects if "plate" in name]
    cabinets = [name for name in fixtures if "wooden_cabinet" in name]
    if not bowls or not plates or not cabinets:
        return {"status": "failed", "reason": "Required scene entities missing"}

    cabinet = cabinets[0]
    cabinet_pos = fixtures[cabinet]["pos"]
    cabinet_yaw = fixtures[cabinet]["yaw"]
    region_name = cabinet + "_top_region"
    if region_name not in sites:
        return {"status": "failed", "reason": "Top drawer region missing"}

    region = sites[region_name]
    target = bowls[0]
    best = policy_distance(objects[target]["pos"], region)
    for name in bowls[1:]:
        distance = policy_distance(objects[name]["pos"], region)
        if distance < best:
            target = name
            best = distance

    plate = plates[0]
    front = [sin(cabinet_yaw), -cos(cabinet_yaw)]
    displacement = [
        region[0] - cabinet_pos[0],
        region[1] - cabinet_pos[1]
    ]
    extension = displacement[0] * front[0] + displacement[1] * front[1]
    yaw = atan2(front[1], front[0])
    top = sites.get(cabinet + "_top_side", region)
    safe_z = max(top[2] + 0.13, state["eef_pos"][2],
                 objects[target]["pos"][2] + 0.16)

    policy_gripper(sdk, "open", 8)

    # A retracted drawer must be pulled out before approaching its contents.
    if extension < 0.09:
        handle = None
        for name in sites:
            if cabinet in name and "top" in name and "handle" in name:
                handle = sites[name]
                break

        if handle is None:
            handle = [
                region[0] + front[0] * 0.095,
                region[1] + front[1] * 0.095,
                region[2] - 0.035
            ]

        approach = [
            handle[0] + front[0] * 0.065,
            handle[1] + front[1] * 0.065,
            safe_z
        ]
        if not policy_move(sdk, approach, yaw, "open", 40):
            return {"status": "failed", "reason": "Drawer approach blocked"}
        approach[2] = handle[2]
        policy_move(sdk, approach, yaw, "open", 30)
        if not policy_move(sdk, handle, yaw, "open", 25):
            return {"status": "failed", "reason": "Drawer handle inaccessible"}
        policy_gripper(sdk, "close", 10)

        pull = min(0.20, max(0.12, 0.17 - extension))
        for fraction in [0.5, 1.0]:
            destination = [
                handle[0] + front[0] * pull * fraction,
                handle[1] + front[1] * pull * fraction,
                handle[2]
            ]
            policy_move(sdk, destination, yaw, "close", 25)

        policy_gripper(sdk, "open", 8)
        state = sdk.get_state()
        retreat = list(state["eef_pos"])
        retreat[0] += front[0] * 0.035
        retreat[1] += front[1] * 0.035
        policy_move(sdk, retreat, yaw, "open", 15)
        retreat[2] = safe_z
        policy_move(sdk, retreat, yaw, "open", 25)

        new_region = sdk.get_state()["sites"][region_name]
        if policy_distance(new_region, region) < 0.045:
            return {"status": "failed", "reason": "Drawer did not open"}

    held = False
    offset = None
    for attempt in range(3):
        state = sdk.get_state()
        remaining = min(450, state["max_steps"]) - state["steps_used"]
        if remaining < 125:
            break

        bowl = list(state["objects"][target]["pos"])
        grasp_yaw = yaw if attempt < 2 else yaw + pi / 2
        side = 0.0 if attempt == 0 else 0.027
        grasp = [
            bowl[0] - front[1] * side,
            bowl[1] + front[0] * side,
            bowl[2] + (0.018 if attempt == 0 else 0.009)
        ]
        hover = [grasp[0], grasp[1], safe_z]

        policy_gripper(sdk, "open", 6)
        if not policy_move(sdk, hover, grasp_yaw, "open", 40):
            continue
        if not policy_move(sdk, grasp, grasp_yaw, "open", 35, 0.007):
            continue

        policy_gripper(sdk, "close", 14)
        before = list(sdk.get_state()["objects"][target]["pos"])
        lift = [grasp[0], grasp[1], grasp[2] + 0.085]
        policy_move(sdk, lift, grasp_yaw, "close", 30)

        state = sdk.get_state()
        after = state["objects"][target]["pos"]
        eef = state["eef_pos"]
        if after[2] - before[2] > 0.045 and policy_distance(after, eef) < 0.12:
            offset = [after[i] - eef[i] for i in range(3)]
            yaw = grasp_yaw
            held = True
            break

    if not held:
        return {"status": "failed", "reason": "Bowl grasp not verified",
                "target": target}

    state = sdk.get_state()
    eef = state["eef_pos"]
    clearance = max(safe_z, eef[2], state["objects"][plate]["pos"][2] + 0.18)
    if not policy_move(sdk, [eef[0], eef[1], clearance], yaw, "close", 30):
        return {"status": "failed", "reason": "Could not clear cabinet"}

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    if policy_distance(bowl, state["eef_pos"]) > 0.13:
        return {"status": "failed", "reason": "Bowl slipped during lift"}

    plate_pos = state["objects"][plate]["pos"]
    above_plate = [
        plate_pos[0] - offset[0],
        plate_pos[1] - offset[1],
        clearance
    ]
    if not policy_move(sdk, above_plate, yaw, "close", 45):
        return {"status": "failed", "reason": "Plate approach incomplete"}

    plate_pos = sdk.get_state()["objects"][plate]["pos"]
    placement = [
        plate_pos[0] - offset[0],
        plate_pos[1] - offset[1],
        plate_pos[2] + 0.035 - offset[2]
    ]
    if not policy_move(sdk, placement, yaw, "close", 35, 0.008):
        return {"status": "failed", "reason": "Placement height not reached"}

    policy_gripper(sdk, "open", 14)
    retreat = list(sdk.get_state()["eef_pos"])
    retreat[2] += 0.12
    policy_move(sdk, retreat, yaw, "open", 25)
    policy_gripper(sdk, "open", 8)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    centered = hypot(bowl[0] - plate_pos[0], bowl[1] - plate_pos[1]) < 0.055
    supported_height = -0.005 < bowl[2] - plate_pos[2] < 0.075
    released = policy_distance(bowl, state["eef_pos"]) > 0.08
    if centered and supported_height and released:
        return {"status": "done", "target": target}
    return {"status": "failed", "reason": "Final placement not verified",
            "target": target}
