def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(450, state["max_steps"]) - state["steps_used"])


def distance_xyz(a, b):
    return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def move(sdk, pos, yaw, grip, steps=35, tol=0.012):
    budget = remaining_steps(sdk)
    if budget <= 0:
        return False
    result = sdk.move_to(
        pos, yaw=yaw, gripper=grip,
        max_steps=min(steps, budget), tol=tol
    )
    return result["reached"]


def actuate(sdk, action, steps=12):
    budget = remaining_steps(sdk)
    if budget > 0:
        sdk.set_gripper(action, steps=min(steps, budget))


def angle_distance(a, b):
    return abs(atan2(sin(a - b), cos(a - b)))


def free_space_angle(state, target, previous):
    bowl = state["objects"][target]["pos"]
    neighbors = []
    for name in state["objects"]:
        if name != target:
            neighbors.append(state["objects"][name]["pos"])
    for name in state["fixtures"]:
        neighbors.append(state["fixtures"][name]["pos"])

    # Positions are exposed, but collision extents are not. Use the
    # minimum planar distance to every neighboring object/fixture.
    best_angle = None
    best_clearance = -1.0
    for index in range(180):
        angle = 2.0 * pi * index / 180.0
        if any(angle_distance(angle, old) < pi / 3.0 for old in previous):
            continue
        x = bowl[0] + 0.04 * cos(angle)
        y = bowl[1] + 0.04 * sin(angle)
        clearance = 10.0
        for position in neighbors:
            clearance = min(clearance, hypot(x - position[0], y - position[1]))
        if clearance > best_clearance:
            best_clearance = clearance
            best_angle = angle
    return best_angle


def transit_height(state, target):
    height = state["objects"][target]["pos"][2] + 0.15
    for name in state["objects"]:
        height = max(height, state["objects"][name]["pos"][2] + 0.12)
    for name in state["fixtures"]:
        height = max(height, state["fixtures"][name]["pos"][2] + 0.12)
    for name in state["sites"]:
        if "top_side" in name:
            height = max(height, state["sites"][name][2] + 0.12)
    return height


def run_policy(sdk):
    state = sdk.get_state()
    bowls = [name for name in state["objects"] if "black_bowl" in name]
    plates = [name for name in state["objects"] if "plate" in name]
    cabinets = [name for name in state["fixtures"] if "wooden_cabinet" in name]
    if not bowls or not plates or not cabinets:
        return {"status": "failed", "reason": "Required entities missing"}

    cabinet = cabinets[0]
    region_name = cabinet + "_top_region"
    if region_name not in state["sites"]:
        return {"status": "failed", "reason": "Top drawer region missing"}
    region = list(state["sites"][region_name])
    target = bowls[0]
    for name in bowls:
        if distance_xyz(state["objects"][name]["pos"], region) < distance_xyz(
            state["objects"][target]["pos"], region
        ):
            target = name
    plate = plates[0]

    fixture = state["fixtures"][cabinet]
    front = [sin(fixture["yaw"]), -cos(fixture["yaw"])]
    handle_yaw = atan2(front[1], front[0])
    extension = sum(
        (region[i] - fixture["pos"][i]) * front[i] for i in range(2)
    )
    actuate(sdk, "open", 6)

    if extension < 0.09:
        handle = None
        for name in state["sites"]:
            if cabinet in name and "top" in name and "handle" in name:
                handle = list(state["sites"][name])
                break
        if handle is None:
            handle = [
                region[0] + 0.095 * front[0],
                region[1] + 0.095 * front[1],
                region[2] - 0.035
            ]
        high = max(transit_height(state, target), state["eef_pos"][2])
        approach = [
            handle[0] + 0.065 * front[0],
            handle[1] + 0.065 * front[1],
            high
        ]
        move(sdk, approach, handle_yaw, "open", 30)
        approach[2] = handle[2]
        move(sdk, approach, handle_yaw, "open", 22)
        move(sdk, handle, handle_yaw, "open", 22, 0.01)
        if distance_xyz(sdk.get_state()["eef_pos"], handle) > 0.03:
            return {"status": "failed", "reason": "Drawer handle inaccessible"}
        actuate(sdk, "close", 10)
        pull = min(0.20, max(0.12, 0.17 - extension))
        for fraction in [0.5, 1.0]:
            move(sdk, [
                handle[0] + pull * fraction * front[0],
                handle[1] + pull * fraction * front[1],
                handle[2]
            ], handle_yaw, "close", 20)
        actuate(sdk, "open", 6)
        state = sdk.get_state()
        retreat = [
            state["eef_pos"][0] + 0.035 * front[0],
            state["eef_pos"][1] + 0.035 * front[1],
            state["eef_pos"][2]
        ]
        move(sdk, retreat, handle_yaw, "open", 12)
        retreat[2] = high
        move(sdk, retreat, handle_yaw, "open", 22)
        state = sdk.get_state()
        new_region = state["sites"][region_name]
        opened = sum((new_region[i] - region[i]) * front[i] for i in range(2))
        if opened < 0.045:
            return {"status": "failed", "reason": "Drawer opening not verified"}

    previous = []
    held = False
    baseline_z = None
    grasp_yaw = 0.0
    for attempt in range(3):
        if remaining_steps(sdk) < 165:
            break
        state = sdk.get_state()
        angle = free_space_angle(state, target, previous)
        if angle is None:
            break
        previous.append(angle)

        # Task-specific probe: radius .04, Z offset .02, jaws perpendicular
        # to the WORLD radial direction. Object yaw is never consulted.
        grasp_yaw = atan2(sin(angle + pi / 2.0), cos(angle + pi / 2.0))
        bowl = list(state["objects"][target]["pos"])
        high = transit_height(state, target)
        actuate(sdk, "open", 5)
        eef = sdk.get_state()["eef_pos"]
        move(sdk, [eef[0], eef[1], high], grasp_yaw, "open", 20)
        hover = [
            bowl[0] + 0.04 * cos(angle),
            bowl[1] + 0.04 * sin(angle),
            high
        ]
        move(sdk, hover, grasp_yaw, "open", 28)

        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        baseline_z = bowl[2]
        grasp = [
            bowl[0] + 0.04 * cos(angle),
            bowl[1] + 0.04 * sin(angle),
            bowl[2] + 0.02
        ]
        move(sdk, grasp, grasp_yaw, "open", 32, 0.009)
        state = sdk.get_state()
        if distance_xyz(state["eef_pos"], grasp) > 0.018:
            move(sdk, grasp, grasp_yaw, "open", 10, 0.012)

        # A false reached flag must not prevent contact actuation.
        state = sdk.get_state()
        if distance_xyz(state["eef_pos"], grasp) > 0.035:
            continue
        actuate(sdk, "close", 14)
        eef = list(sdk.get_state()["eef_pos"])
        move(sdk, [eef[0], eef[1], eef[2] + 0.085],
             grasp_yaw, "close", 26, 0.01)
        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        if bowl[2] - baseline_z >= 0.03 and distance_xyz(
            bowl, state["eef_pos"]
        ) < 0.12:
            held = True
            break

        # Lower a partial lift before releasing and selecting another angle.
        if bowl[2] - baseline_z > 0.01 and distance_xyz(
            bowl, state["eef_pos"]
        ) < 0.12:
            eef = state["eef_pos"]
            move(sdk, [
                eef[0], eef[1],
                eef[2] - (bowl[2] - baseline_z)
            ], grasp_yaw, "close", 15)
        actuate(sdk, "open", 6)

    if not held:
        return {"status": "failed", "reason": "Bowl lift not verified",
                "target": target}

    state = sdk.get_state()
    offset = [
        state["objects"][target]["pos"][i] - state["eef_pos"][i]
        for i in range(3)
    ]
    high = max(transit_height(state, target), state["eef_pos"][2])
    eef = state["eef_pos"]
    move(sdk, [eef[0], eef[1], high], grasp_yaw, "close", 28)
    state = sdk.get_state()
    if state["eef_pos"][2] < high - 0.025:
        return {"status": "failed", "reason": "Cabinet clearance not reached"}
    bowl = state["objects"][target]["pos"]
    if bowl[2] - baseline_z < 0.03 or distance_xyz(
        bowl, state["eef_pos"]
    ) >= 0.12:
        return {"status": "failed", "reason": "Bowl slipped during extraction"}

    offset = [bowl[i] - state["eef_pos"][i] for i in range(3)]
    plate_pos = state["objects"][plate]["pos"]
    above = [plate_pos[0] - offset[0], plate_pos[1] - offset[1], high]
    move(sdk, above, grasp_yaw, "close", 36, 0.01)
    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    if distance_xyz(bowl, state["eef_pos"]) >= 0.12:
        return {"status": "failed", "reason": "Bowl slipped during transport"}

    offset = [bowl[i] - state["eef_pos"][i] for i in range(3)]
    placement = [
        plate_pos[0] - offset[0],
        plate_pos[1] - offset[1],
        plate_pos[2] + 0.035 - offset[2]
    ]
    move(sdk, placement, grasp_yaw, "close", 32, 0.008)
    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    if (hypot(bowl[0] - plate_pos[0], bowl[1] - plate_pos[1]) > 0.04
            or bowl[2] - plate_pos[2] > 0.065):
        return {"status": "failed", "reason": "Placement pose not reached"}

    actuate(sdk, "open", 12)
    eef = list(sdk.get_state()["eef_pos"])
    move(sdk, [eef[0], eef[1], eef[2] + 0.12],
         grasp_yaw, "open", 22)
    actuate(sdk, "open", 6)
    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    if (hypot(bowl[0] - plate_pos[0], bowl[1] - plate_pos[1]) < 0.055
            and -0.005 < bowl[2] - plate_pos[2] < 0.075
            and distance_xyz(bowl, state["eef_pos"]) > 0.08):
        return {"status": "done", "target": target}
    return {"status": "failed", "reason": "Final placement not verified",
            "target": target}
