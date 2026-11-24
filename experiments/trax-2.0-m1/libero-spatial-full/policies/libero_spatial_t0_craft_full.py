def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(450, state["max_steps"]) - state["steps_used"])


def move_pose(sdk, position, yaw, grip, steps, tolerance=0.01):
    available = remaining_steps(sdk)
    if available <= 0:
        return False
    result = sdk.move_to(
        position,
        yaw=yaw,
        gripper=grip,
        max_steps=min(steps, available),
        tol=tolerance,
    )
    return result["reached"]


def actuate(sdk, action, steps):
    available = remaining_steps(sdk)
    if available <= 0:
        return False
    sdk.set_gripper(action, steps=min(steps, available))
    return True


def distance3(a, b):
    return sqrt(sum([(a[i] - b[i]) ** 2 for i in range(3)]))


def free_space_angle(state, target, radius, attempted):
    bowl = state["objects"][target]["pos"]
    obstacles = []
    for name in state["objects"]:
        if name != target:
            obstacles.append(state["objects"][name]["pos"])
    for name in state["fixtures"]:
        obstacles.append(state["fixtures"][name]["pos"])
        for site in state["sites"]:
            if name in site:
                obstacles.append(state["sites"][site])

    best_angle = None
    best_clearance = -1.0
    for index in range(180):
        angle = 2.0 * pi * index / 180.0
        distinct = True
        for previous in attempted:
            separation = abs(atan2(sin(angle - previous), cos(angle - previous)))
            if separation < pi / 4.0:
                distinct = False
        if distinct:
            x = bowl[0] + radius * cos(angle)
            y = bowl[1] + radius * sin(angle)
            clearance = 1000.0
            for obstacle in obstacles:
                clearance = min(
                    clearance, hypot(x - obstacle[0], y - obstacle[1])
                )
            if clearance > best_clearance:
                best_clearance = clearance
                best_angle = angle
    return best_angle


def aligned_yaw(angle, current):
    # The successful probe used jaw_rel_rad=pi/2.
    yaw = angle + pi / 2.0
    while yaw - current > pi / 2.0:
        yaw -= pi
    while yaw - current < -pi / 2.0:
        yaw += pi
    return yaw


def run_policy(sdk):
    state = sdk.get_state()
    objects = state["objects"]
    bowls = [name for name in objects if "black_bowl" in name]
    plates = [name for name in objects if "plate" in name]
    ramekins = [name for name in objects if "ramekin" in name]
    if not bowls or not plates or not ramekins:
        return {"status": "failed", "reason": "Required objects are missing"}

    plate_name = plates[0]
    plate = objects[plate_name]["pos"]
    ramekin = objects[ramekins[0]]["pos"]
    midpoint = [(plate[i] + ramekin[i]) * 0.5 for i in range(2)]
    region = None
    for name in state["sites"]:
        if "between_plate_ramekin" in name:
            region = state["sites"][name]
            break

    target = bowls[0]
    best_score = None
    for name in bowls:
        position = objects[name]["pos"]
        score = sum([(position[i] - midpoint[i]) ** 2 for i in range(2)])
        if region is not None:
            score += sum([(position[i] - region[i]) ** 2 for i in range(2)])
        if best_score is None or score < best_score:
            best_score = score
            target = name

    attempted = []
    held = False
    baseline_z = 0.0
    grasp_yaw = state["eef_yaw"]

    for attempt in range(3):
        if remaining_steps(sdk) < 205:
            break

        state = sdk.get_state()
        # Keep contact geometry at the best measured successful probe.
        radius = 0.04
        z_offset = 0.02
        angle = free_space_angle(state, target, radius, attempted)
        if angle is None:
            break
        attempted.append(angle)
        grasp_yaw = aligned_yaw(angle, state["eef_yaw"])
        bowl = list(state["objects"][target]["pos"])
        baseline_z = bowl[2]
        eef = state["eef_pos"]
        approach_z = max(bowl[2] + 0.15, eef[2])

        move_pose(
            sdk, [eef[0], eef[1], approach_z],
            state["eef_yaw"], "open", 18
        )
        actuate(sdk, "open", 6)

        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        contact = [
            bowl[0] + radius * cos(angle),
            bowl[1] + radius * sin(angle),
            bowl[2] + z_offset,
        ]
        move_pose(
            sdk, [contact[0], contact[1], approach_z],
            grasp_yaw, "open", 30
        )

        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        baseline_z = bowl[2]
        contact = [
            bowl[0] + radius * cos(angle),
            bowl[1] + radius * sin(angle),
            bowl[2] + z_offset,
        ]
        move_pose(sdk, contact, grasp_yaw, "open", 32, 0.006)

        state = sdk.get_state()
        if distance3(state["eef_pos"], contact) > 0.018:
            bowl = state["objects"][target]["pos"]
            contact = [
                bowl[0] + radius * cos(angle),
                bowl[1] + radius * sin(angle),
                bowl[2] + z_offset,
            ]
            move_pose(sdk, contact, grasp_yaw, "open", 12, 0.008)

        # A false convergence flag must not bypass gripper actuation.
        actuate(sdk, "close", 12)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move_pose(
            sdk, [eef[0], eef[1], eef[2] + 0.10],
            grasp_yaw, "close", 26, 0.008
        )

        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        eef = state["eef_pos"]
        if bowl[2] - baseline_z >= 0.03 and distance3(bowl, eef) <= 0.10:
            held = True
            break

        actuate(sdk, "open", 6)

    if not held:
        return {"status": "failed", "reason": "No verified bowl lift"}

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    plate = state["objects"][plate_name]["pos"]
    transit_z = max(eef[2], plate[2] + 0.14 - offset[2])

    move_pose(
        sdk, [eef[0], eef[1], transit_z],
        grasp_yaw, "close", 20, 0.01
    )
    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    if bowl[2] - baseline_z < 0.03 or distance3(bowl, eef) > 0.10:
        return {"status": "failed", "reason": "Bowl lost before transport"}

    offset = [bowl[i] - eef[i] for i in range(3)]
    plate = state["objects"][plate_name]["pos"]
    move_pose(
        sdk,
        [plate[0] - offset[0], plate[1] - offset[1], transit_z],
        grasp_yaw, "close", 36, 0.008
    )

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    if bowl[2] - baseline_z < 0.03 or distance3(bowl, eef) > 0.10:
        return {"status": "failed", "reason": "Bowl lost during transport"}

    offset = [bowl[i] - eef[i] for i in range(3)]
    if hypot(bowl[0] - plate[0], bowl[1] - plate[1]) > 0.02:
        move_pose(
            sdk,
            [plate[0] - offset[0], plate[1] - offset[1], eef[2]],
            grasp_yaw, "close", 16, 0.008
        )

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    if hypot(bowl[0] - plate[0], bowl[1] - plate[1]) > 0.035:
        return {"status": "failed", "reason": "Could not center bowl above plate"}

    offset = [bowl[i] - eef[i] for i in range(3)]
    destination = [
        plate[0] - offset[0],
        plate[1] - offset[1],
        plate[2] + 0.025 - offset[2],
    ]
    move_pose(sdk, destination, grasp_yaw, "close", 30, 0.006)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if (
        hypot(bowl[0] - plate[0], bowl[1] - plate[1]) > 0.035
        or bowl[2] > plate[2] + 0.065
    ):
        return {"status": "failed", "reason": "Placement pose was not reached"}

    if not actuate(sdk, "open", 12):
        return {"status": "failed", "reason": "No steps left to release"}

    state = sdk.get_state()
    eef = state["eef_pos"]
    move_pose(
        sdk, [eef[0], eef[1], eef[2] + 0.12],
        grasp_yaw, "open", 22
    )
    actuate(sdk, "open", 8)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if (
        hypot(bowl[0] - plate[0], bowl[1] - plate[1]) < 0.045
        and plate[2] - 0.02 < bowl[2] < plate[2] + 0.06
        and distance3(bowl, state["eef_pos"]) > 0.09
    ):
        return {"status": "done", "object": target}
    return {"status": "failed", "reason": "Released placement could not be verified"}
