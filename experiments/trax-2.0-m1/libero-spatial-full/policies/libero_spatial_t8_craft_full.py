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


def grip(sdk, action, steps):
    budget = remaining_steps(sdk)
    if budget <= 0:
        return False
    sdk.set_gripper(action, steps=min(steps, budget))
    return True


def angle_distance(a, b):
    return abs(atan2(sin(a - b), cos(a - b)))


def free_space_angle(state, bowl_name, radius, tried):
    bowl = state["objects"][bowl_name]["pos"]
    obstacles = []
    for name in state["objects"]:
        if name != bowl_name:
            obstacles.append(state["objects"][name]["pos"])
    for name in state["fixtures"]:
        obstacles.append(state["fixtures"][name]["pos"])
        for site_name in state["sites"]:
            if name in site_name:
                obstacles.append(state["sites"][site_name])

    best_angle = None
    best_clearance = -1.0
    for index in range(128):
        angle = 2.0 * pi * index / 128.0
        if any(angle_distance(angle, previous) < pi / 3 for previous in tried):
            continue
        x = bowl[0] + radius * cos(angle)
        y = bowl[1] + radius * sin(angle)
        clearance = 100.0
        for obstacle in obstacles:
            clearance = min(
                clearance, hypot(x - obstacle[0], y - obstacle[1])
            )
        if clearance > best_clearance:
            best_clearance = clearance
            best_angle = angle
    return best_angle


def segment_distance(point, start, finish):
    dx = finish[0] - start[0]
    dy = finish[1] - start[1]
    length_squared = dx * dx + dy * dy
    fraction = 0.0
    if length_squared > 0.000001:
        fraction = max(0.0, min(
            1.0,
            ((point[0] - start[0]) * dx +
             (point[1] - start[1]) * dy) / length_squared
        ))
    return hypot(
        point[0] - start[0] - fraction * dx,
        point[1] - start[1] - fraction * dy
    )


def run_policy(sdk):
    state = sdk.get_state()
    objects = state["objects"]
    plates = [name for name in objects if "plate" in name]
    bowls = [
        name for name in objects if "black" in name and "bowl" in name
    ]
    if not plates or not bowls:
        return {"status": "failed", "reason": "Required objects unavailable"}

    plate_name = plates[0]
    region = state["sites"].get("main_table_plate_region")
    if region is not None:
        best = 100.0
        for name in plates:
            pos = objects[name]["pos"]
            distance = hypot(pos[0] - region[0], pos[1] - region[1])
            if distance < best:
                best = distance
                plate_name = name

    reference = state["sites"].get(
        "main_table_next_to_plate_region", objects[plate_name]["pos"]
    )
    bowl_name = bowls[0]
    best = 100.0
    for name in bowls:
        pos = objects[name]["pos"]
        distance = hypot(pos[0] - reference[0], pos[1] - reference[1])
        if distance < best:
            best = distance
            bowl_name = name

    tried = []
    held = False
    yaw = state["eef_yaw"]
    lift_origin = objects[bowl_name]["pos"][2]

    for attempt in range(3):
        if remaining_steps(sdk) < 205:
            break

        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        radius = 0.04
        z_offset = 0.02
        angle = free_space_angle(state, bowl_name, radius, tried)
        if angle is None:
            break
        tried.append(angle)

        # Tangential jaw alignment, as measured by the successful probes.
        desired_yaw = angle - pi / 2
        yaw = state["eef_yaw"] + (
            (desired_yaw - state["eef_yaw"] + pi / 2) % pi - pi / 2
        )
        eef = state["eef_pos"]
        approach_z = max(eef[2], bowl[2] + 0.14)
        move(
            sdk, [eef[0], eef[1], approach_z],
            state["eef_yaw"], "open", 22
        )
        grip(sdk, "open", 5)

        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        contact = [
            bowl[0] + radius * cos(angle),
            bowl[1] + radius * sin(angle),
            bowl[2] + z_offset
        ]
        move(
            sdk, [contact[0], contact[1], approach_z],
            yaw, "open", 30
        )

        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        lift_origin = bowl[2]
        contact = [
            bowl[0] + radius * cos(angle),
            bowl[1] + radius * sin(angle),
            bowl[2] + z_offset
        ]
        move(sdk, contact, yaw, "open", 32, 0.008)

        # Contact may prevent strict convergence; do not skip closing.
        state = sdk.get_state()
        bowl = state["objects"][bowl_name]["pos"]
        contact = [
            bowl[0] + radius * cos(angle),
            bowl[1] + radius * sin(angle),
            bowl[2] + z_offset
        ]
        if distance_xyz(state["eef_pos"], contact) > 0.018:
            move(sdk, contact, yaw, "open", 10, 0.010)
        grip(sdk, "close", 12)

        state = sdk.get_state()
        eef = state["eef_pos"]
        move(
            sdk, [eef[0], eef[1], eef[2] + 0.10],
            yaw, "close", 27, 0.010
        )
        state = sdk.get_state()
        bowl = state["objects"][bowl_name]["pos"]
        if (
            bowl[2] - lift_origin >= 0.03
            and distance_xyz(bowl, state["eef_pos"]) <= 0.115
        ):
            held = True
            break

        # Lower a possible partial grasp before opening and changing angle.
        if bowl[2] - lift_origin > 0.012:
            eef = state["eef_pos"]
            move(
                sdk,
                [eef[0], eef[1], eef[2] - (bowl[2] - lift_origin)],
                yaw, "close", 15
            )
        grip(sdk, "open", 6)

    if not held:
        return {"status": "failed", "reason": "Live lift not verified"}

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    object_height = max(bowl[2], plate[2] + 0.14)

    for name in state["objects"]:
        if name != bowl_name:
            obstacle = state["objects"][name]["pos"]
            if segment_distance(obstacle, bowl, plate) < 0.12:
                object_height = max(object_height, obstacle[2] + 0.14)
    for name in state["fixtures"]:
        obstacle = state["fixtures"][name]["pos"]
        if segment_distance(obstacle, bowl, plate) < 0.20:
            object_height = max(object_height, obstacle[2] + 0.16)
            for site_name in state["sites"]:
                if name in site_name:
                    object_height = max(
                        object_height, state["sites"][site_name][2] + 0.12
                    )

    carry_z = max(eef[2], object_height - offset[2])
    move(sdk, [eef[0], eef[1], carry_z], yaw, "close", 25)
    state = sdk.get_state()
    if state["eef_pos"][2] < carry_z - 0.025:
        return {"status": "failed", "reason": "Transport height unavailable"}

    for correction in range(2):
        state = sdk.get_state()
        bowl = state["objects"][bowl_name]["pos"]
        plate = state["objects"][plate_name]["pos"]
        eef = state["eef_pos"]
        if (
            bowl[2] - lift_origin < 0.03
            or distance_xyz(bowl, eef) > 0.115
        ):
            return {"status": "failed", "reason": "Bowl lost during transport"}
        target = [
            eef[0] + plate[0] - bowl[0],
            eef[1] + plate[1] - bowl[1],
            carry_z
        ]
        move(sdk, target, yaw, "close", 35 if correction == 0 else 15, 0.008)
        state = sdk.get_state()
        bowl = state["objects"][bowl_name]["pos"]
        plate = state["objects"][plate_name]["pos"]
        if hypot(bowl[0] - plate[0], bowl[1] - plate[1]) < 0.018:
            break

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    if (
        hypot(bowl[0] - plate[0], bowl[1] - plate[1]) > 0.035
        or distance_xyz(bowl, eef) > 0.115
        or bowl[2] - lift_origin < 0.03
    ):
        return {"status": "failed", "reason": "Plate alignment not verified"}

    target = [
        eef[0] + plate[0] - bowl[0],
        eef[1] + plate[1] - bowl[1],
        eef[2] + plate[2] + 0.035 - bowl[2]
    ]
    move(sdk, target, yaw, "close", 32, 0.008)

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if (
        hypot(bowl[0] - plate[0], bowl[1] - plate[1]) > 0.04
        or bowl[2] - plate[2] > 0.075
    ):
        return {"status": "failed", "reason": "Release pose not verified"}

    if not grip(sdk, "open", 12):
        return {"status": "failed", "reason": "No release budget"}
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        sdk, [eef[0], eef[1], eef[2] + 0.12],
        yaw, "open", 22
    )
    grip(sdk, "open", 8)

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if (
        hypot(bowl[0] - plate[0], bowl[1] - plate[1]) < 0.045
        and -0.015 <= bowl[2] - plate[2] <= 0.065
        and distance_xyz(bowl, state["eef_pos"]) > 0.09
    ):
        return {"status": "done", "object": bowl_name, "target": plate_name}
    return {"status": "failed", "reason": "Final placement not verified"}
