def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(450, state["max_steps"]) - state["steps_used"])


def distance_xy(a, b):
    return hypot(a[0] - b[0], a[1] - b[1])


def distance_xyz(a, b):
    return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def move_pose(sdk, position, yaw, grip, steps, tolerance=0.012):
    available = remaining_steps(sdk)
    if available <= 0:
        return False
    sdk.move_to(
        position,
        yaw=yaw,
        gripper=grip,
        max_steps=min(steps, available),
        tol=tolerance,
    )
    return distance_xyz(sdk.get_state()["eef_pos"], position) <= tolerance


def actuate(sdk, action, steps):
    available = remaining_steps(sdk)
    if available <= 0:
        return False
    sdk.set_gripper(action, steps=min(steps, available))
    return True


def free_space_angle(state, bowl_name, attempted):
    bowl = state["objects"][bowl_name]["pos"]
    neighbors = []
    for name in state["objects"]:
        if name != bowl_name:
            neighbors.append(state["objects"][name]["pos"])
    for name in state["fixtures"]:
        neighbors.append(state["fixtures"][name]["pos"])
        for site_name in state["sites"]:
            if name in site_name:
                neighbors.append(state["sites"][site_name])

    best_angle = None
    best_clearance = -1.0
    for index in range(144):
        angle = 2.0 * pi * index / 144.0
        distinct = True
        for previous in attempted:
            separation = abs(atan2(sin(angle - previous), cos(angle - previous)))
            if separation < pi / 3.0:
                distinct = False
        if distinct:
            contact = [
                bowl[0] + 0.04 * cos(angle),
                bowl[1] + 0.04 * sin(angle),
            ]
            clearance = 10.0
            for neighbor in neighbors:
                clearance = min(clearance, distance_xy(contact, neighbor))
            if clearance > best_clearance:
                best_clearance = clearance
                best_angle = angle
    return best_angle


def aligned_yaw(angle, jaw_relative, current_yaw):
    yaw = atan2(sin(angle + jaw_relative), cos(angle + jaw_relative))
    choices = [yaw - pi, yaw, yaw + pi]
    best = choices[0]
    for candidate in choices:
        if abs(candidate - current_yaw) < abs(best - current_yaw):
            best = candidate
    return best


def lifted_near_eef(state, bowl_name, reference_z):
    bowl = state["objects"][bowl_name]["pos"]
    return (
        bowl[2] - reference_z >= 0.03
        and distance_xyz(bowl, state["eef_pos"]) <= 0.11
    )


def run_policy(sdk):
    state = sdk.get_state()
    bowls = [name for name in state["objects"] if "black_bowl" in name]
    plates = [name for name in state["objects"] if "plate" in name]
    if not bowls or not plates:
        return {"status": "failed", "reason": "Required objects unavailable"}

    center = state["sites"].get("table_top")
    if center is None:
        if "akita_black_bowl_1" not in bowls:
            return {"status": "failed", "reason": "Cannot identify center bowl"}
        bowl_name = "akita_black_bowl_1"
    else:
        bowl_name = bowls[0]
        for name in bowls:
            if distance_xy(state["objects"][name]["pos"], center) < distance_xy(
                state["objects"][bowl_name]["pos"], center
            ):
                bowl_name = name

    plate_name = plates[0]
    for name in plates:
        if distance_xy(
            state["objects"][name]["pos"], state["objects"][bowl_name]["pos"]
        ) < distance_xy(
            state["objects"][plate_name]["pos"],
            state["objects"][bowl_name]["pos"],
        ):
            plate_name = name

    initial_bowl_z = state["objects"][bowl_name]["pos"][2]
    origin_height_difference = (
        initial_bowl_z - state["objects"][plate_name]["pos"][2]
    )
    attempted = []
    held = False
    reference_z = initial_bowl_z
    grasp_yaw = state["eef_yaw"]

    for attempt in range(3):
        if remaining_steps(sdk) < 220:
            break

        state = sdk.get_state()
        angle = free_space_angle(state, bowl_name, attempted)
        if angle is None:
            break
        attempted.append(angle)

        # Both alignments lifted in the probes; use the best sample's
        # radial alignment first, with the measured perpendicular alternative.
        jaw_relative = pi / 2.0 if attempt == 2 else 0.0
        grasp_yaw = aligned_yaw(angle, jaw_relative, state["eef_yaw"])
        offset_x = 0.04 * cos(angle)
        offset_y = 0.04 * sin(angle)
        bowl = state["objects"][bowl_name]["pos"]
        hover_z = bowl[2] + 0.16

        actuate(sdk, "open", 6)

        # A small hover residual must not restart an unreachable upward move.
        hover = [bowl[0] + offset_x, bowl[1] + offset_y, hover_z]
        move_pose(sdk, hover, grasp_yaw, "open", 55, 0.02)
        state = sdk.get_state()
        if distance_xy(state["eef_pos"], hover) > 0.025:
            move_pose(sdk, hover, grasp_yaw, "open", 18, 0.02)
            state = sdk.get_state()
        if (
            distance_xy(state["eef_pos"], hover) > 0.025
            or state["eef_pos"][2] < bowl[2] + 0.09
        ):
            continue

        bowl = state["objects"][bowl_name]["pos"]
        reference_z = bowl[2]
        contact = [
            bowl[0] + offset_x,
            bowl[1] + offset_y,
            bowl[2] + 0.02,
        ]
        move_pose(sdk, contact, grasp_yaw, "open", 42, 0.007)
        state = sdk.get_state()
        bowl = state["objects"][bowl_name]["pos"]
        contact = [
            bowl[0] + offset_x,
            bowl[1] + offset_y,
            bowl[2] + 0.02,
        ]
        if distance_xyz(state["eef_pos"], contact) > 0.012:
            move_pose(sdk, contact, grasp_yaw, "open", 14, 0.008)
            state = sdk.get_state()
        if distance_xyz(state["eef_pos"], contact) > 0.015:
            eef = state["eef_pos"]
            move_pose(
                sdk, [eef[0], eef[1], hover_z],
                grasp_yaw, "open", 20, 0.02
            )
            continue

        actuate(sdk, "close", 12)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move_pose(
            sdk, [eef[0], eef[1], eef[2] + 0.10],
            grasp_yaw, "close", 30, 0.015
        )
        state = sdk.get_state()
        if lifted_near_eef(state, bowl_name, reference_z):
            held = True
            break

        # Release a failed attempt near its live support height.
        eef = state["eef_pos"]
        bowl = state["objects"][bowl_name]["pos"]
        if distance_xyz(eef, bowl) < 0.11:
            move_pose(
                sdk, [eef[0], eef[1], reference_z + 0.03],
                grasp_yaw, "close", 15, 0.012
            )
        actuate(sdk, "open", 6)
        state = sdk.get_state()
        eef = state["eef_pos"]
        bowl = state["objects"][bowl_name]["pos"]
        move_pose(
            sdk, [eef[0], eef[1], bowl[2] + 0.15],
            grasp_yaw, "open", 20, 0.02
        )

    if not held:
        return {"status": "failed", "reason": "No live lift verified"}
    if remaining_steps(sdk) < 105:
        return {"status": "failed", "reason": "Insufficient transport budget"}

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    eef = state["eef_pos"]
    plate = state["objects"][plate_name]["pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]

    # Lift vertically before carrying the bowl across neighboring objects.
    transit_bowl_z = max(bowl[2], plate[2] + 0.14)
    for name in state["objects"]:
        if name != bowl_name:
            transit_bowl_z = max(
                transit_bowl_z, state["objects"][name]["pos"][2] + 0.12
            )
    move_pose(
        sdk, [eef[0], eef[1], transit_bowl_z - offset[2]],
        grasp_yaw, "close", 23, 0.015
    )
    state = sdk.get_state()
    if not lifted_near_eef(state, bowl_name, reference_z):
        return {"status": "failed", "reason": "Lift lost before transport"}

    bowl = state["objects"][bowl_name]["pos"]
    eef = state["eef_pos"]
    plate = state["objects"][plate_name]["pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    move_pose(
        sdk,
        [plate[0] - offset[0], plate[1] - offset[1],
         transit_bowl_z - offset[2]],
        grasp_yaw, "close", min(48, remaining_steps(sdk) - 55), 0.012
    )

    state = sdk.get_state()
    if not lifted_near_eef(state, bowl_name, reference_z):
        return {"status": "failed", "reason": "Lift lost during transport"}
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if distance_xy(bowl, plate) > 0.025:
        return {"status": "failed", "reason": "Bowl not aligned above plate"}

    eef = state["eef_pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    resting_z = plate[2] + origin_height_difference + 0.015
    move_pose(
        sdk,
        [plate[0] - offset[0], plate[1] - offset[1],
         resting_z + 0.008 - offset[2]],
        grasp_yaw, "close", min(38, remaining_steps(sdk) - 28), 0.008
    )
    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if (
        distance_xy(bowl, plate) > 0.03
        or abs(bowl[2] - resting_z) > 0.035
        or distance_xyz(bowl, state["eef_pos"]) > 0.11
    ):
        return {"status": "failed", "reason": "Release pose not verified"}

    actuate(sdk, "open", 9)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move_pose(
        sdk, [eef[0], eef[1], eef[2] + 0.12],
        grasp_yaw, "open", min(23, remaining_steps(sdk) - 6), 0.02
    )
    available = remaining_steps(sdk)
    if available > 0:
        sdk.step_raw(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
            steps=min(8, available),
        )

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    expected_z = plate[2] + origin_height_difference + 0.015
    placed = (
        distance_xy(bowl, plate) <= 0.035
        and abs(bowl[2] - expected_z) <= 0.03
        and state["eef_pos"][2] - bowl[2] >= 0.07
    )
    return {
        "status": "done" if placed else "failed",
        "object": bowl_name,
        "target": plate_name,
        "placement_verified": placed,
        "grasp_verified_by_lift": held,
    }
