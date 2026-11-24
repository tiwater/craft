def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, state["max_steps"] - state["steps_used"])

    def move(position, yaw, grip, steps=30, tolerance=0.012):
        budget = remaining()
        if budget <= 0:
            return False
        result = sdk.move_to(
            position,
            yaw=yaw,
            gripper=grip,
            max_steps=min(steps, budget),
            tol=tolerance,
        )
        return result["reached"]

    def gripper(action, steps):
        budget = remaining()
        if budget > 0:
            return sdk.set_gripper(action, steps=min(steps, budget))
        return {"gripper_width": 0.0, "grasped_hint": False}

    state = sdk.get_state()
    bowls = [name for name in state["objects"] if "black_bowl" in name]
    plates = [name for name in state["objects"] if "plate" in name]
    cabinets = [
        name for name in state["fixtures"] if "wooden_cabinet" in name
    ]
    if not bowls or not plates or not cabinets:
        return {"status": "failed", "reason": "Required scene entities missing"}

    cabinet = cabinets[0]
    anchor = state["fixtures"][cabinet]["pos"]
    top_name = cabinet + "_top_region"
    if top_name in state["sites"]:
        anchor = state["sites"][top_name]

    bowl = bowls[0]
    best = float("inf")
    for name in bowls:
        position = state["objects"][name]["pos"]
        score = (
            (position[0] - anchor[0]) ** 2
            + (position[1] - anchor[1]) ** 2
            + 0.5 * (position[2] - anchor[2]) ** 2
        )
        if score < best:
            best = score
            bowl = name

    plate = plates[0]
    original = list(state["objects"][bowl]["pos"])
    bottom_offset = 0.035
    if top_name in state["sites"]:
        estimate = original[2] - state["sites"][top_name][2]
        if 0.015 <= estimate <= 0.065:
            bottom_offset = estimate

    clearance = max(state["eef_pos"][2], original[2] + 0.12)
    yaw = state["eef_yaw"]
    held = False
    attempts = [
        [0.045, 0.020, 0.0],
        [-0.045, 0.010, 0.0],
        [0.045, 0.025, pi / 2.0],
        [0.035, 0.0, pi / 2.0],
    ]

    for attempt in attempts:
        if remaining() < 175:
            break

        gripper("open", 6)
        state = sdk.get_state()
        position = list(state["objects"][bowl]["pos"])
        eef = state["eef_pos"]
        safe_height = max(clearance, position[2] + 0.12)
        move([eef[0], eef[1], safe_height], yaw, "open", 25)

        angle = yaw + attempt[2]
        radial = [-sin(angle), cos(angle)]
        grasp = [
            position[0] + radial[0] * attempt[0],
            position[1] + radial[1] * attempt[0],
            position[2] + attempt[1],
        ]
        move([grasp[0], grasp[1], safe_height], yaw, "open", 30)
        state = sdk.get_state()
        if hypot(
            state["eef_pos"][0] - grasp[0],
            state["eef_pos"][1] - grasp[1],
        ) > 0.025:
            continue

        position = state["objects"][bowl]["pos"]
        grasp = [
            position[0] + radial[0] * attempt[0],
            position[1] + radial[1] * attempt[0],
            position[2] + attempt[1],
        ]
        move(grasp, yaw, "open", 30, 0.006)
        state = sdk.get_state()
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - grasp[0], eef[1] - grasp[1]) > 0.022
            or abs(eef[2] - grasp[2]) > 0.035
        ):
            continue

        before = list(state["objects"][bowl]["pos"])
        gripper("close", 12)
        hand_before = list(sdk.get_state()["eef_pos"])
        move(
            [hand_before[0], hand_before[1], hand_before[2] + 0.09],
            yaw,
            "close",
            28,
            0.008,
        )
        state = sdk.get_state()
        lifted = state["objects"][bowl]["pos"]
        hand = state["eef_pos"]
        rise = lifted[2] - before[2]
        hand_rise = hand[2] - hand_before[2]
        if (
            rise > 0.045
            and abs(rise - hand_rise) < 0.035
            and hypot(lifted[0] - hand[0], lifted[1] - hand[1]) < 0.11
        ):
            held = True
            break

    if not held:
        return {"status": "failed", "reason": "No verified rim grasp"}

    state = sdk.get_state()
    position = state["objects"][bowl]["pos"]
    eef = state["eef_pos"]
    offset = [eef[i] - position[i] for i in range(3)]

    # Keep only the clearance needed above the source support.
    travel_height = max(eef[2], original[2] + 0.09 + offset[2])
    move([eef[0], eef[1], travel_height], yaw, "close", 22)

    target = sdk.get_state()["objects"][plate]["pos"]
    # A high waypoint can be outside the arm workspace. Its reached flag
    # must not prevent proceeding from an otherwise useful transport pose.
    move(
        [target[0] + offset[0], target[1] + offset[1], travel_height],
        yaw,
        "close",
        40,
        0.015,
    )

    aligned = False
    for stage in range(7):
        if remaining() < 65:
            break
        state = sdk.get_state()
        position = state["objects"][bowl]["pos"]
        target = state["objects"][plate]["pos"]
        eef = state["eef_pos"]
        current_offset = [eef[i] - position[i] for i in range(3)]
        if max(abs(current_offset[i] - offset[i]) for i in range(3)) > 0.05:
            return {"status": "failed", "reason": "Grasp lost during transport"}

        error = hypot(position[0] - target[0], position[1] - target[1])
        hover_z = target[2] + bottom_offset + 0.10
        if error < 0.025 and position[2] <= hover_z + 0.025:
            aligned = True
            break

        distance_from_source = hypot(
            position[0] - original[0], position[1] - original[1]
        )
        next_bowl_z = max(hover_z, position[2] - 0.055)

        # Preserve clearance while still over the cabinet. Once the bowl
        # is near the destination, descend through reachable waypoints.
        if distance_from_source < 0.12 and error > 0.07:
            next_bowl_z = max(next_bowl_z, original[2] + 0.065)

        move(
            [
                target[0] + current_offset[0],
                target[1] + current_offset[1],
                next_bowl_z + current_offset[2],
            ],
            yaw,
            "close",
            24,
            0.010,
        )

    state = sdk.get_state()
    position = state["objects"][bowl]["pos"]
    target = state["objects"][plate]["pos"]
    if hypot(position[0] - target[0], position[1] - target[1]) > 0.045:
        return {"status": "failed", "reason": "Unable to align above plate"}

    # Use the live bowl pose for final corrections, including grasp settling.
    for stage in range(3):
        if remaining() < 36:
            break
        state = sdk.get_state()
        position = state["objects"][bowl]["pos"]
        target = state["objects"][plate]["pos"]
        eef = state["eef_pos"]
        desired_z = target[2] + bottom_offset + 0.012
        error = hypot(position[0] - target[0], position[1] - target[1])
        if error < 0.018 and position[2] <= desired_z + 0.018:
            break
        move(
            [
                eef[0] + target[0] - position[0],
                eef[1] + target[1] - position[1],
                eef[2] + max(desired_z - position[2], -0.075),
            ],
            yaw,
            "close",
            min(25, remaining() - 35),
            0.006,
        )

    state = sdk.get_state()
    position = state["objects"][bowl]["pos"]
    target = state["objects"][plate]["pos"]
    if (
        hypot(position[0] - target[0], position[1] - target[1]) > 0.04
        or position[2] - target[2] > bottom_offset + 0.065
        or remaining() < 20
    ):
        return {"status": "failed", "reason": "Release pose not reached"}

    gripper("open", 10)
    eef = sdk.get_state()["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.10],
        yaw,
        "open",
        min(22, max(1, remaining() - 10)),
    )
    gripper("open", min(14, remaining()))

    state = sdk.get_state()
    position = list(state["objects"][bowl]["pos"])
    target = state["objects"][plate]["pos"]
    error = hypot(position[0] - target[0], position[1] - target[1])
    height = position[2] - target[2]

    # Object origins are not contact surfaces; allow their geometry offsets.
    placed = error < 0.05 and -0.025 < height < bottom_offset + 0.07
    return {
        "status": "done" if placed else "failed",
        "object": bowl,
        "target": plate,
        "placement_verified_by_pose": placed,
        "horizontal_error_m": error,
        "relative_height_m": height,
        "steps_used": state["steps_used"],
    }
