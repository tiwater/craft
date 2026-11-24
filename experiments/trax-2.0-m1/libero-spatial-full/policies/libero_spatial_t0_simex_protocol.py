def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return state["max_steps"] - state["steps_used"]

    def move(position, yaw, action, steps=30, tol=0.008):
        budget = remaining()
        if budget <= 0:
            return False
        result = sdk.move_to(
            position, yaw=yaw, gripper=action,
            max_steps=min(steps, budget), tol=tol
        )
        return result["reached"]

    def grip(action, steps=12):
        budget = remaining()
        if budget <= 0:
            return False
        sdk.set_gripper(action, steps=min(steps, budget))
        return True

    state = sdk.get_state()
    objects = state["objects"]
    sites = state["sites"]
    bowls = sorted([name for name in objects if "black_bowl" in name])
    plates = sorted([name for name in objects if "plate" in name])
    ramekins = sorted([name for name in objects if "ramekin" in name])
    if not bowls or not plates or not ramekins:
        return {"status": "failed", "reason": "Required objects missing"}

    regions = sorted([
        name for name in sites
        if "between" in name and "plate" in name and "ramekin" in name
    ])
    region_name = regions[0] if regions else None
    candidates = []
    for plate_name in plates:
        plate = objects[plate_name]["pos"]
        for ramekin_name in ramekins:
            ramekin = objects[ramekin_name]["pos"]
            vx = ramekin[0] - plate[0]
            vy = ramekin[1] - plate[1]
            length_squared = vx * vx + vy * vy
            if length_squared < 0.0001:
                continue
            anchor = sites[region_name] if region_name else [
                (plate[i] + ramekin[i]) * 0.5 for i in range(3)
            ]
            for bowl_name in bowls:
                bowl = objects[bowl_name]["pos"]
                projection = (
                    (bowl[0] - plate[0]) * vx +
                    (bowl[1] - plate[1]) * vy
                ) / length_squared
                perpendicular = abs(
                    (bowl[0] - plate[0]) * vy -
                    (bowl[1] - plate[1]) * vx
                ) / sqrt(length_squared)
                outside = max(0.0, -projection, projection - 1.0)
                distance = hypot(bowl[0] - anchor[0], bowl[1] - anchor[1])
                score = distance + perpendicular * 0.5 + outside * 0.3
                candidates.append([
                    score, bowl_name, plate_name, ramekin_name,
                    projection, perpendicular, distance
                ])

    if not candidates:
        return {"status": "failed", "reason": "No valid plate and ramekin pair"}

    candidates = sorted(candidates)
    selected = candidates[0]
    target_name = selected[1]
    plate_name = selected[2]
    source = list(objects[target_name]["pos"])
    grounding = {
        "object": target_name,
        "object_site": target_name + "_default_site"
        if target_name + "_default_site" in sites else None,
        "source_region": region_name,
        "source_position": source,
        "target": plate_name,
        "target_position": list(objects[plate_name]["pos"]),
        "ramekin": selected[3],
        "between_projection": selected[4],
        "line_distance": selected[5],
        "region_distance": selected[6],
        "candidate_scores": candidates
    }

    base_yaw = state["eef_yaw"]
    held = False
    attempts = []
    offset = None

    for attempt in range(3):
        if remaining() < 155:
            break
        state = sdk.get_state()
        bowl = list(state["objects"][target_name]["pos"])
        plate = state["objects"][plate_name]["pos"]
        eef = state["eef_pos"]
        clearance = max(bowl[2], plate[2]) + 0.18
        yaw = base_yaw + [0.0, pi / 2, 0.0][attempt]

        move(
            [eef[0], eef[1], max(eef[2], clearance)],
            state["eef_yaw"], "open", 22, 0.012
        )
        grip("open", 6)
        state = sdk.get_state()
        bowl = list(state["objects"][target_name]["pos"])
        move([bowl[0], bowl[1], clearance], yaw, "open", 30)

        state = sdk.get_state()
        bowl = list(state["objects"][target_name]["pos"])
        eef = state["eef_pos"]
        if hypot(eef[0] - bowl[0], eef[1] - bowl[1]) > 0.018:
            continue

        grasp_z = bowl[2] + [0.050, 0.040, 0.060][attempt]
        move([bowl[0], bowl[1], grasp_z], yaw, "open", 32, 0.005)
        state = sdk.get_state()
        before = list(state["objects"][target_name]["pos"])
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - before[0], eef[1] - before[1]) > 0.018
            or abs(eef[2] - grasp_z) > 0.018
        ):
            attempts.append({"attempt": attempt, "reason": "Grasp pose not reached"})
            continue

        grip("close", 16)
        state = sdk.get_state()
        width = state["gripper_width"]
        eef = state["eef_pos"]
        move([eef[0], eef[1], eef[2] + 0.065], yaw, "close", 25, 0.006)
        state = sdk.get_state()
        bowl = state["objects"][target_name]["pos"]
        eef = state["eef_pos"]
        lift = bowl[2] - before[2]
        attempts.append({
            "attempt": attempt,
            "grasp_height_offset": grasp_z - before[2],
            "gripper_width": width,
            "observed_lift": lift
        })
        if lift > 0.035:
            move(
                [eef[0], eef[1], max(clearance, eef[2] + 0.045)],
                yaw, "close", 22
            )
            state = sdk.get_state()
            bowl = state["objects"][target_name]["pos"]
            eef = state["eef_pos"]
            if (
                bowl[2] > before[2] + 0.075
                and hypot(bowl[0] - eef[0], bowl[1] - eef[1]) < 0.055
            ):
                held = True
                offset = [bowl[i] - eef[i] for i in range(3)]
                break
        grip("open", 6)

    if not held:
        return {
            "status": "failed",
            "reason": "Grounded bowl did not remain lifted",
            "grounding": grounding,
            "attempts": attempts
        }

    state = sdk.get_state()
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    transit_z = max(eef[2], plate[2] + 0.14 - offset[2])
    move([eef[0], eef[1], transit_z], yaw, "close", 15)
    move(
        [plate[0] - offset[0], plate[1] - offset[1], transit_z],
        yaw, "close", 35, 0.006
    )

    for height in [0.085, 0.050, 0.030]:
        if remaining() < 43:
            break
        state = sdk.get_state()
        plate = state["objects"][plate_name]["pos"]
        bowl = state["objects"][target_name]["pos"]
        eef = state["eef_pos"]
        move(
            [
                eef[0] + plate[0] - bowl[0],
                eef[1] + plate[1] - bowl[1],
                eef[2] + plate[2] + height - bowl[2]
            ],
            yaw, "close", 20, 0.005
        )

    state = sdk.get_state()
    bowl = state["objects"][target_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if (
        hypot(bowl[0] - plate[0], bowl[1] - plate[1]) > 0.025
        or bowl[2] > plate[2] + 0.065
        or bowl[2] < plate[2] - 0.015
    ):
        return {
            "status": "failed",
            "reason": "Release pose not reached",
            "grounding": grounding,
            "attempts": attempts,
            "bowl_position": list(bowl),
            "plate_position": list(plate)
        }

    if not grip("open", 12):
        return {"status": "failed", "reason": "No release budget"}

    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.12], yaw, "open", 20, 0.012)
    grip("open", 8)

    state = sdk.get_state()
    bowl = state["objects"][target_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    centered = hypot(bowl[0] - plate[0], bowl[1] - plate[1]) < 0.040
    resting = plate[2] - 0.015 < bowl[2] < plate[2] + 0.060
    return {
        "status": "done" if centered and resting else "failed",
        "grounding": grounding,
        "attempts": attempts,
        "final_bowl_position": list(bowl),
        "final_plate_position": list(plate)
    }
