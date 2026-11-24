def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, state["max_steps"] - state["steps_used"])

    def move(position, yaw, action, limit, tolerance=0.009):
        budget = remaining()
        if budget < 1:
            return False
        result = sdk.move_to(
            position, yaw=yaw, gripper=action,
            max_steps=min(limit, budget), tol=tolerance
        )
        return result["reached"]

    def grip(action, count):
        budget = remaining()
        if budget < 1:
            return False
        sdk.set_gripper(action, steps=min(count, budget))
        return True

    state = sdk.get_state()
    objects = state["objects"]
    sites = state["sites"]
    bowls = sorted([name for name in objects if "black_bowl" in name])
    plates = sorted([name for name in objects if "plate" in name])
    candidates = []
    for region in sites:
        if "wooden_cabinet" in region and region.endswith("_top_region"):
            anchor = sites[region]
            for name in bowls:
                position = objects[name]["pos"]
                horizontal = hypot(position[0] - anchor[0], position[1] - anchor[1])
                vertical = position[2] - anchor[2]
                if horizontal < 0.16 and -0.015 < vertical < 0.12:
                    candidates.append(
                        (horizontal * horizontal + vertical * vertical, name, region)
                    )
    if not candidates:
        for region in sites:
            if "wooden_cabinet" in region and region.endswith("_top_side"):
                anchor = sites[region]
                for name in bowls:
                    position = objects[name]["pos"]
                    horizontal = hypot(position[0] - anchor[0], position[1] - anchor[1])
                    vertical = position[2] - anchor[2]
                    if horizontal < 0.16 and -0.025 < vertical < 0.10:
                        candidates.append(
                            (horizontal * horizontal + vertical * vertical, name, region)
                        )
    if not candidates or not plates:
        return {
            "status": "failed",
            "reason": "Could not ground cabinet bowl and target plate",
            "bowl_candidates": bowls,
            "plate_candidates": plates,
            "available_sites": sorted(list(sites)),
        }

    selection = sorted(candidates)[0]
    bowl = selection[1]
    source_site = selection[2]
    if len(plates) == 1:
        plate = plates[0]
    else:
        targets = []
        for name in plates:
            for region in sites:
                if region.endswith("_plate_region"):
                    distance = sum([
                        (objects[name]["pos"][i] - sites[region][i]) ** 2
                        for i in range(3)
                    ])
                    targets.append((distance, name))
        if not targets:
            return {"status": "failed", "reason": "Ambiguous target plate"}
        plate = sorted(targets)[0][1]

    initial_bowl = list(objects[bowl]["pos"])
    grounding = {
        "object": bowl,
        "object_position": list(initial_bowl),
        "source_site": source_site,
        "source_position": list(sites[source_site]),
        "target": plate,
        "target_position": list(objects[plate]["pos"]),
    }
    bottom_offset = 0.035
    if source_site.endswith("_top_region"):
        estimate = initial_bowl[2] - sites[source_site][2]
        if 0.015 <= estimate <= 0.065:
            bottom_offset = estimate

    initial_yaw = state["eef_yaw"]
    attempts = []
    held = False
    grasp_heights = [0.025, 0.015, 0.005]

    for attempt in range(3):
        if remaining() < 180:
            break
        state = sdk.get_state()
        position = list(state["objects"][bowl]["pos"])
        eef = list(state["eef_pos"])
        yaw = initial_yaw
        if attempt == 1:
            yaw += pi / 2.0
        safe_height = max(position[2] + 0.10, eef[2])
        move([eef[0], eef[1], safe_height], yaw, "open", 20)
        grip("open", 8)
        move(
            [position[0], position[1], safe_height],
            yaw, "open", 42
        )

        state = sdk.get_state()
        position = list(state["objects"][bowl]["pos"])
        eef = state["eef_pos"]
        if hypot(eef[0] - position[0], eef[1] - position[1]) > 0.018:
            attempts.append({"stage": "approach", "reached": False})
            continue

        grasp_position = [
            position[0], position[1], position[2] + grasp_heights[attempt]
        ]
        move(grasp_position, yaw, "open", 32, 0.005)
        state = sdk.get_state()
        eef = state["eef_pos"]
        error = sqrt(sum([
            (eef[i] - grasp_position[i]) ** 2 for i in range(3)
        ]))
        if error > 0.018:
            attempts.append({"stage": "descent", "position_error_m": error})
            continue

        before = list(state["objects"][bowl]["pos"])
        grip("close", 16)
        closed = sdk.get_state()
        eef_before = list(closed["eef_pos"])
        move(
            [eef_before[0], eef_before[1], eef_before[2] + 0.085],
            yaw, "close", 30, 0.008
        )
        state = sdk.get_state()
        after = state["objects"][bowl]["pos"]
        eef = state["eef_pos"]
        bowl_rise = after[2] - before[2]
        hand_rise = eef[2] - eef_before[2]
        held = (
            bowl_rise > 0.04
            and abs(bowl_rise - hand_rise) < 0.025
            and hypot(after[0] - eef[0], after[1] - eef[1]) < 0.065
        )
        attempts.append({
            "stage": "lift",
            "grasp_height_offset_m": grasp_heights[attempt],
            "bowl_rise_m": bowl_rise,
            "hand_rise_m": hand_rise,
            "gripper_width": closed["gripper_width"],
            "verified": held,
        })
        if held:
            break

    if not held:
        return {
            "status": "failed",
            "reason": "Grounded bowl did not follow gripper during lift",
            "grounding": grounding,
            "attempts": attempts,
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    position = state["objects"][bowl]["pos"]
    eef = state["eef_pos"]
    offset = [eef[i] - position[i] for i in range(3)]
    travel_height = max(initial_bowl[2] + 0.115, position[2]) + offset[2]
    move([eef[0], eef[1], travel_height], yaw, "close", 22)

    state = sdk.get_state()
    if state["objects"][bowl]["pos"][2] < initial_bowl[2] + 0.06:
        return {"status": "failed", "reason": "Insufficient cabinet clearance"}
    target = state["objects"][plate]["pos"]
    move(
        [target[0] + offset[0], target[1] + offset[1], travel_height],
        yaw, "close", 60, 0.007
    )

    for correction in range(2):
        state = sdk.get_state()
        position = state["objects"][bowl]["pos"]
        target = state["objects"][plate]["pos"]
        eef = state["eef_pos"]
        drift = sqrt(sum([
            (eef[i] - position[i] - offset[i]) ** 2 for i in range(3)
        ]))
        if drift > 0.035:
            return {"status": "failed", "reason": "Bowl slipped during transport"}
        error_x = target[0] - position[0]
        error_y = target[1] - position[1]
        if hypot(error_x, error_y) < 0.008:
            break
        if remaining() < 85:
            break
        move(
            [eef[0] + error_x, eef[1] + error_y, travel_height],
            yaw, "close", 18, 0.005
        )

    state = sdk.get_state()
    position = state["objects"][bowl]["pos"]
    target = state["objects"][plate]["pos"]
    if hypot(position[0] - target[0], position[1] - target[1]) > 0.025:
        return {"status": "failed", "reason": "Plate alignment failed"}

    for clearance in [0.045, 0.006]:
        if remaining() < 35:
            break
        state = sdk.get_state()
        position = state["objects"][bowl]["pos"]
        target = state["objects"][plate]["pos"]
        eef = state["eef_pos"]
        desired_height = target[2] + bottom_offset + clearance
        move(
            [
                eef[0] + target[0] - position[0],
                eef[1] + target[1] - position[1],
                eef[2] + desired_height - position[2],
            ],
            yaw, "close", min(38, remaining() - 28), 0.005
        )

    state = sdk.get_state()
    position = state["objects"][bowl]["pos"]
    target = state["objects"][plate]["pos"]
    horizontal_error = hypot(position[0] - target[0], position[1] - target[1])
    release_gap = position[2] - target[2] - bottom_offset
    if horizontal_error > 0.025 or release_gap > 0.03 or release_gap < -0.02:
        return {
            "status": "failed",
            "reason": "Release pose outside placement tolerance",
            "horizontal_error_m": horizontal_error,
            "release_gap_m": release_gap,
            "grounding": grounding,
        }
    if remaining() < 20:
        return {"status": "failed", "reason": "Insufficient release budget"}

    grip("open", 10)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.10],
        yaw, "open", min(22, max(1, remaining() - 8)), 0.012
    )
    grip("open", 8)

    state = sdk.get_state()
    position = state["objects"][bowl]["pos"]
    target = state["objects"][plate]["pos"]
    horizontal_error = hypot(position[0] - target[0], position[1] - target[1])
    height_difference = position[2] - target[2]
    placed = (
        horizontal_error < 0.04
        and 0.012 < height_difference < bottom_offset + 0.025
        and state["eef_pos"][2] - position[2] > 0.055
    )
    return {
        "status": "done" if placed else "failed",
        "grounding": grounding,
        "attempts": attempts,
        "placement_verified_by_pose": placed,
        "horizontal_error_m": horizontal_error,
        "height_difference_m": height_difference,
        "steps_used": state["steps_used"],
    }
