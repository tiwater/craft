def policy_move(sdk, pos, yaw, gripper, limit=35, tol=0.008):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        pos, yaw=yaw, gripper=gripper,
        max_steps=min(limit, remaining), tol=tol
    )
    return result["reached"]


def policy_grip(sdk, action, steps=12):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining > 0:
        return sdk.set_gripper(action, steps=min(steps, remaining))
    return None


def run_policy(sdk):
    state = sdk.get_state()
    objects = state["objects"]
    sites = state["sites"]
    bowls = sorted([name for name in objects if "black_bowl" in name])
    plates = sorted([name for name in objects if "plate" in name])
    ramekins = sorted([name for name in objects if "ramekin" in name])
    if not bowls or not plates or not ramekins:
        return {"status": "failed", "reason": "Required objects unavailable"}

    regions = sorted([
        name for name in sites
        if "between" in name and "plate" in name and "ramekin" in name
    ])
    candidates = []
    for plate_name in plates:
        plate = objects[plate_name]["pos"]
        for ramekin_name in ramekins:
            ramekin = objects[ramekin_name]["pos"]
            vx = ramekin[0] - plate[0]
            vy = ramekin[1] - plate[1]
            length2 = vx * vx + vy * vy
            if length2 < 0.000001:
                continue
            for bowl_name in bowls:
                bowl = objects[bowl_name]["pos"]
                t = ((bowl[0] - plate[0]) * vx +
                     (bowl[1] - plate[1]) * vy) / length2
                lateral2 = ((bowl[0] - plate[0] - t * vx) ** 2 +
                            (bowl[1] - plate[1] - t * vy) ** 2)
                if t <= 0.0 or t >= 1.0 or lateral2 > 0.0036:
                    continue
                region_name = ""
                region_distance2 = 0.0
                if regions:
                    ranked_regions = sorted([
                        ((bowl[0] - sites[name][0]) ** 2 +
                         (bowl[1] - sites[name][1]) ** 2, name)
                        for name in regions
                    ])
                    region_distance2 = ranked_regions[0][0]
                    region_name = ranked_regions[0][1]
                score = lateral2 + 0.1 * length2 * (t - 0.5) ** 2
                if regions:
                    score += region_distance2
                candidates.append((
                    score, bowl_name, plate_name, ramekin_name,
                    region_name, t, sqrt(lateral2)
                ))

    if not candidates:
        return {"status": "failed", "reason": "No grounded bowl between references"}

    selected = sorted(candidates)[0]
    target = selected[1]
    plate_name = selected[2]
    ramekin_name = selected[3]
    target_site = target + "_default_site"
    plate_site = plate_name + "_default_site"
    grounding = {
        "target": target,
        "plate": plate_name,
        "ramekin": ramekin_name,
        "region": selected[4],
        "segment_fraction": selected[5],
        "lateral_distance": selected[6],
        "initial_bowl_pos": list(objects[target]["pos"]),
        "initial_plate_pos": list(objects[plate_name]["pos"]),
        "attempts": [],
    }
    if target_site in sites:
        grounding["target_site_pos"] = list(sites[target_site])
    if plate_site in sites:
        grounding["plate_site_pos"] = list(sites[plate_site])
    if selected[4]:
        grounding["region_pos"] = list(sites[selected[4]])

    initial_z = objects[target]["pos"][2]
    initial_yaw = state["eef_yaw"]
    grasped = False
    yaw = initial_yaw

    for attempt in range(2):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 190:
            break
        bowl = state["objects"][target]["pos"]
        plate = state["objects"][plate_name]["pos"]
        ramekin = state["objects"][ramekin_name]["pos"]
        clearance = max(bowl[2], plate[2], ramekin[2]) + 0.17
        eef = state["eef_pos"]
        policy_move(
            sdk, [eef[0], eef[1], max(eef[2], clearance)],
            state["eef_yaw"], "open", 25
        )
        policy_grip(sdk, "open", 8)
        yaw = initial_yaw + attempt * pi / 2.0
        bowl = sdk.get_state()["objects"][target]["pos"]
        policy_move(
            sdk, [bowl[0], bowl[1], clearance],
            yaw, "open", 35, 0.007
        )
        bowl = sdk.get_state()["objects"][target]["pos"]
        policy_move(
            sdk, [bowl[0], bowl[1], bowl[2] + 0.075],
            yaw, "open", 22, 0.006
        )
        bowl = sdk.get_state()["objects"][target]["pos"]
        grasp_height = 0.020 if attempt == 0 else 0.005
        grasp_pos = [bowl[0], bowl[1], bowl[2] + grasp_height]
        policy_move(sdk, grasp_pos, yaw, "open", 30, 0.005)

        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        eef = state["eef_pos"]
        record = {
            "attempt": attempt,
            "requested_grasp_pos": list(grasp_pos),
            "actual_eef_pos": list(eef),
            "bowl_pos": list(bowl),
        }
        grounding["attempts"].append(record)
        if (hypot(eef[0] - bowl[0], eef[1] - bowl[1]) > 0.018 or
                abs(eef[2] - grasp_pos[2]) > 0.018):
            record["result"] = "Grasp approach missed"
            continue

        before_z = bowl[2]
        policy_grip(sdk, "close", 18)
        state = sdk.get_state()
        record["closed_width"] = state["gripper_width"]
        eef = state["eef_pos"]
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.095],
            yaw, "close", 30, 0.007
        )
        state = sdk.get_state()
        lift = state["objects"][target]["pos"][2] - before_z
        record["lift"] = lift
        record["result"] = "Lift verified" if lift > 0.05 else "Lift failed"
        if lift > 0.05:
            grasped = True
            break
        policy_grip(sdk, "open", 8)

    if not grasped:
        return {
            "status": "failed",
            "reason": "Grounded bowl did not lift",
            "grounding": grounding,
        }

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    transport_z = max(bowl[2], plate[2] + 0.14)
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + transport_z - bowl[2]],
        yaw, "close", 25, 0.008
    )

    for segment in range(3):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 75:
            break
        bowl = state["objects"][target]["pos"]
        plate = state["objects"][plate_name]["pos"]
        eef = state["eef_pos"]
        if bowl[2] < initial_z + 0.045:
            return {
                "status": "failed",
                "reason": "Bowl slipped during transport",
                "grounding": grounding,
            }
        dx = plate[0] - bowl[0]
        dy = plate[1] - bowl[1]
        if hypot(dx, dy) < 0.009:
            break
        policy_move(
            sdk,
            [eef[0] + dx, eef[1] + dy,
             eef[2] + transport_z - bowl[2]],
            yaw, "close", 32, 0.006
        )

    for height in [0.060, 0.025]:
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 40:
            break
        bowl = state["objects"][target]["pos"]
        plate = state["objects"][plate_name]["pos"]
        eef = state["eef_pos"]
        if hypot(bowl[0] - plate[0], bowl[1] - plate[1]) > 0.025:
            return {
                "status": "failed",
                "reason": "Bowl not aligned above selected plate",
                "grounding": grounding,
            }
        policy_move(
            sdk,
            [eef[0] + plate[0] - bowl[0],
             eef[1] + plate[1] - bowl[1],
             eef[2] + plate[2] + height - bowl[2]],
            yaw, "close", 25, 0.006
        )

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if (hypot(bowl[0] - plate[0], bowl[1] - plate[1]) > 0.025 or
            not plate[2] - 0.015 <= bowl[2] <= plate[2] + 0.065):
        return {
            "status": "failed",
            "reason": "Release pose outside placement tolerance",
            "grounding": grounding,
        }

    policy_grip(sdk, "open", 15)
    eef = sdk.get_state()["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.12],
        yaw, "open", 22, 0.010
    )
    policy_grip(sdk, "open", 10)
    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate = state["objects"][plate_name]["pos"]
    placed = (
        hypot(bowl[0] - plate[0], bowl[1] - plate[1]) < 0.035 and
        plate[2] - 0.015 <= bowl[2] <= plate[2] + 0.065 and
        state["gripper_width"] > 0.04
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Placement verified by live poses"
        if placed else "Final placement outside tolerance",
        "grounding": grounding,
        "final_bowl_pos": list(bowl),
        "final_plate_pos": list(plate),
    }
