def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def distance(a, b):
        return hypot(a[0] - b[0], a[1] - b[1])

    def move(pos, yaw, action, budget=30, tol=0.006):
        count = min(budget, remaining())
        if count <= 0:
            return False
        return sdk.move_to(
            pos, yaw=yaw, gripper=action, max_steps=count, tol=tol
        )["reached"]

    def grip(action, budget=12):
        count = min(budget, remaining())
        if count > 0:
            return sdk.set_gripper(action, steps=count)
        return {}

    def nearest_yaw(yaw, reference):
        while yaw - reference > pi / 2:
            yaw -= pi
        while yaw - reference < -pi / 2:
            yaw += pi
        return yaw

    state = sdk.get_state()
    objects = state["objects"]
    sites = state["sites"]
    bowls = sorted([
        name for name in objects
        if "black" in name.lower() and "bowl" in name.lower()
    ])
    cookies = sorted([name for name in objects if "cookie" in name.lower()])
    plates = sorted([name for name in objects if "plate" in name.lower()])
    if not bowls or not cookies or not plates:
        return {"status": "failed", "reason": "Required objects are missing"}

    region = ""
    if "main_table_next_to_box_region" in sites:
        region = "main_table_next_to_box_region"
    else:
        for name in sorted(list(sites)):
            label = name.lower()
            if "next_to" in label and ("cookie" in label or "box" in label):
                region = name
                break

    scores = []
    for name in bowls:
        pos = objects[name]["pos"]
        cookie_distance = min(
            distance(pos, objects[cookie]["pos"]) for cookie in cookies
        )
        region_distance = distance(pos, sites[region]) if region else cookie_distance
        scores.append((region_distance, cookie_distance, name))
    scores = sorted(scores)
    target = scores[0][2]
    if len(scores) > 1 and abs(scores[1][0] - scores[0][0]) < 0.015:
        return {"status": "failed", "reason": "Ambiguous source bowl"}

    plate = plates[0]
    if len(plates) > 1:
        if "main_table_plate_region" not in sites:
            return {"status": "failed", "reason": "Ambiguous destination plate"}
        ranked = sorted([
            (distance(objects[name]["pos"], sites["main_table_plate_region"]), name)
            for name in plates
        ])
        plate = ranked[0][1]

    grounding = {
        "target": target,
        "destination": plate,
        "source_region": region,
        "initial_target_pos": list(objects[target]["pos"]),
        "initial_plate_pos": list(objects[plate]["pos"]),
        "candidate_scores": scores,
        "target_site": target + "_default_site",
        "destination_site": plate + "_default_site",
    }
    if target + "_default_site" in sites:
        grounding["initial_target_site_pos"] = list(sites[target + "_default_site"])
    if plate + "_default_site" in sites:
        grounding["initial_destination_site_pos"] = list(sites[plate + "_default_site"])

    held = False
    offset = [0.0, 0.0, 0.0]
    grasp_yaw = state["eef_yaw"]
    attempts = []

    for attempt in range(3):
        if remaining() < 205:
            break

        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        eef = list(state["eef_pos"])
        yaw = state["objects"][target]["yaw"]
        if attempt == 1:
            yaw += pi / 2
        grasp_yaw = nearest_yaw(yaw, state["eef_yaw"])
        clearance = max(eef[2], bowl[2] + 0.16)

        grip("open", 8)
        move([eef[0], eef[1], clearance], grasp_yaw, "open", 22)
        state = sdk.get_state()
        if state["eef_pos"][2] < bowl[2] + 0.10:
            continue
        move([bowl[0], bowl[1], clearance], grasp_yaw, "open", 35)

        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        if distance(state["eef_pos"], bowl) > 0.015:
            continue

        grasp_height = [0.045, 0.035, 0.055][attempt]
        move(
            [bowl[0], bowl[1], bowl[2] + 0.085],
            grasp_yaw, "open", 20
        )
        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        goal = [bowl[0], bowl[1], bowl[2] + grasp_height]
        move(goal, grasp_yaw, "open", 26, 0.004)

        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        eef = list(state["eef_pos"])
        if distance(eef, bowl) > 0.014 or abs(eef[2] - goal[2]) > 0.012:
            attempts.append({"attempt": attempt, "reason": "Grasp pose not reached"})
            continue

        initial_z = bowl[2]
        closure = grip("close", 18)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        move(
            [eef[0], eef[1], eef[2] + 0.055],
            grasp_yaw, "close", 23, 0.005
        )
        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        eef = list(state["eef_pos"])
        lift = bowl[2] - initial_z
        attempts.append({
            "attempt": attempt,
            "grasp_height": grasp_height,
            "lift": lift,
            "closure": closure,
        })
        if lift < 0.025:
            grip("open", 8)
            move(
                [eef[0], eef[1], max(eef[2], initial_z + 0.15)],
                grasp_yaw, "open", 18
            )
            continue

        first_offset = [bowl[i] - eef[i] for i in range(3)]
        move(
            [eef[0], eef[1], eef[2] + 0.075],
            grasp_yaw, "close", 22
        )
        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        eef = list(state["eef_pos"])
        offset = [bowl[i] - eef[i] for i in range(3)]
        drift = sqrt(sum(
            (offset[i] - first_offset[i]) ** 2 for i in range(3)
        ))
        if bowl[2] > initial_z + 0.08 and drift < 0.025:
            held = True
            break
        grip("open", 8)

    if not held:
        return {
            "status": "failed",
            "reason": "Could not verify a stable grasp",
            "grounding": grounding,
            "attempts": attempts,
        }

    state = sdk.get_state()
    plate_pos = list(state["objects"][plate]["pos"])
    eef = list(state["eef_pos"])
    transit_z = max(eef[2], plate_pos[2] + 0.16 - offset[2])
    move([eef[0], eef[1], transit_z], grasp_yaw, "close", 15)
    move(
        [plate_pos[0] - offset[0], plate_pos[1] - offset[1], transit_z],
        grasp_yaw, "close", 42
    )

    for correction in range(2):
        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        plate_pos = list(state["objects"][plate]["pos"])
        eef = list(state["eef_pos"])
        current_offset = [bowl[i] - eef[i] for i in range(3)]
        drift = sqrt(sum(
            (current_offset[i] - offset[i]) ** 2 for i in range(3)
        ))
        if drift > 0.035:
            return {
                "status": "failed",
                "reason": "Bowl slipped during transport",
                "grounding": grounding,
            }
        offset = current_offset
        if distance(bowl, plate_pos) < 0.006:
            break
        if remaining() < 80:
            break
        move(
            [
                eef[0] + plate_pos[0] - bowl[0],
                eef[1] + plate_pos[1] - bowl[1],
                transit_z,
            ],
            grasp_yaw, "close", 16, 0.004
        )

    state = sdk.get_state()
    bowl = list(state["objects"][target]["pos"])
    plate_pos = list(state["objects"][plate]["pos"])
    eef = list(state["eef_pos"])
    if distance(bowl, plate_pos) > 0.018 or remaining() < 65:
        return {
            "status": "failed",
            "reason": "Placement alignment or budget insufficient",
            "grounding": grounding,
        }

    offset = [bowl[i] - eef[i] for i in range(3)]
    move(
        [
            plate_pos[0] - offset[0],
            plate_pos[1] - offset[1],
            plate_pos[2] + 0.045 - offset[2],
        ],
        grasp_yaw, "close", 25, 0.004
    )
    state = sdk.get_state()
    bowl = list(state["objects"][target]["pos"])
    plate_pos = list(state["objects"][plate]["pos"])
    eef = list(state["eef_pos"])
    move(
        [
            eef[0] + plate_pos[0] - bowl[0],
            eef[1] + plate_pos[1] - bowl[1],
            eef[2] + plate_pos[2] + 0.018 - bowl[2],
        ],
        grasp_yaw, "close", 18, 0.004
    )

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    if distance(bowl, plate_pos) > 0.022 or bowl[2] - plate_pos[2] > 0.06:
        return {
            "status": "failed",
            "reason": "Release pose not reached",
            "grounding": grounding,
        }

    grip("open", 10)
    state = sdk.get_state()
    eef = list(state["eef_pos"])
    move(
        [eef[0], eef[1], eef[2] + 0.12],
        grasp_yaw, "open", min(20, max(0, remaining() - 8)), 0.008
    )
    grip("open", 8)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    horizontal_error = distance(bowl, plate_pos)
    height_difference = bowl[2] - plate_pos[2]
    placed = horizontal_error < 0.035 and -0.015 < height_difference < 0.055
    return {
        "status": "done" if placed else "failed",
        "grounding": grounding,
        "attempts": attempts,
        "final_target_pos": list(bowl),
        "final_plate_pos": list(plate_pos),
        "horizontal_error_m": horizontal_error,
        "height_above_plate_m": height_difference,
    }
