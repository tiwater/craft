def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def distance(a, b):
        return hypot(a[0] - b[0], a[1] - b[1])

    def move(pos, yaw, action, steps=40, tol=0.008):
        budget = min(steps, remaining())
        if budget <= 0:
            return False
        return sdk.move_to(
            pos, yaw=yaw, gripper=action, max_steps=budget, tol=tol
        )["reached"]

    def grip(action, steps=12):
        budget = min(steps, remaining())
        if budget > 0:
            return sdk.set_gripper(action, steps=budget)
        return {}

    state = sdk.get_state()
    objects = state["objects"]
    sites = state["sites"]
    bowls = [name for name in objects if "black_bowl" in name]
    ramekins = [name for name in objects if "ramekin" in name]
    plates = [name for name in objects if "plate" in name]
    if not bowls or not ramekins or not plates:
        return {"status": "failed", "reason": "Required objects unavailable"}

    region_name = "main_table_next_to_ramekin_region"
    region = sites.get(region_name)
    ranked = []
    for name in bowls:
        pos = objects[name]["pos"]
        near = min(distance(pos, objects[item]["pos"]) for item in ramekins)
        score = distance(pos, region) if region is not None else near
        ranked.append((score, near, name))
    ranked = sorted(ranked)
    target = ranked[0][2]

    plate_region = sites.get("main_table_plate_region")
    ranked_plates = []
    for name in plates:
        pos = objects[name]["pos"]
        score = distance(
            pos, plate_region if plate_region is not None else objects[target]["pos"]
        )
        ranked_plates.append((score, name))
    plate = sorted(ranked_plates)[0][1]

    ranked_ramekins = []
    for name in ramekins:
        ranked_ramekins.append(
            (distance(objects[name]["pos"], objects[target]["pos"]), name)
        )
    ramekin = sorted(ranked_ramekins)[0][1]
    initial_bowl = list(objects[target]["pos"])
    initial_plate = list(objects[plate]["pos"])
    grounding = {
        "target": target,
        "target_initial_pos": initial_bowl,
        "target_default_site": sites.get(target + "_default_site"),
        "selection_region": region_name if region is not None else None,
        "selection_region_pos": region,
        "ramekin": ramekin,
        "plate": plate,
        "plate_initial_pos": initial_plate,
        "plate_default_site": sites.get(plate + "_default_site"),
        "candidate_scores": ranked,
    }
    attempts = []
    grasped = False
    base_yaw = state["eef_yaw"]

    for attempt in range(2):
        if remaining() < 195:
            break
        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        nearby = state["objects"][ramekin]["pos"]
        direction = atan2(bowl[1] - nearby[1], bowl[0] - nearby[0])
        if attempt == 1:
            direction = direction + pi / 2.0
        radial = 0.035
        grasp_x = bowl[0] + radial * cos(direction)
        grasp_y = bowl[1] + radial * sin(direction)
        yaw = direction - pi / 2.0
        while yaw - base_yaw > pi / 2.0:
            yaw = yaw - pi
        while yaw - base_yaw < -pi / 2.0:
            yaw = yaw + pi

        clearance = max(bowl[2], state["objects"][plate]["pos"][2]) + 0.15
        eef = state["eef_pos"]
        move([eef[0], eef[1], max(eef[2], clearance)], yaw, "open", 30, 0.012)
        grip("open", 8)
        move([grasp_x, grasp_y, clearance], yaw, "open", 50, 0.008)

        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        grasp_x = bowl[0] + radial * cos(direction)
        grasp_y = bowl[1] + radial * sin(direction)
        grasp_z = bowl[2] + (0.035 if attempt == 0 else 0.025)
        move([grasp_x, grasp_y, bowl[2] + 0.075], yaw, "open", 24, 0.006)
        reached = move([grasp_x, grasp_y, grasp_z], yaw, "open", 32, 0.004)
        state = sdk.get_state()
        if not reached and distance(state["eef_pos"], [grasp_x, grasp_y]) > 0.015:
            attempts.append({"attempt": attempt, "reason": "Grasp approach misaligned"})
            continue
        if remaining() < 145:
            break

        before = list(state["objects"][target]["pos"])
        grip("close", 22)
        state = sdk.get_state()
        lift_start = list(state["eef_pos"])
        move(
            [lift_start[0], lift_start[1], lift_start[2] + 0.035],
            yaw, "close", 20, 0.006
        )
        move(
            [lift_start[0], lift_start[1], lift_start[2] + 0.09],
            yaw, "close", 25, 0.008
        )
        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        rise = bowl[2] - before[2]
        attempts.append({
            "attempt": attempt,
            "grasp_pos": [grasp_x, grasp_y, grasp_z],
            "yaw": yaw,
            "rise": rise,
            "width": state["gripper_width"],
        })
        if rise > 0.045 and distance(bowl, state["eef_pos"]) < 0.09:
            grasped = True
            break
        if rise > 0.015:
            eef = state["eef_pos"]
            move([eef[0], eef[1], grasp_z], yaw, "close", 18)
        grip("open", 8)

    if not grasped:
        return {
            "status": "failed",
            "reason": "Grounded bowl did not lift",
            "grounding": grounding,
            "attempts": attempts,
        }

    state = sdk.get_state()
    bowl = list(state["objects"][target]["pos"])
    plate_pos = list(state["objects"][plate]["pos"])
    eef = list(state["eef_pos"])
    safe_z = max(initial_bowl[2], plate_pos[2]) + 0.12
    move(
        [eef[0], eef[1], eef[2] + max(0.0, safe_z - bowl[2])],
        yaw, "close", 25, 0.008
    )

    for segment in range(3):
        if remaining() < 58:
            break
        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        plate_pos = list(state["objects"][plate]["pos"])
        eef = list(state["eef_pos"])
        if bowl[2] < initial_bowl[2] + 0.035:
            return {
                "status": "failed",
                "reason": "Bowl lost during transport",
                "grounding": grounding,
                "attempts": attempts,
            }
        if distance(bowl, plate_pos) < 0.01:
            break
        move(
            [
                eef[0] + plate_pos[0] - bowl[0],
                eef[1] + plate_pos[1] - bowl[1],
                eef[2] + max(0.0, safe_z - bowl[2]),
            ],
            yaw, "close", min(45, remaining() - 55), 0.006
        )

    state = sdk.get_state()
    bowl = list(state["objects"][target]["pos"])
    plate_pos = list(state["objects"][plate]["pos"])
    if distance(bowl, plate_pos) > 0.025 or remaining() < 38:
        return {
            "status": "failed",
            "reason": "Placement alignment insufficient",
            "grounding": grounding,
            "attempts": attempts,
        }

    for height in [0.045, 0.018]:
        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        plate_pos = list(state["objects"][plate]["pos"])
        eef = list(state["eef_pos"])
        move(
            [
                eef[0] + plate_pos[0] - bowl[0],
                eef[1] + plate_pos[1] - bowl[1],
                eef[2] + plate_pos[2] + height - bowl[2],
            ],
            yaw, "close", min(25, remaining() - 24), 0.005
        )

    state = sdk.get_state()
    bowl = list(state["objects"][target]["pos"])
    plate_pos = list(state["objects"][plate]["pos"])
    if distance(bowl, plate_pos) > 0.03 or bowl[2] > plate_pos[2] + 0.05:
        return {
            "status": "failed",
            "reason": "Release pose unavailable",
            "grounding": grounding,
            "attempts": attempts,
        }

    grip("open", 10)
    state = sdk.get_state()
    eef = list(state["eef_pos"])
    move(
        [eef[0], eef[1], eef[2] + 0.10],
        yaw, "open", min(22, remaining() - 4), 0.012
    )
    grip("open", 4)
    state = sdk.get_state()
    bowl = list(state["objects"][target]["pos"])
    plate_pos = list(state["objects"][plate]["pos"])
    placed = (
        distance(bowl, plate_pos) < 0.04
        and plate_pos[2] - 0.01 <= bowl[2] <= plate_pos[2] + 0.05
        and abs(plate_pos[2] - initial_plate[2]) < 0.025
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Placement verified from poses" if placed else "Placement unverified",
        "grounding": grounding,
        "attempts": attempts,
        "bowl_final_pos": bowl,
        "plate_final_pos": plate_pos,
    }
