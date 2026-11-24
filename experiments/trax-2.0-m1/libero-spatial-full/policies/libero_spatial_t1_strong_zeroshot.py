def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, grip, steps=45, tol=0.012):
        budget = min(steps, remaining())
        if budget < 1:
            return False
        result = sdk.move_to(
            pos, yaw=yaw, gripper=grip, max_steps=budget, tol=tol
        )
        return result["reached"]

    def grip(action, steps=12):
        budget = min(steps, remaining())
        if budget > 0:
            return sdk.set_gripper(action, steps=budget)
        return {}

    def distance(a, b):
        return hypot(a[0] - b[0], a[1] - b[1])

    state = sdk.get_state()
    objects = state["objects"]
    bowls = [name for name in objects if "black_bowl" in name]
    ramekins = [name for name in objects if "ramekin" in name]
    plates = [name for name in objects if "plate" in name]

    if not bowls or not ramekins or not plates:
        return {"status": "failed", "reason": "Required objects unavailable"}

    region = state["sites"].get("main_table_next_to_ramekin_region")
    target = bowls[0]
    best_score = float("inf")
    for name in bowls:
        pos = objects[name]["pos"]
        nearby = min(
            distance(pos, objects[ramekin]["pos"]) for ramekin in ramekins
        )
        score = nearby
        if region is not None:
            score += 2.0 * distance(pos, region)
        if score < best_score:
            best_score = score
            target = name

    plate = plates[0]
    plate_region = state["sites"].get("main_table_plate_region")
    if plate_region is not None:
        best_score = float("inf")
        for name in plates:
            score = distance(objects[name]["pos"], plate_region)
            if score < best_score:
                best_score = score
                plate = name

    initial_pos = list(objects[target]["pos"])
    grasped = False
    yaw = state["eef_yaw"]

    for attempt in range(2):
        if remaining() < 155:
            break

        state = sdk.get_state()
        bowl_pos = list(state["objects"][target]["pos"])
        plate_pos = state["objects"][plate]["pos"]
        yaw = state["eef_yaw"] + attempt * pi / 2.0
        clearance = max(bowl_pos[2], plate_pos[2]) + 0.17
        eef = state["eef_pos"]

        move([eef[0], eef[1], max(eef[2], clearance)],
             yaw, "open", 30)
        grip("open", 8)

        if not move([bowl_pos[0], bowl_pos[1], clearance],
                    yaw, "open", 45):
            continue

        state = sdk.get_state()
        bowl_pos = list(state["objects"][target]["pos"])
        grasp_z = bowl_pos[2] + (0.025 if attempt == 0 else 0.012)
        if not move([bowl_pos[0], bowl_pos[1], grasp_z],
                    yaw, "open", 40, 0.006):
            continue

        grip("close", 16)
        state = sdk.get_state()
        before_lift = list(state["objects"][target]["pos"])
        eef = list(state["eef_pos"])

        move([eef[0], eef[1], eef[2] + 0.11],
             yaw, "close", 35, 0.008)
        state = sdk.get_state()
        lifted = state["objects"][target]["pos"]
        if (lifted[2] - before_lift[2] > 0.045
                and distance(lifted, state["eef_pos"]) < 0.075):
            grasped = True
            break

        # Return to the pickup height before releasing a marginal grasp.
        eef = state["eef_pos"]
        move([eef[0], eef[1], grasp_z], yaw, "close", 22)
        grip("open", 8)

    if not grasped:
        return {"status": "failed", "reason": "Bowl did not lift"}

    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    offset = [bowl_pos[i] - eef[i] for i in range(3)]
    plate_pos = state["objects"][plate]["pos"]
    travel_z = max(initial_pos[2], plate_pos[2]) + 0.18
    travel_z = max(travel_z, eef[2])

    if not move([eef[0], eef[1], travel_z], yaw, "close", 25):
        return {"status": "failed", "reason": "Could not reach clearance"}

    if not move(
        [plate_pos[0] - offset[0], plate_pos[1] - offset[1], travel_z],
        yaw, "close", 55, 0.008
    ):
        return {"status": "failed", "reason": "Could not align above plate"}

    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    if bowl_pos[2] < initial_pos[2] + 0.04:
        return {"status": "failed", "reason": "Bowl lost during transport"}

    offset = [bowl_pos[i] - eef[i] for i in range(3)]
    plate_pos = state["objects"][plate]["pos"]
    release_pos = [
        plate_pos[0] - offset[0],
        plate_pos[1] - offset[1],
        plate_pos[2] + 0.025 - offset[2],
    ]

    if remaining() < 30:
        return {"status": "failed", "reason": "Insufficient placement budget"}

    if not move(release_pos, yaw, "close",
                min(40, remaining() - 22), 0.008):
        return {"status": "failed", "reason": "Could not reach release pose"}

    grip("open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.12],
         yaw, "open", 25)
    grip("open", 10)

    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    placed = (
        distance(bowl_pos, plate_pos) < 0.045
        and plate_pos[2] - 0.02 <= bowl_pos[2] <= plate_pos[2] + 0.065
    )
    return {
        "status": "done" if placed else "failed",
        "target": target,
        "reason": "Placement verified from poses" if placed else "Placement unverified",
    }
