def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, grip, budget=45, tol=0.012):
        budget = min(budget, remaining())
        if budget <= 0:
            return False
        result = sdk.move_to(
            pos, yaw=yaw, gripper=grip, max_steps=budget, tol=tol
        )
        return result["reached"]

    def grip(action, budget=12):
        budget = min(budget, remaining())
        if budget > 0:
            return sdk.set_gripper(action, steps=budget)
        return {}

    state = sdk.get_state()
    objects = state["objects"]
    bowls = [
        name for name in objects
        if "black" in name.lower() and "bowl" in name.lower()
    ]
    cookies = [name for name in objects if "cookie" in name.lower()]
    plates = [name for name in objects if "plate" in name.lower()]
    if not bowls or not cookies or not plates:
        return {"status": "failed", "reason": "Required objects are missing"}

    target = bowls[0]
    best_distance = float("inf")
    for bowl in bowls:
        pos = objects[bowl]["pos"]
        for cookie in cookies:
            box = objects[cookie]["pos"]
            distance = hypot(pos[0] - box[0], pos[1] - box[1])
            if distance < best_distance:
                best_distance = distance
                target = bowl

    plate = plates[0]
    original_pos = list(objects[target]["pos"])
    original_yaw = objects[target]["yaw"]
    held = False
    grasp_yaw = original_yaw

    for attempt in range(3):
        if remaining() < 160:
            break
        state = sdk.get_state()
        bowl_pos = list(state["objects"][target]["pos"])
        plate_pos = state["objects"][plate]["pos"]
        grasp_yaw = original_yaw + (pi / 2 if attempt == 1 else 0.0)
        grasp_height = [0.025, 0.012, 0.04][attempt]
        clearance = max(
            state["eef_pos"][2], bowl_pos[2] + 0.17, plate_pos[2] + 0.17
        )

        grip("open", 8)
        eef = sdk.get_state()["eef_pos"]
        if not move([eef[0], eef[1], clearance], grasp_yaw, "open", 30):
            continue
        if not move(
            [bowl_pos[0], bowl_pos[1], clearance],
            grasp_yaw, "open", 45
        ):
            continue

        bowl_pos = list(sdk.get_state()["objects"][target]["pos"])
        if not move(
            [bowl_pos[0], bowl_pos[1], bowl_pos[2] + grasp_height],
            grasp_yaw, "open", 40, 0.008
        ):
            continue

        grip("close", 15)
        before = sdk.get_state()
        before_bowl = list(before["objects"][target]["pos"])
        eef = before["eef_pos"]
        move(
            [eef[0], eef[1], eef[2] + 0.12],
            grasp_yaw, "close", 35, 0.01
        )
        lifted = sdk.get_state()
        lifted_bowl = lifted["objects"][target]["pos"]
        rise = lifted_bowl[2] - before_bowl[2]
        if rise > 0.055:
            eef = lifted["eef_pos"]
            offset = [lifted_bowl[i] - eef[i] for i in range(3)]
            move(
                [eef[0], eef[1], eef[2] + 0.025],
                grasp_yaw, "close", 12, 0.008
            )
            checked = sdk.get_state()
            checked_bowl = checked["objects"][target]["pos"]
            checked_eef = checked["eef_pos"]
            offset_error = sqrt(sum(
                (checked_bowl[i] - checked_eef[i] - offset[i]) ** 2
                for i in range(3)
            ))
            if offset_error < 0.025 and checked_bowl[2] > before_bowl[2] + 0.05:
                held = True
                break

    if not held:
        return {"status": "failed", "reason": "Could not verify a stable grasp"}

    state = sdk.get_state()
    eef = state["eef_pos"]
    bowl_pos = state["objects"][target]["pos"]
    offset = [bowl_pos[i] - eef[i] for i in range(3)]
    plate_pos = state["objects"][plate]["pos"]
    desired_bowl_z = plate_pos[2] + 0.025
    transit_z = max(eef[2], desired_bowl_z - offset[2] + 0.13)

    if not move(
        [eef[0], eef[1], transit_z], grasp_yaw, "close", 25
    ):
        return {"status": "failed", "reason": "Could not reach transport clearance"}

    plate_pos = sdk.get_state()["objects"][plate]["pos"]
    if not move(
        [plate_pos[0] - offset[0], plate_pos[1] - offset[1], transit_z],
        grasp_yaw, "close", 55, 0.01
    ):
        return {"status": "failed", "reason": "Could not reach the plate"}

    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    current_offset = [bowl_pos[i] - eef[i] for i in range(3)]
    if sqrt(sum(
        (current_offset[i] - offset[i]) ** 2 for i in range(3)
    )) > 0.04:
        return {"status": "failed", "reason": "Bowl slipped during transport"}

    plate_pos = state["objects"][plate]["pos"]
    release_pos = [
        plate_pos[0] - current_offset[0],
        plate_pos[1] - current_offset[1],
        plate_pos[2] + 0.025 - current_offset[2],
    ]
    if not move(release_pos, grasp_yaw, "close", 45, 0.008):
        return {"status": "failed", "reason": "Could not lower the bowl onto the plate"}

    if remaining() < 20:
        return {"status": "failed", "reason": "Insufficient time to release and settle"}

    grip("open", 12)
    eef = sdk.get_state()["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.12],
        grasp_yaw, "open", 25, 0.015
    )
    grip("open", 10)

    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    horizontal_error = hypot(
        bowl_pos[0] - plate_pos[0], bowl_pos[1] - plate_pos[1]
    )
    height_difference = bowl_pos[2] - plate_pos[2]
    placed = horizontal_error < 0.045 and -0.015 < height_difference < 0.07
    return {
        "status": "done" if placed else "failed",
        "target": target,
        "horizontal_error_m": horizontal_error,
        "height_above_plate_m": height_difference,
    }
