def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, grip, budget=55, tol=0.008):
        available = remaining()
        if available < 1:
            return False
        result = sdk.move_to(
            pos,
            yaw=yaw,
            gripper=grip,
            max_steps=min(budget, available),
            tol=tol,
        )
        return result["reached"]

    def gripper(action, steps=12):
        available = remaining()
        if available > 0:
            sdk.set_gripper(action, steps=min(steps, available))

    def distance_xy(a, b):
        return hypot(a[0] - b[0], a[1] - b[1])

    state = sdk.get_state()
    objects = state["objects"]
    bowls = []
    boxes = []
    plates = []
    for name in objects:
        if "black_bowl" in name:
            bowls.append(name)
        if "cookie" in name:
            boxes.append(name)
        if "plate" in name:
            plates.append(name)

    if not bowls or not boxes or not plates:
        return {"status": "failed", "reason": "Required objects unavailable"}

    bowl_name = bowls[0]
    best_score = 1000000.0
    for bowl in bowls:
        bp = objects[bowl]["pos"]
        for box in boxes:
            cp = objects[box]["pos"]
            score = distance_xy(bp, cp) + 0.25 * abs(bp[2] - cp[2])
            if bp[2] < cp[2] - 0.02:
                score += 1.0
            if score < best_score:
                best_score = score
                bowl_name = bowl

    plate_name = plates[0]
    held = False
    grasp_yaw = state["eef_yaw"]

    for attempt in range(2):
        if remaining() < 170:
            break

        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        plate = state["objects"][plate_name]["pos"]
        grasp_yaw = state["eef_yaw"] + attempt * pi / 2.0
        approach_z = max(bowl[2] + 0.15, plate[2] + 0.16)
        eef = state["eef_pos"]

        if not move(
            [eef[0], eef[1], max(eef[2], approach_z)],
            grasp_yaw,
            "open",
            35,
        ):
            continue

        gripper("open", 8)
        if not move(
            [bowl[0], bowl[1], approach_z], grasp_yaw, "open", 45
        ):
            continue

        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        grasp_z = bowl[2] + (0.025 if attempt == 0 else 0.012)
        if not move(
            [bowl[0], bowl[1], grasp_z], grasp_yaw, "open", 45, 0.005
        ):
            continue

        gripper("close", 16)
        state = sdk.get_state()
        before_lift = list(state["objects"][bowl_name]["pos"])
        eef = state["eef_pos"]
        move(
            [eef[0], eef[1], eef[2] + 0.12],
            grasp_yaw,
            "close",
            45,
        )
        state = sdk.get_state()
        lifted = state["objects"][bowl_name]["pos"]
        eef = state["eef_pos"]
        if (
            lifted[2] - before_lift[2] > 0.055
            and distance_xy(lifted, eef) < 0.07
        ):
            held = True
            break
        gripper("open", 8)

    if not held:
        return {"status": "failed", "reason": "Bowl did not follow the grasp"}

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    eef = state["eef_pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    plate = state["objects"][plate_name]["pos"]
    carry_z = max(eef[2], plate[2] + 0.16 - offset[2])

    if not move(
        [eef[0], eef[1], carry_z], grasp_yaw, "close", 25
    ):
        return {"status": "failed", "reason": "Could not establish clearance"}

    if not move(
        [plate[0] - offset[0], plate[1] - offset[1], carry_z],
        grasp_yaw,
        "close",
        55,
    ):
        return {"status": "failed", "reason": "Could not reach plate"}

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    eef = state["eef_pos"]
    if bowl[2] < plate[2] + 0.045 or distance_xy(bowl, eef) > 0.08:
        return {"status": "failed", "reason": "Grasp lost during transport"}

    offset = [bowl[i] - eef[i] for i in range(3)]
    plate = state["objects"][plate_name]["pos"]
    release_pos = [
        plate[0] - offset[0],
        plate[1] - offset[1],
        plate[2] + 0.018 - offset[2],
    ]

    if remaining() < 35:
        return {"status": "failed", "reason": "Insufficient time for placement"}

    move(release_pos, grasp_yaw, "close", min(40, remaining() - 25), 0.006)
    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if distance_xy(bowl, plate) > 0.035 or bowl[2] > plate[2] + 0.06:
        return {"status": "failed", "reason": "Placement alignment failed"}

    gripper("open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.12],
        grasp_yaw,
        "open",
        30,
    )
    gripper("open", 10)

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    placed = (
        distance_xy(bowl, plate) < 0.045
        and plate[2] - 0.015 <= bowl[2] <= plate[2] + 0.055
        and state["eef_pos"][2] - bowl[2] > 0.06
    )
    return {
        "status": "done" if placed else "failed",
        "object": bowl_name,
        "target": plate_name,
    }
