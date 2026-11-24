def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, limit=45, tol=0.008):
        budget = min(limit, remaining())
        if budget <= 0:
            return False
        result = sdk.move_to(pos, yaw=yaw, max_steps=budget, tol=tol)
        return result["reached"]

    def grip(action, steps=10):
        budget = min(steps, remaining())
        if budget > 0:
            return sdk.set_gripper(action, steps=budget)
        return {}

    def distance_xy(a, b):
        return hypot(a[0] - b[0], a[1] - b[1])

    state = sdk.get_state()
    objects = state["objects"]
    bowls = [name for name in objects if "black_bowl" in name]
    plates = [name for name in objects if "plate" in name]
    if not bowls or not plates:
        return {"status": "failed", "reason": "Required objects unavailable"}

    center = state["sites"].get("table_top")
    if center is None:
        return {"status": "failed", "reason": "Table center unavailable"}

    bowl_name = bowls[0]
    for name in bowls:
        if distance_xy(objects[name]["pos"], center) < distance_xy(
            objects[bowl_name]["pos"], center
        ):
            bowl_name = name

    plate_name = plates[0]
    for name in plates:
        if distance_xy(objects[name]["pos"], objects[bowl_name]["pos"]) < distance_xy(
            objects[plate_name]["pos"], objects[bowl_name]["pos"]
        ):
            plate_name = name

    initial_bowl_z = objects[bowl_name]["pos"][2]
    initial_plate_z = objects[plate_name]["pos"][2]
    held = False
    grasp_yaw = state["eef_yaw"]

    for attempt in range(3):
        if remaining() < 155:
            break

        state = sdk.get_state()
        bowl = state["objects"][bowl_name]["pos"]
        start_z = bowl[2]
        grasp_yaw = state["objects"][bowl_name]["yaw"]
        if attempt == 1:
            grasp_yaw += pi / 2.0

        offset_x = 0.0
        offset_y = 0.0
        grasp_height = 0.018
        if attempt == 1:
            grasp_height = 0.008
        if attempt == 2:
            offset_x = 0.032 * cos(grasp_yaw)
            offset_y = 0.032 * sin(grasp_yaw)
            grasp_height = 0.022

        eef = state["eef_pos"]
        safe_z = max(eef[2], bowl[2] + 0.17,
                     state["objects"][plate_name]["pos"][2] + 0.17)
        if not move([eef[0], eef[1], safe_z], grasp_yaw, 30):
            continue
        grip("open", 8)

        if not move(
            [bowl[0] + offset_x, bowl[1] + offset_y, safe_z],
            grasp_yaw, 40
        ):
            continue

        state = sdk.get_state()
        bowl = state["objects"][bowl_name]["pos"]
        if not move(
            [bowl[0] + offset_x, bowl[1] + offset_y,
             bowl[2] + grasp_height],
            grasp_yaw, 40, 0.005
        ):
            continue

        grip("close", 12)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move([eef[0], eef[1], eef[2] + 0.12], grasp_yaw, 35)

        state = sdk.get_state()
        bowl = state["objects"][bowl_name]["pos"]
        if bowl[2] > start_z + 0.055:
            held = True
            break
        grip("open", 8)

    if not held:
        return {"status": "failed", "reason": "Could not verify a lifted bowl"}

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    eef = state["eef_pos"]
    plate = state["objects"][plate_name]["pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]

    # Preserve the measured grasp offset when aligning the bowl over the plate.
    clearance_z = max(bowl[2], plate[2] + 0.15)
    target = [
        plate[0] - offset[0],
        plate[1] - offset[1],
        clearance_z - offset[2]
    ]
    if not move(target, grasp_yaw, 55):
        return {"status": "failed", "reason": "Could not reach above the plate"}

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    eef = state["eef_pos"]
    plate = state["objects"][plate_name]["pos"]
    if bowl[2] < initial_bowl_z + 0.045:
        return {"status": "failed", "reason": "Bowl slipped during transport"}

    offset = [bowl[i] - eef[i] for i in range(3)]
    # Relative object-origin heights account for their different mesh origins.
    resting_z = plate[2] + (initial_bowl_z - initial_plate_z) + 0.015
    target = [
        plate[0] - offset[0],
        plate[1] - offset[1],
        resting_z + 0.012 - offset[2]
    ]

    if remaining() < 30:
        return {"status": "failed", "reason": "Insufficient placement budget"}

    reached = move(target, grasp_yaw, min(45, remaining() - 24), 0.006)
    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if distance_xy(bowl, plate) > 0.035 or bowl[2] > resting_z + 0.055:
        return {"status": "failed", "reason": "Placement alignment not verified"}

    grip("open", 10)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.12], grasp_yaw,
         min(25, max(0, remaining() - 6)))
    if remaining() > 0:
        sdk.step_raw([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
                     steps=min(10, remaining()))

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    expected_z = plate[2] + (initial_bowl_z - initial_plate_z) + 0.015
    placed = (
        distance_xy(bowl, plate) < 0.04
        and abs(bowl[2] - expected_z) < 0.035
        and state["gripper_width"] > 0.05
        and state["eef_pos"][2] - bowl[2] > 0.07
    )
    return {
        "status": "done" if placed else "failed",
        "object": bowl_name,
        "target": plate_name,
        "placement_verified": placed
    }
