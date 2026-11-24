def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return state["max_steps"] - state["steps_used"]

    def move(pos, yaw, grip, limit=40, tol=0.008):
        budget = remaining()
        if budget <= 0:
            return False
        result = sdk.move_to(
            pos, yaw=yaw, gripper=grip,
            max_steps=min(limit, budget), tol=tol
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
    plates = [name for name in objects if "plate" in name]
    bowls = [name for name in objects if "black" in name and "bowl" in name]
    if not plates or not bowls:
        return {"status": "failed", "reason": "Required objects unavailable"}

    plate_name = plates[0]
    reference = state["sites"].get("main_table_plate_region")
    if reference is not None:
        best = float("inf")
        for name in plates:
            pos = objects[name]["pos"]
            distance = hypot(pos[0] - reference[0], pos[1] - reference[1])
            if distance < best:
                plate_name = name
                best = distance

    reference = state["sites"].get(
        "main_table_next_to_plate_region", objects[plate_name]["pos"]
    )
    bowl_name = bowls[0]
    best = float("inf")
    for name in bowls:
        pos = objects[name]["pos"]
        distance = hypot(pos[0] - reference[0], pos[1] - reference[1])
        if distance < best:
            bowl_name = name
            best = distance

    # Pinch one rim: closing across the bowl's center can exceed jaw travel.
    initial_yaw = state["eef_yaw"]
    held = False
    for attempt in range(3):
        if remaining() < 155:
            break

        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        plate = state["objects"][plate_name]["pos"]
        eef = state["eef_pos"]
        yaw = initial_yaw + (pi / 2 if attempt == 2 else 0.0)
        clearance = max(bowl[2], plate[2]) + 0.14

        move(
            [eef[0], eef[1], max(eef[2], clearance)],
            state["eef_yaw"], "open", 25
        )
        grip("open", 6)

        radius = [0.043, -0.043, 0.05][attempt]
        height = [0.018, 0.008, 0.012][attempt]
        dx = -sin(yaw) * radius
        dy = cos(yaw) * radius

        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        if not move(
            [bowl[0] + dx, bowl[1] + dy, clearance],
            yaw, "open", 35
        ):
            continue

        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        target = [bowl[0] + dx, bowl[1] + dy, bowl[2] + height]
        move(target, yaw, "open", 30, 0.005)
        state = sdk.get_state()
        eef = state["eef_pos"]
        if hypot(eef[0] - target[0], eef[1] - target[1]) > 0.018:
            continue
        if abs(eef[2] - target[2]) > 0.025:
            continue

        start = list(state["objects"][bowl_name]["pos"])
        grip("close", 16)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move([eef[0], eef[1], eef[2] + 0.09], yaw, "close", 30)

        state = sdk.get_state()
        bowl = state["objects"][bowl_name]["pos"]
        if bowl[2] - start[2] > 0.045:
            held = True
            break

    if not held:
        return {"status": "failed", "reason": "Rim pickup not verified"}

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    eef = state["eef_pos"]
    plate = state["objects"][plate_name]["pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    carry_z = max(eef[2], plate[2] + 0.13 - offset[2])

    move([eef[0], eef[1], carry_z], yaw, "close", 20)
    move(
        [plate[0] - offset[0], plate[1] - offset[1], carry_z],
        yaw, "close", 45
    )

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    if bowl[2] < plate[2] + 0.055:
        return {"status": "failed", "reason": "Bowl lost during transport"}

    # Correct using the actual held-object offset before descending.
    offset = [bowl[i] - eef[i] for i in range(3)]
    target = [
        plate[0] - offset[0],
        plate[1] - offset[1],
        eef[2],
    ]
    move(target, yaw, "close", 20, 0.005)

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if hypot(bowl[0] - plate[0], bowl[1] - plate[1]) > 0.025:
        return {"status": "failed", "reason": "Bowl not aligned over plate"}

    eef = state["eef_pos"]
    target = [
        eef[0] + plate[0] - bowl[0],
        eef[1] + plate[1] - bowl[1],
        eef[2] + plate[2] + 0.035 - bowl[2],
    ]
    move(target, yaw, "close", 35, 0.006)
    if not grip("open", 14):
        return {"status": "failed", "reason": "No time to release"}

    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.12], yaw, "open", 25)
    grip("open", 12)

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if (
        hypot(bowl[0] - plate[0], bowl[1] - plate[1]) < 0.045
        and -0.015 <= bowl[2] - plate[2] <= 0.065
        and state["gripper_width"] > 0.045
    ):
        return {"status": "done", "object": bowl_name, "target": plate_name}
    return {"status": "failed", "reason": "Final placement not verified"}
