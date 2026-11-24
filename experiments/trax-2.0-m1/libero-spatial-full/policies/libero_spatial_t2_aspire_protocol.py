def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, action, steps, tol=0.012):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.move_to(pos, yaw=yaw, gripper=action,
                        max_steps=budget, tol=tol)
        return sdk.get_state()

    def grip(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)
        return sdk.get_state()

    def distance(a, b):
        return hypot(a[0] - b[0], a[1] - b[1])

    state = sdk.get_state()
    objects = state["objects"]
    bowls = [name for name in objects if "black_bowl" in name]
    plates = [name for name in objects if "plate" in name]
    if not bowls or not plates:
        return {"status": "failed", "reason": "Required objects unavailable"}

    center = state["sites"].get("table_center")
    if center is None:
        for name in state["sites"]:
            if "table" in name and "center" in name:
                center = state["sites"][name]
                break
    if center is None:
        center = state["sites"].get("table_top")
    if center is None:
        for name in state["fixtures"]:
            if "table" in name:
                center = state["fixtures"][name]["pos"]
                break
    if center is None:
        return {"status": "failed", "reason": "Table reference unavailable"}

    bowl_name = bowls[0]
    for name in bowls:
        if distance(objects[name]["pos"], center) < distance(objects[bowl_name]["pos"], center):
            bowl_name = name

    plate_name = plates[0]
    for name in plates:
        if distance(objects[name]["pos"], center) < distance(objects[plate_name]["pos"], center):
            plate_name = name

    initial_bowl_z = objects[bowl_name]["pos"][2]
    initial_plate_z = objects[plate_name]["pos"][2]
    resting_difference = initial_bowl_z - initial_plate_z
    initial_yaw = state["eef_yaw"]
    held = False
    offset = [0.0, 0.0, 0.0]

    # Pinch one wall of the bowl instead of spanning its full diameter.
    # Panda fingers close along the wrist's local y axis.
    for attempt in range(3):
        if remaining() < 155:
            break

        yaw = initial_yaw
        radius = 0.05
        height = 0.018
        if attempt == 1:
            radius = -0.05
            height = 0.008
        elif attempt == 2:
            yaw = initial_yaw + pi / 2.0
            radius = 0.06
            height = 0.025

        state = sdk.get_state()
        bowl = state["objects"][bowl_name]["pos"]
        eef = state["eef_pos"]
        hover_z = bowl[2] + 0.115

        if eef[2] < hover_z - 0.02:
            state = move([eef[0], eef[1], hover_z],
                         state["eef_yaw"], "open", 22)
        state = grip("open", 7)
        bowl = state["objects"][bowl_name]["pos"]
        dx = -sin(yaw) * radius
        dy = cos(yaw) * radius
        target = [bowl[0] + dx, bowl[1] + dy, hover_z]
        state = move(target, yaw, "open", 35)
        if distance(state["eef_pos"], target) > 0.025:
            continue

        bowl = state["objects"][bowl_name]["pos"]
        target = [bowl[0] + dx, bowl[1] + dy, bowl[2] + height]
        state = move(target, yaw, "open", 30, 0.006)
        if distance(state["eef_pos"], target) > 0.02:
            continue
        if abs(state["eef_pos"][2] - target[2]) > 0.025:
            continue

        start_z = state["objects"][bowl_name]["pos"][2]
        state = grip("close", 12)
        eef = state["eef_pos"]
        before_offset = [
            state["objects"][bowl_name]["pos"][i] - eef[i]
            for i in range(3)
        ]

        state = move([eef[0], eef[1], eef[2] + 0.075],
                     yaw, "close", 25, 0.008)
        bowl = state["objects"][bowl_name]["pos"]
        eef = state["eef_pos"]
        after_offset = [bowl[i] - eef[i] for i in range(3)]
        lifted = bowl[2] - start_z

        if lifted > 0.035 and distance(before_offset, after_offset) < 0.025:
            state = move([eef[0], eef[1], eef[2] + 0.035],
                         yaw, "close", 15, 0.008)
            bowl = state["objects"][bowl_name]["pos"]
            eef = state["eef_pos"]
            confirmed_offset = [bowl[i] - eef[i] for i in range(3)]
            if (
                bowl[2] > start_z + 0.055
                and abs(confirmed_offset[2] - after_offset[2]) < 0.02
                and distance(confirmed_offset, after_offset) < 0.02
            ):
                held = True
                offset = confirmed_offset
                break

    if not held:
        return {"status": "failed", "reason": "Rim grasp did not lift bowl"}

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    resting_z = plate[2] + resting_difference + 0.012
    transit_z = max(bowl[2], resting_z + 0.09)
    state = move(
        [plate[0] - offset[0], plate[1] - offset[1], transit_z - offset[2]],
        yaw, "close", min(55, max(0, remaining() - 58)), 0.009
    )

    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    current_offset = [bowl[i] - eef[i] for i in range(3)]
    if (
        abs(current_offset[2] - offset[2]) > 0.03
        or distance(current_offset, offset) > 0.03
    ):
        return {"status": "failed", "reason": "Bowl slipped during transport"}

    if distance(bowl, plate) > 0.018 and remaining() > 65:
        state = move(
            [eef[0] + plate[0] - bowl[0],
             eef[1] + plate[1] - bowl[1], eef[2]],
            yaw, "close", 20, 0.006
        )

    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    if distance(bowl, plate) > 0.035:
        return {"status": "failed", "reason": "Plate alignment failed"}

    offset = [bowl[i] - eef[i] for i in range(3)]
    resting_z = plate[2] + resting_difference + 0.012
    state = move(
        [plate[0] - offset[0], plate[1] - offset[1],
         resting_z + 0.006 - offset[2]],
        yaw, "close", min(35, max(0, remaining() - 28)), 0.006
    )
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if distance(bowl, plate) > 0.035 or bowl[2] > resting_z + 0.035:
        return {"status": "failed", "reason": "Placement descent failed"}

    state = grip("open", 10)
    eef = state["eef_pos"]
    state = move([eef[0], eef[1], eef[2] + 0.10],
                 yaw, "open", min(20, max(0, remaining() - 8)), 0.015)
    if remaining() > 0:
        sdk.step_raw([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
                     steps=min(12, remaining()))

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    placed = (
        distance(bowl, plate) < 0.035
        and abs(bowl[2] - (plate[2] + resting_difference + 0.012)) < 0.035
        and state["gripper_width"] > 0.05
        and state["eef_pos"][2] > bowl[2] + 0.06
    )
    return {
        "status": "done" if placed else "failed",
        "object": bowl_name,
        "target": plate_name,
        "placement_verified": placed
    }
