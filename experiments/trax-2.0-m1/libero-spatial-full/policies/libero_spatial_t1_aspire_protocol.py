def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def distance(a, b):
        return hypot(a[0] - b[0], a[1] - b[1])

    def move(pos, yaw, action, steps=35, tol=0.012):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.move_to(pos, yaw=yaw, gripper=action,
                        max_steps=budget, tol=tol)

    def grip(action, steps=10):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    state = sdk.get_state()
    objects = state["objects"]
    bowls = [name for name in objects if "black_bowl" in name]
    ramekins = [name for name in objects if "ramekin" in name]
    plates = [name for name in objects if "plate" in name]
    if not bowls or not ramekins or not plates:
        return {"status": "failed", "reason": "Required objects unavailable"}

    region = state["sites"].get("main_table_next_to_ramekin_region")
    target = bowls[0]
    best = float("inf")
    for name in bowls:
        pos = objects[name]["pos"]
        score = min(distance(pos, objects[r]["pos"]) for r in ramekins)
        if region is not None:
            score += 2.0 * distance(pos, region)
        if score < best:
            best = score
            target = name

    plate = plates[0]
    region = state["sites"].get("main_table_plate_region")
    if region is not None:
        best = float("inf")
        for name in plates:
            score = distance(objects[name]["pos"], region)
            if score < best:
                best = score
                plate = name

    initial_z = objects[target]["pos"][2]
    yaw = state["eef_yaw"]
    grasped = False

    # A move's reached flag can include orientation error. Judge the actual
    # position before deciding whether an approach or descent is usable.
    for attempt in range(3):
        if remaining() < 160:
            break

        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        plate_pos = state["objects"][plate]["pos"]
        clearance = max(bowl[2], plate_pos[2]) + 0.16
        eef = state["eef_pos"]
        move([eef[0], eef[1], max(eef[2], clearance)],
             yaw, "open", 22)
        grip("open", 5)

        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        nearest = ramekins[0]
        for name in ramekins:
            if distance(bowl, state["objects"][name]["pos"]) < distance(
                    bowl, state["objects"][nearest]["pos"]):
                nearest = name
        neighbor = state["objects"][nearest]["pos"]

        # Keep the initial wrist orientation to avoid wasting the approach
        # budget on a large rotation. Pinch the wall along the finger axis.
        ux = -sin(yaw)
        uy = cos(yaw)
        side = 1.0
        if ux * (bowl[0] - neighbor[0]) + uy * (bowl[1] - neighbor[1]) < 0:
            side = -1.0
        if attempt == 2:
            side = -side
        radius = 0.038 if attempt == 0 else 0.028
        gx = bowl[0] + side * radius * ux
        gy = bowl[1] + side * radius * uy

        move([gx, gy, clearance], yaw, "open", 38)
        state = sdk.get_state()
        if distance(state["eef_pos"], [gx, gy]) > 0.025:
            move([gx, gy, clearance], yaw, "open", 18)
            state = sdk.get_state()
        if distance(state["eef_pos"], [gx, gy]) > 0.035:
            continue

        bowl = list(state["objects"][target]["pos"])
        gx = bowl[0] + side * radius * ux
        gy = bowl[1] + side * radius * uy
        grasp_z = bowl[2] + (0.035 if attempt == 0 else 0.015)
        move([gx, gy, grasp_z], yaw, "open", 35, 0.008)

        state = sdk.get_state()
        eef = state["eef_pos"]
        bowl = state["objects"][target]["pos"]
        if distance(eef, [gx, gy]) > 0.03 or eef[2] > bowl[2] + 0.075:
            continue

        before = list(bowl)
        grip("close", 14)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        move([eef[0], eef[1], eef[2] + 0.105],
             yaw, "close", 28, 0.01)

        state = sdk.get_state()
        after = list(state["objects"][target]["pos"])
        if after[2] - before[2] > 0.04:
            # Verify continued attachment with a second vertical motion.
            eef = list(state["eef_pos"])
            move([eef[0], eef[1], eef[2] + 0.025],
                 yaw, "close", 12, 0.008)
            check = sdk.get_state()
            lifted = check["objects"][target]["pos"]
            if lifted[2] > before[2] + 0.045 and distance(
                    lifted, check["eef_pos"]) < 0.11:
                grasped = True
                break

        state = sdk.get_state()
        after = state["objects"][target]["pos"]
        if after[2] > before[2] + 0.012:
            eef = state["eef_pos"]
            move([eef[0], eef[1],
                  eef[2] - (after[2] - before[2])],
                 yaw, "close", 18)
        grip("open", 7)

    if not grasped:
        return {"status": "failed", "target": target,
                "reason": "No verified bowl lift"}

    state = sdk.get_state()
    eef = list(state["eef_pos"])
    plate_pos = state["objects"][plate]["pos"]
    travel_z = max(eef[2], initial_z + 0.18, plate_pos[2] + 0.18)
    move([eef[0], eef[1], travel_z], yaw, "close", 20)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    plate_pos = state["objects"][plate]["pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    move([plate_pos[0] - offset[0], plate_pos[1] - offset[1], travel_z],
         yaw, "close", min(45, max(1, remaining() - 55)), 0.008)

    for correction in range(2):
        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        eef = state["eef_pos"]
        plate_pos = state["objects"][plate]["pos"]
        if bowl[2] < initial_z + 0.025:
            return {"status": "failed", "reason": "Bowl lost during transport"}
        if distance(bowl, plate_pos) <= 0.015:
            break
        if remaining() < 55:
            break
        move([eef[0] + plate_pos[0] - bowl[0],
              eef[1] + plate_pos[1] - bowl[1], eef[2]],
             yaw, "close", min(18, remaining() - 40), 0.006)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    plate_pos = state["objects"][plate]["pos"]
    if distance(bowl, plate_pos) > 0.035 or remaining() < 25:
        return {"status": "failed", "reason": "Placement approach incomplete"}

    release = [
        eef[0] + plate_pos[0] - bowl[0],
        eef[1] + plate_pos[1] - bowl[1],
        eef[2] + plate_pos[2] + 0.035 - bowl[2],
    ]
    move(release, yaw, "close", min(30, remaining() - 22), 0.008)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    if distance(bowl, plate_pos) > 0.04 or bowl[2] > plate_pos[2] + 0.085:
        return {"status": "failed", "reason": "Release pose unverified"}

    grip("open", 9)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.12],
         yaw, "open", min(20, max(0, remaining() - 6)))
    grip("open", 6)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    placed = (
        distance(bowl, plate_pos) < 0.045
        and plate_pos[2] - 0.015 <= bowl[2] <= plate_pos[2] + 0.07
    )
    return {"status": "done" if placed else "failed",
            "target": target,
            "reason": "Placement verified" if placed else "Placement unverified"}
