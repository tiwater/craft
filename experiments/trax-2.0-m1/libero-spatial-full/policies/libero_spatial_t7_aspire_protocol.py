def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return state["max_steps"] - state["steps_used"]

    def move(pos, yaw, grip, steps=35, tol=0.012):
        budget = min(steps, remaining())
        if budget <= 0:
            return False
        result = sdk.move_to(
            pos, yaw=yaw, gripper=grip, max_steps=budget, tol=tol
        )
        return result["reached"]

    def gripper(action, steps=10):
        budget = min(steps, remaining())
        if budget > 0:
            return sdk.set_gripper(action, steps=budget)
        return {"grasped_hint": False, "gripper_width": 0.0}

    state = sdk.get_state()
    bowls = [name for name in state["objects"] if "black_bowl" in name]
    plates = [name for name in state["objects"] if "plate" in name]
    if not bowls or not plates:
        return {"status": "failed", "reason": "Required objects unavailable"}

    stove_points = []
    for name in state["sites"]:
        if "stove" in name and ("burner" in name or "cook_region" in name):
            stove_points.append(state["sites"][name])
    if not stove_points:
        for name in state["fixtures"]:
            if "stove" in name:
                stove_points.append(state["fixtures"][name]["pos"])
    if not stove_points:
        return {"status": "failed", "reason": "Stove pose unavailable"}

    bowl_name = bowls[0]
    best = 1000000.0
    for name in bowls:
        pos = state["objects"][name]["pos"]
        score = min([
            sum((pos[i] - point[i]) ** 2 for i in range(3))
            for point in stove_points
        ])
        if score < best:
            best = score
            bowl_name = name

    plate_name = plates[0]
    initial_yaw = state["eef_yaw"]
    held = False
    offset = [0.0, 0.0, 0.0]

    # A centered descent can stall against a wide bowl before closing.
    # Approach the rim, tolerate contact, and verify pickup by object motion.
    for attempt in range(3):
        if remaining() < 155:
            break
        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        yaw = initial_yaw + (pi / 2.0 if attempt == 2 else 0.0)
        side = 1.0 if attempt != 1 else -1.0
        rim_offset = 0.043
        dx = -sin(yaw) * rim_offset * side
        dy = cos(yaw) * rim_offset * side
        clearance = max(state["eef_pos"][2], bowl[2] + 0.16)

        gripper("open", 8)
        eef = sdk.get_state()["eef_pos"]
        move([eef[0], eef[1], clearance], yaw, "open", 22)
        move([bowl[0] + dx, bowl[1] + dy, clearance], yaw, "open", 35)

        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        eef = state["eef_pos"]
        target_x = bowl[0] + dx
        target_y = bowl[1] + dy
        if hypot(eef[0] - target_x, eef[1] - target_y) > 0.035:
            continue

        grasp_z = bowl[2] + (0.045 if attempt < 2 else 0.030)
        move([target_x, target_y, grasp_z], yaw, "open", 28, 0.008)
        state = sdk.get_state()
        eef = state["eef_pos"]
        bowl = state["objects"][bowl_name]["pos"]
        if hypot(eef[0] - bowl[0] - dx, eef[1] - bowl[1] - dy) > 0.035:
            continue
        if eef[2] - bowl[2] > 0.10:
            continue

        original = list(bowl)
        gripper("close", 14)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        move([eef[0], eef[1], eef[2] + 0.105], yaw, "close", 28)

        state = sdk.get_state()
        bowl = state["objects"][bowl_name]["pos"]
        eef = state["eef_pos"]
        if bowl[2] - original[2] > 0.045:
            offset = [bowl[i] - eef[i] for i in range(3)]
            held = True
            break

    if not held:
        return {"status": "failed", "reason": "Bowl did not follow lift"}

    state = sdk.get_state()
    eef = state["eef_pos"]
    plate = state["objects"][plate_name]["pos"]
    bowl = state["objects"][bowl_name]["pos"]
    travel_z = max(eef[2], plate[2] + 0.17 - offset[2], bowl[2] + 0.04 - offset[2])
    move([eef[0], eef[1], travel_z], yaw, "close", 25)
    move(
        [plate[0] - offset[0], plate[1] - offset[1], travel_z],
        yaw, "close", 50
    )

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    eef = state["eef_pos"]
    plate = state["objects"][plate_name]["pos"]
    current_offset = [bowl[i] - eef[i] for i in range(3)]
    if sqrt(sum((current_offset[i] - offset[i]) ** 2 for i in range(3))) > 0.045:
        return {"status": "failed", "reason": "Bowl slipped during transport"}

    for correction in range(2):
        state = sdk.get_state()
        bowl = state["objects"][bowl_name]["pos"]
        plate = state["objects"][plate_name]["pos"]
        eef = state["eef_pos"]
        error_x = plate[0] - bowl[0]
        error_y = plate[1] - bowl[1]
        if hypot(error_x, error_y) < 0.012:
            break
        move([eef[0] + error_x, eef[1] + error_y, eef[2]], yaw, "close", 20)

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    if hypot(bowl[0] - plate[0], bowl[1] - plate[1]) > 0.035:
        return {"status": "failed", "reason": "Could not align bowl with plate"}

    # Release near the support surface even if contact prevents exact convergence.
    target_z = eef[2] + plate[2] + 0.035 - bowl[2]
    move([eef[0], eef[1], target_z], yaw, "close", 32, 0.01)

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if bowl[2] - plate[2] > 0.09:
        return {"status": "failed", "reason": "Bowl remained too high to release"}

    gripper("open", 12)
    eef = sdk.get_state()["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.12], yaw, "open", 25)
    gripper("open", 12)

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    centered = hypot(bowl[0] - plate[0], bowl[1] - plate[1]) < 0.045
    supported = -0.01 < bowl[2] - plate[2] < 0.065
    if centered and supported:
        return {"status": "done", "object": bowl_name}
    return {"status": "failed", "reason": "Final placement could not be verified"}
