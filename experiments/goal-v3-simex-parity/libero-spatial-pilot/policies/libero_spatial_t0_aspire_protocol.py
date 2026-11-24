def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, state["max_steps"] - state["steps_used"])

    def move(pos, yaw, grip, steps=35, tol=0.012):
        budget = min(steps, remaining())
        if budget > 0:
            return sdk.move_to(
                pos, yaw=yaw, gripper=grip, max_steps=budget, tol=tol
            )
        return None

    def grip(action, steps=12):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    state = sdk.get_state()
    objects = state["objects"]
    bowls = [name for name in objects if "black_bowl" in name]
    plates = [name for name in objects if "plate" in name]
    ramekins = [name for name in objects if "ramekin" in name]
    if not bowls or not plates or not ramekins:
        return {"status": "failed", "reason": "Required objects unavailable"}

    plate_name = plates[0]
    plate = objects[plate_name]["pos"]
    ramekin = objects[ramekins[0]]["pos"]
    vx = ramekin[0] - plate[0]
    vy = ramekin[1] - plate[1]
    length2 = max(vx * vx + vy * vy, 0.000001)
    region = None
    for name in state["sites"]:
        if "between_plate_ramekin" in name:
            region = state["sites"][name]
            break

    target = bowls[0]
    best = 1000000.0
    for name in bowls:
        pos = objects[name]["pos"]
        t = ((pos[0] - plate[0]) * vx + (pos[1] - plate[1]) * vy) / length2
        projection = max(0.0, min(1.0, t))
        dx = pos[0] - plate[0] - projection * vx
        dy = pos[1] - plate[1] - projection * vy
        score = dx * dx + dy * dy + 0.15 * length2 * (t - 0.5) ** 2
        if t < 0.0 or t > 1.0:
            score += length2
        if region is not None:
            score += 0.25 * (
                (pos[0] - region[0]) ** 2 + (pos[1] - region[1]) ** 2
            )
        if score < best:
            best = score
            target = name

    base_yaw = state["eef_yaw"]
    initial_z = objects[target]["pos"][2]
    acquired = False
    yaw = base_yaw

    # Pinch a rim rather than attempting to span the entire bowl.
    # Contact can prevent exact pose convergence; verify pickup by object motion.
    for attempt in range(3):
        if remaining() < 155:
            break
        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        plate = state["objects"][plate_name]["pos"]
        eef = state["eef_pos"]
        yaw = base_yaw + (pi / 2.0 if attempt == 2 else 0.0)
        side = -1.0 if attempt == 1 else 1.0
        radius = 0.043 if attempt < 2 else 0.035
        ox = -sin(yaw) * side * radius
        oy = cos(yaw) * side * radius
        clearance = max(initial_z, bowl[2], plate[2]) + 0.16

        move([eef[0], eef[1], max(eef[2], clearance)], yaw, "open", 25)
        grip("open", 6)
        bowl = sdk.get_state()["objects"][target]["pos"]
        move([bowl[0] + ox, bowl[1] + oy, clearance], yaw, "open", 32)

        bowl = sdk.get_state()["objects"][target]["pos"]
        grasp_z = bowl[2] + (0.035 if attempt == 0 else 0.018)
        move([bowl[0] + ox, bowl[1] + oy, grasp_z], yaw, "open", 32, 0.01)

        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - bowl[0] - ox, eef[1] - bowl[1] - oy) > 0.035
            or eef[2] > bowl[2] + 0.085
        ):
            continue

        before = list(bowl)
        grip("close", 14)
        eef = sdk.get_state()["eef_pos"]
        move([eef[0], eef[1], eef[2] + 0.105], yaw, "close", 30)
        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        rise = bowl[2] - before[2]

        if rise > 0.04:
            held_before = list(bowl)
            eef = state["eef_pos"]
            move([eef[0], eef[1], eef[2] + 0.025], yaw, "close", 15)
            state = sdk.get_state()
            bowl = state["objects"][target]["pos"]
            if bowl[2] >= held_before[2] - 0.012:
                acquired = True
                break
        grip("open", 6)

    if not acquired:
        return {"status": "failed", "target": target, "reason": "Pickup not verified"}

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    transport_z = max(bowl[2], plate[2] + 0.15)
    move(
        [eef[0], eef[1], transport_z - offset[2]],
        yaw, "close", 20
    )
    move(
        [
            plate[0] - offset[0],
            plate[1] - offset[1],
            transport_z - offset[2],
        ],
        yaw, "close", 45
    )

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if bowl[2] < initial_z + 0.035:
        return {"status": "failed", "target": target, "reason": "Bowl slipped"}

    # Recompute the held-object offset after transport.
    eef = state["eef_pos"]
    move(
        [
            eef[0] + plate[0] - bowl[0],
            eef[1] + plate[1] - bowl[1],
            eef[2] + plate[2] + 0.065 - bowl[2],
        ],
        yaw, "close", 30
    )
    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    if hypot(bowl[0] - plate[0], bowl[1] - plate[1]) > 0.035:
        return {"status": "failed", "target": target, "reason": "Plate alignment failed"}

    move(
        [
            eef[0] + plate[0] - bowl[0],
            eef[1] + plate[1] - bowl[1],
            eef[2] + plate[2] + 0.03 - bowl[2],
        ],
        yaw, "close", 22
    )
    if remaining() < 12:
        return {"status": "failed", "target": target, "reason": "Insufficient release budget"}

    grip("open", 12)
    eef = sdk.get_state()["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.13], yaw, "open", 25)
    grip("open", 8)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate = state["objects"][plate_name]["pos"]
    placed = (
        hypot(bowl[0] - plate[0], bowl[1] - plate[1]) < 0.04
        and plate[2] - 0.025 <= bowl[2] <= plate[2] + 0.07
        and state["gripper_width"] > 0.04
    )
    return {
        "status": "done" if placed else "failed",
        "target": target,
        "reason": "Placement verified" if placed else "Final placement outside tolerance",
    }
