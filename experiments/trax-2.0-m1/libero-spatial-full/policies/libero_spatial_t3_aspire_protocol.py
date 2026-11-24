def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def distance_xy(a, b):
        return hypot(a[0] - b[0], a[1] - b[1])

    def move(pos, yaw, grip, steps=35, tol=0.01):
        budget = min(steps, remaining())
        if budget <= 0:
            return False
        result = sdk.move_to(
            pos, yaw=yaw, gripper=grip, max_steps=budget, tol=tol
        )
        return result["reached"]

    def grip(action, steps=10):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    state = sdk.get_state()
    objects = state["objects"]
    bowls = [name for name in objects if "black_bowl" in name]
    boxes = [name for name in objects if "cookie" in name]
    plates = [name for name in objects if "plate" in name]
    if not bowls or not boxes or not plates:
        return {"status": "failed", "reason": "Required objects unavailable"}

    bowl_name = bowls[0]
    best_score = 1000000.0
    for name in bowls:
        bowl = objects[name]["pos"]
        for box_name in boxes:
            box = objects[box_name]["pos"]
            score = distance_xy(bowl, box)
            if bowl[2] < box[2] - 0.015:
                score += 1.0
            if score < best_score:
                best_score = score
                bowl_name = name

    plate_name = plates[0]
    initial_bowl = list(objects[bowl_name]["pos"])
    base_yaw = state["eef_yaw"]
    held = False
    grasp_yaw = base_yaw

    # A strict reached flag can fail at contact. Judge the actual pose,
    # and use progressively deeper grasps followed by a rim pinch.
    for attempt in range(4):
        if remaining() < 135:
            break

        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        eef = state["eef_pos"]
        grasp_yaw = base_yaw
        if attempt == 1:
            grasp_yaw += pi / 2.0

        clearance = max(bowl[2] + 0.14, eef[2])
        move(
            [eef[0], eef[1], clearance],
            state["eef_yaw"],
            "open",
            22,
        )
        grip("open", 6)

        shift_x = 0.0
        shift_y = 0.0
        depth = 0.005
        if attempt == 1:
            depth = -0.012
        if attempt >= 2:
            # Panda fingers close along the local y axis. Offset across
            # that axis to put one finger inside and one outside the rim.
            side = 1.0 if attempt == 2 else -1.0
            shift_x = -sin(grasp_yaw) * 0.043 * side
            shift_y = cos(grasp_yaw) * 0.043 * side
            depth = 0.005

        move(
            [bowl[0] + shift_x, bowl[1] + shift_y, clearance],
            grasp_yaw,
            "open",
            30,
        )
        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        target = [bowl[0] + shift_x, bowl[1] + shift_y, bowl[2] + depth]
        if distance_xy(state["eef_pos"], target) > 0.025:
            continue

        move(target, grasp_yaw, "open", 30, 0.006)
        state = sdk.get_state()
        eef = state["eef_pos"]
        if distance_xy(eef, target) > 0.025 or abs(eef[2] - target[2]) > 0.04:
            continue

        grip("close", 14)
        state = sdk.get_state()
        before = list(state["objects"][bowl_name]["pos"])
        eef_before = list(state["eef_pos"])
        move(
            [eef_before[0], eef_before[1], eef_before[2] + 0.10],
            grasp_yaw,
            "close",
            28,
            0.008,
        )
        state = sdk.get_state()
        after = state["objects"][bowl_name]["pos"]
        eef_after = state["eef_pos"]
        bowl_rise = after[2] - before[2]
        hand_rise = eef_after[2] - eef_before[2]
        if (
            bowl_rise > 0.045
            and hand_rise > 0.045
            and abs(bowl_rise - hand_rise) < 0.035
            and distance_xy(after, eef_after) < 0.105
        ):
            held = True
            break
        grip("open", 6)

    if not held:
        return {"status": "failed", "reason": "No verified bowl grasp"}

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    safe_bowl_z = max(initial_bowl[2] + 0.13, plate[2] + 0.17)
    carry_z = max(eef[2], safe_bowl_z - offset[2])
    move([eef[0], eef[1], carry_z], grasp_yaw, "close", 25)

    state = sdk.get_state()
    plate = state["objects"][plate_name]["pos"]
    move(
        [plate[0] - offset[0], plate[1] - offset[1], carry_z],
        grasp_yaw,
        "close",
        45,
        0.008,
    )

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    eef = state["eef_pos"]
    plate = state["objects"][plate_name]["pos"]
    current_offset = [bowl[i] - eef[i] for i in range(3)]
    offset_error = sqrt(sum(
        (current_offset[i] - offset[i]) ** 2 for i in range(3)
    ))
    if offset_error > 0.045:
        return {"status": "failed", "reason": "Bowl slipped during transfer"}

    if distance_xy(bowl, plate) > 0.012:
        move(
            [
                plate[0] - current_offset[0],
                plate[1] - current_offset[1],
                eef[2],
            ],
            grasp_yaw,
            "close",
            20,
            0.006,
        )

    if remaining() < 30:
        return {"status": "failed", "reason": "Insufficient placement budget"}

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    eef = state["eef_pos"]
    plate = state["objects"][plate_name]["pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]

    # Leave a small settling distance instead of forcing the bowl's
    # body origin down to the plate's body origin.
    move(
        [
            plate[0] - offset[0],
            plate[1] - offset[1],
            plate[2] + 0.045 - offset[2],
        ],
        grasp_yaw,
        "close",
        min(35, remaining() - 25),
        0.007,
    )
    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if distance_xy(bowl, plate) > 0.035 or bowl[2] > plate[2] + 0.085:
        return {"status": "failed", "reason": "Bowl not positioned for release"}

    grip("open", 10)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.12],
        grasp_yaw,
        "open",
        min(25, max(0, remaining() - 8)),
    )
    grip("open", 8)

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    placed = (
        distance_xy(bowl, plate) < 0.045
        and plate[2] - 0.015 <= bowl[2] <= plate[2] + 0.065
        and state["eef_pos"][2] > bowl[2] + 0.06
    )
    return {
        "status": "done" if placed else "failed",
        "object": bowl_name,
        "target": plate_name,
    }
