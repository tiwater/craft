def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def position(state, name):
        return list(state["objects"][name]["pos"])

    def distance_xy(a, b):
        return hypot(a[0] - b[0], a[1] - b[1])

    def move(target, yaw, grip, budget=35, tol=0.008):
        available = remaining()
        if available <= 0:
            return False
        result = sdk.move_to(
            target, yaw=yaw, gripper=grip,
            max_steps=min(budget, available), tol=tol
        )
        return result["reached"] or result["pos_err_m"] <= tol

    def actuate(action, steps=12):
        if remaining() > 0:
            return sdk.set_gripper(action, steps=min(steps, remaining()))
        return {"gripper_width": 0.0, "grasped_hint": False}

    state = sdk.get_state()
    candidates = []
    plates = []
    for name in state["objects"]:
        if "plate" in name:
            plates.append(name)
        if "black_bowl" in name:
            bowl = position(state, name)
            for support in state["objects"]:
                if "cookie" in support:
                    box = position(state, support)
                    separation = distance_xy(bowl, box)
                    height = bowl[2] - box[2]
                    if separation < 0.075 and -0.015 <= height <= 0.12:
                        candidates.append(
                            (separation + 0.2 * abs(height), name, support)
                        )

    if not candidates or not plates:
        return {"status": "failed", "reason": "Required grounded objects unavailable"}

    candidates = sorted(candidates)
    bowl_name = candidates[0][1]
    box_name = candidates[0][2]
    if len(candidates) > 1 and candidates[1][0] - candidates[0][0] < 0.005:
        return {"status": "failed", "reason": "Ambiguous bowl on cookie box"}

    targets = []
    for name in plates:
        score = 0.0
        if "main_table_plate_region" in state["sites"]:
            score = distance_xy(
                position(state, name), state["sites"]["main_table_plate_region"]
            )
        targets.append((score, name))
    plate_name = sorted(targets)[0][1]

    grounding = {
        "object": bowl_name,
        "support": box_name,
        "target": plate_name,
        "coordinate_source": "objects.pos",
        "initial_object_position": position(state, bowl_name),
        "initial_support_position": position(state, box_name),
        "initial_target_position": position(state, plate_name),
    }
    for name in [bowl_name, box_name, plate_name]:
        site_name = name + "_default_site"
        if site_name in state["sites"]:
            grounding[site_name] = list(state["sites"][site_name])

    def failure(reason):
        return {"status": "failed", "reason": reason, "grounding": grounding}

    initial_yaw = state["eef_yaw"]
    held = False
    grasp_yaw = initial_yaw
    grasp_heights = [0.018, 0.005]
    attempts = []

    for attempt in range(2):
        if remaining() < 165:
            break
        state = sdk.get_state()
        bowl = position(state, bowl_name)
        plate = position(state, plate_name)
        eef = state["eef_pos"]
        clearance = max(bowl[2] + 0.15, plate[2] + 0.15, eef[2])
        grasp_yaw = initial_yaw + attempt * pi / 2.0

        move([eef[0], eef[1], clearance], initial_yaw, "open", 25, 0.012)
        actuate("open", 8)
        state = sdk.get_state()
        bowl = position(state, bowl_name)
        if not move(
            [bowl[0], bowl[1], clearance],
            grasp_yaw, "open", 38, 0.008
        ):
            continue

        state = sdk.get_state()
        bowl = position(state, bowl_name)
        move(
            [bowl[0], bowl[1], bowl[2] + 0.060],
            grasp_yaw, "open", 25, 0.008
        )
        state = sdk.get_state()
        bowl = position(state, bowl_name)
        target = [bowl[0], bowl[1], bowl[2] + grasp_heights[attempt]]
        move(target, grasp_yaw, "open", 32, 0.005)

        state = sdk.get_state()
        bowl = position(state, bowl_name)
        eef = state["eef_pos"]
        record = {
            "target_eef_position": target,
            "actual_eef_position": list(eef),
            "object_position": list(bowl),
        }
        attempts.append(record)
        grounding["grasp_attempts"] = attempts
        if distance_xy(eef, bowl) > 0.018 or abs(eef[2] - target[2]) > 0.018:
            continue

        before_lift = list(bowl)
        grasp = actuate("close", 18)
        record["closed_width"] = grasp["gripper_width"]
        state = sdk.get_state()
        eef = state["eef_pos"]
        move(
            [eef[0], eef[1], eef[2] + 0.065],
            grasp_yaw, "close", 25, 0.008
        )
        state = sdk.get_state()
        bowl = position(state, bowl_name)
        record["observed_lift"] = bowl[2] - before_lift[2]
        if (
            bowl[2] - before_lift[2] > 0.035
            and distance_xy(bowl, state["eef_pos"]) < 0.055
        ):
            held = True
            break
        actuate("open", 8)

    if not held:
        return failure("Selected bowl did not follow grasp")
    if remaining() < 90:
        return failure("Insufficient placement budget")

    state = sdk.get_state()
    bowl = position(state, bowl_name)
    plate = position(state, plate_name)
    eef = state["eef_pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    carried_offset = list(offset)
    carry_z = max(eef[2], plate[2] + 0.13 - offset[2], bowl[2] + 0.06 - offset[2])
    move([eef[0], eef[1], carry_z], grasp_yaw, "close", 22, 0.010)

    state = sdk.get_state()
    plate = position(state, plate_name)
    if not move(
        [plate[0] - offset[0], plate[1] - offset[1], carry_z],
        grasp_yaw, "close", min(42, max(1, remaining() - 52)), 0.008
    ):
        return failure("Could not reach plate approach")

    for height in [0.055, 0.020]:
        if remaining() < 30:
            return failure("Insufficient release budget")
        state = sdk.get_state()
        bowl = position(state, bowl_name)
        plate = position(state, plate_name)
        eef = state["eef_pos"]
        offset = [bowl[i] - eef[i] for i in range(3)]
        if (
            abs(offset[2] - carried_offset[2]) > 0.035
            or distance_xy(bowl, eef) > 0.065
        ):
            return failure("Grasp lost during transport")
        target = [
            eef[0] + plate[0] - bowl[0],
            eef[1] + plate[1] - bowl[1],
            eef[2] + plate[2] + height - bowl[2],
        ]
        grounding["release_eef_position"] = target
        move(
            target, grasp_yaw, "close",
            min(28, max(1, remaining() - 26)), 0.006
        )

    state = sdk.get_state()
    bowl = position(state, bowl_name)
    plate = position(state, plate_name)
    if (
        distance_xy(bowl, plate) > 0.028
        or bowl[2] > plate[2] + 0.055
        or bowl[2] < plate[2] - 0.015
    ):
        return failure("Release pose outside plate region")

    actuate("open", 10)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.12],
        grasp_yaw, "open", min(25, max(1, remaining() - 6)), 0.012
    )
    actuate("open", 6)

    state = sdk.get_state()
    bowl = position(state, bowl_name)
    plate = position(state, plate_name)
    placed = (
        distance_xy(bowl, plate) < 0.040
        and plate[2] - 0.015 <= bowl[2] <= plate[2] + 0.055
        and state["eef_pos"][2] - bowl[2] > 0.060
    )
    return {
        "status": "done" if placed else "failed",
        "object": bowl_name,
        "target": plate_name,
        "grounding": grounding,
        "final_object_position": bowl,
        "final_target_position": plate,
    }
