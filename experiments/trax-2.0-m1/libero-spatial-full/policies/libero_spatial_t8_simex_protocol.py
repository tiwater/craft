def policy_distance(a, b):
    return hypot(a[0] - b[0], a[1] - b[1])


def policy_remaining(sdk):
    state = sdk.get_state()
    return max(0, state["max_steps"] - state["steps_used"])


def policy_move(sdk, position, yaw, grip, limit=30, tol=0.008):
    remaining = policy_remaining(sdk)
    if remaining <= 0:
        return False
    result = sdk.move_to(
        position, yaw=yaw, gripper=grip,
        max_steps=min(limit, remaining), tol=tol
    )
    return result["reached"] or result["pos_err_m"] <= tol


def policy_grip(sdk, action, steps=12):
    remaining = policy_remaining(sdk)
    if remaining <= 0:
        return False
    sdk.set_gripper(action, steps=min(steps, remaining))
    return True


def run_policy(sdk):
    state = sdk.get_state()
    objects = state["objects"]
    sites = state["sites"]
    plates = sorted([name for name in objects if "plate" in name])
    bowls = sorted([
        name for name in objects if "black" in name and "bowl" in name
    ])
    if not plates or not bowls:
        return {"status": "failed", "reason": "Required objects unavailable"}

    plate_name = plates[0]
    plate_region = sites.get("main_table_plate_region")
    if plate_region is not None:
        for name in plates:
            if policy_distance(objects[name]["pos"], plate_region) < policy_distance(
                objects[plate_name]["pos"], plate_region
            ):
                plate_name = name

    reference = sites.get(
        "main_table_next_to_plate_region", objects[plate_name]["pos"]
    )
    bowl_name = bowls[0]
    distances = {}
    for name in bowls:
        distances[name] = policy_distance(objects[name]["pos"], reference)
        if distances[name] < distances[bowl_name]:
            bowl_name = name

    grounding = {
        "object": bowl_name,
        "target": plate_name,
        "selection_reference": list(reference),
        "candidate_distances": distances,
        "initial_object_pos": list(objects[bowl_name]["pos"]),
        "initial_target_pos": list(objects[plate_name]["pos"]),
        "object_site": sites.get(bowl_name + "_default_site"),
        "target_site": sites.get(plate_name + "_default_site"),
    }
    initial_height = objects[bowl_name]["pos"][2]
    plate_initial_height = objects[plate_name]["pos"][2]
    resting_offset = initial_height - plate_initial_height
    initial_yaw = state["eef_yaw"]
    held = False
    attempts = []

    for attempt in range(3):
        if policy_remaining(sdk) < 145:
            break

        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        plate = list(state["objects"][plate_name]["pos"])
        eef = list(state["eef_pos"])
        yaw = initial_yaw
        dx = 0.0
        dy = 0.0
        grasp_height = 0.045
        if attempt == 1:
            dx = 0.043 * cos(yaw)
            dy = 0.043 * sin(yaw)
            grasp_height = 0.032
        if attempt == 2:
            yaw = atan2(sin(initial_yaw + pi / 2), cos(initial_yaw + pi / 2))
            dx = 0.043 * cos(yaw)
            dy = 0.043 * sin(yaw)
            grasp_height = 0.025

        clearance = max(bowl[2], plate[2]) + 0.16
        policy_move(
            sdk, [eef[0], eef[1], max(eef[2], clearance)],
            yaw, "open", 24
        )
        policy_grip(sdk, "open", 6)
        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        if not policy_move(
            sdk, [bowl[0] + dx, bowl[1] + dy, clearance],
            yaw, "open", 35
        ):
            continue

        policy_move(
            sdk,
            [bowl[0] + dx, bowl[1] + dy, bowl[2] + grasp_height],
            yaw, "open", 34, 0.005
        )
        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        eef = list(state["eef_pos"])
        requested = [bowl[0] + dx, bowl[1] + dy, bowl[2] + grasp_height]
        if policy_distance(eef, requested) > 0.016:
            continue
        if abs(eef[2] - requested[2]) > 0.018:
            continue

        policy_grip(sdk, "close", 16)
        state = sdk.get_state()
        lift_start = list(state["objects"][bowl_name]["pos"])
        lift_eef = list(state["eef_pos"])
        policy_move(
            sdk, [lift_eef[0], lift_eef[1], lift_eef[2] + 0.055],
            yaw, "close", 22, 0.006
        )
        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        attempts.append({
            "attempt": attempt,
            "lift": bowl[2] - lift_start[2],
            "gripper_width": state["gripper_width"],
        })
        if bowl[2] - lift_start[2] < 0.025:
            continue

        eef = list(state["eef_pos"])
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.065],
            yaw, "close", 20, 0.007
        )
        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        if bowl[2] - lift_start[2] > 0.075:
            held = True
            break

    if not held:
        return {
            "status": "failed",
            "reason": "Grounded bowl did not remain lifted",
            "grounding": grounding,
            "attempts": attempts,
        }

    for segment in range(6):
        if policy_remaining(sdk) < 65:
            break
        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        plate = list(state["objects"][plate_name]["pos"])
        eef = list(state["eef_pos"])
        if bowl[2] < lift_start[2] + 0.035:
            return {
                "status": "failed",
                "reason": "Selected bowl lost during transport",
                "grounding": grounding,
            }
        distance = policy_distance(bowl, plate)
        if distance < 0.006:
            break
        fraction = min(1.0, 0.06 / max(distance, 0.001))
        carry_height = max(initial_height + 0.12, plate[2] + 0.12)
        policy_move(
            sdk,
            [
                eef[0] + fraction * (plate[0] - bowl[0]),
                eef[1] + fraction * (plate[1] - bowl[1]),
                eef[2] + carry_height - bowl[2],
            ],
            yaw, "close", 23, 0.006
        )

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if policy_distance(bowl, plate) > 0.018:
        return {
            "status": "failed",
            "reason": "Selected bowl not aligned with live plate pose",
            "grounding": grounding,
        }

    for gap in [0.055, 0.025, 0.012]:
        if policy_remaining(sdk) < 30:
            break
        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        plate = list(state["objects"][plate_name]["pos"])
        eef = list(state["eef_pos"])
        target_height = plate[2] + resting_offset + gap
        policy_move(
            sdk,
            [
                eef[0] + plate[0] - bowl[0],
                eef[1] + plate[1] - bowl[1],
                eef[2] + target_height - bowl[2],
            ],
            yaw, "close", 20, 0.005
        )

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if (
        policy_distance(bowl, plate) > 0.02
        or bowl[2] - plate[2] > 0.07
    ):
        return {
            "status": "failed",
            "reason": "Release pose unverified",
            "grounding": grounding,
        }

    if not policy_grip(sdk, "open", 10):
        return {"status": "failed", "reason": "Release budget exhausted"}

    state = sdk.get_state()
    eef = list(state["eef_pos"])
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.12],
        yaw, "open", 18, 0.01
    )
    policy_grip(sdk, "open", 8)
    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    verified = (
        policy_distance(bowl, plate) < 0.03
        and -0.015 <= bowl[2] - plate[2] <= 0.06
        and state["gripper_width"] > 0.045
    )
    return {
        "status": "done" if verified else "failed",
        "grounding": grounding,
        "attempts": attempts,
        "final_object_pos": list(bowl),
        "final_target_pos": list(plate),
    }
