def policy_move(sdk, position, yaw, grip, steps=40, tol=0.008):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        position, yaw=yaw, gripper=grip,
        max_steps=min(steps, remaining), tol=tol
    )
    return result["reached"] or result["pos_err_m"] <= tol


def policy_gripper(sdk, action, steps=12):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining < steps:
        return False
    sdk.set_gripper(action, steps=steps)
    return True


def run_policy(sdk):
    state = sdk.get_state()
    objects = state["objects"]
    sites = state["sites"]
    bowls = sorted([name for name in objects if "black_bowl" in name])
    ramekins = sorted([name for name in objects if "ramekin" in name])
    plates = sorted([name for name in objects if "plate" in name])
    if not bowls or not ramekins or not plates:
        return {"status": "failed", "reason": "Required objects unavailable"}

    region_name = "main_table_next_to_ramekin_region"
    region = sites.get(region_name)
    if region is None:
        matches = sorted([
            name for name in sites
            if "next_to_ramekin" in name and "region" in name
        ])
        if matches:
            region_name = matches[0]
            region = sites[region_name]

    ranked = []
    for name in bowls:
        position = objects[name]["pos"]
        neighbor_distance = min([
            hypot(position[0] - objects[r]["pos"][0],
                  position[1] - objects[r]["pos"][1])
            for r in ramekins
        ])
        score = neighbor_distance
        if region is not None:
            score = hypot(position[0] - region[0], position[1] - region[1])
        ranked.append((score, name))
    ranked = sorted(ranked)
    target = ranked[0][1]

    plate_region = sites.get("main_table_plate_region")
    plate = plates[0]
    if plate_region is not None:
        ranked_plates = sorted([
            (hypot(objects[name]["pos"][0] - plate_region[0],
                   objects[name]["pos"][1] - plate_region[1]), name)
            for name in plates
        ])
        plate = ranked_plates[0][1]

    grounding = {
        "object": target,
        "plate": plate,
        "source_region": region_name if region is not None else None,
        "initial_bowl_position": list(objects[target]["pos"]),
        "initial_plate_position": list(objects[plate]["pos"]),
    }
    initial_bowl_z = objects[target]["pos"][2]
    initial_plate_z = objects[plate]["pos"][2]
    initial_yaw = state["eef_yaw"]
    held = False
    grasp_yaw = initial_yaw

    for attempt in range(3):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 200:
            break
        bowl = list(state["objects"][target]["pos"])
        baseline_z = bowl[2]
        grasp_yaw = initial_yaw
        grasp_height = 0.025
        if attempt == 1:
            grasp_height = 0.010
            grasp_yaw = initial_yaw + pi / 2.0
        if attempt == 2:
            grasp_height = 0.040

        eef = state["eef_pos"]
        clearance = max(eef[2], bowl[2] + 0.16)
        if not policy_move(
            sdk, [eef[0], eef[1], clearance],
            grasp_yaw, "open", 30, 0.012
        ):
            continue
        policy_gripper(sdk, "open", 8)
        if not policy_move(
            sdk, [bowl[0], bowl[1], clearance],
            grasp_yaw, "open", 55, 0.010
        ):
            continue

        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        if not policy_move(
            sdk, [bowl[0], bowl[1], bowl[2] + 0.075],
            grasp_yaw, "open", 35, 0.008
        ):
            continue
        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        grasp = [bowl[0], bowl[1], bowl[2] + grasp_height]
        policy_move(sdk, grasp, grasp_yaw, "open", 35, 0.005)

        state = sdk.get_state()
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - grasp[0], eef[1] - grasp[1]) > 0.015
            or abs(eef[2] - grasp[2]) > 0.018
        ):
            continue

        policy_gripper(sdk, "close", 16)
        state = sdk.get_state()
        eef = state["eef_pos"]
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.09],
            grasp_yaw, "close", 35, 0.008
        )
        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        eef = state["eef_pos"]
        held = (
            bowl[2] > baseline_z + 0.045
            and hypot(bowl[0] - eef[0], bowl[1] - eef[1]) < 0.075
        )
        if held:
            break
        policy_gripper(sdk, "open", 8)

    if not held:
        return {
            "status": "failed",
            "reason": "Grounded target did not lift",
            "grounding": grounding,
        }

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    eef = state["eef_pos"]
    carry_z = max(initial_bowl_z + 0.15, plate_pos[2] + 0.13)
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + carry_z - bowl[2]],
        grasp_yaw, "close", 30, 0.008
    )

    for correction in range(3):
        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        plate_pos = state["objects"][plate]["pos"]
        eef = state["eef_pos"]
        if bowl[2] < initial_bowl_z + 0.045:
            return {"status": "failed", "reason": "Target slipped", "grounding": grounding}
        if hypot(bowl[0] - plate_pos[0], bowl[1] - plate_pos[1]) < 0.010:
            break
        remaining = state["max_steps"] - state["steps_used"]
        if remaining < 90:
            break
        policy_move(
            sdk,
            [eef[0] + plate_pos[0] - bowl[0],
             eef[1] + plate_pos[1] - bowl[1],
             eef[2] + carry_z - bowl[2]],
            grasp_yaw, "close", min(50, remaining - 85), 0.006
        )

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    if hypot(bowl[0] - plate_pos[0], bowl[1] - plate_pos[1]) > 0.025:
        return {"status": "failed", "reason": "Plate alignment failed", "grounding": grounding}

    for height in [0.055, 0.020]:
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining < 45:
            return {"status": "failed", "reason": "Placement budget exhausted"}
        bowl = state["objects"][target]["pos"]
        plate_pos = state["objects"][plate]["pos"]
        eef = state["eef_pos"]
        resting_z = initial_bowl_z + plate_pos[2] - initial_plate_z
        policy_move(
            sdk,
            [eef[0] + plate_pos[0] - bowl[0],
             eef[1] + plate_pos[1] - bowl[1],
             eef[2] + resting_z + height - bowl[2]],
            grasp_yaw, "close", min(30, remaining - 35), 0.006
        )

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    resting_z = initial_bowl_z + plate_pos[2] - initial_plate_z
    if (
        hypot(bowl[0] - plate_pos[0], bowl[1] - plate_pos[1]) > 0.025
        or bowl[2] > resting_z + 0.045
    ):
        return {"status": "failed", "reason": "Release pose not reached", "grounding": grounding}

    if not policy_gripper(sdk, "open", 12):
        return {"status": "failed", "reason": "Release budget exhausted"}
    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.12],
        grasp_yaw, "open", 25, 0.010
    )
    policy_gripper(sdk, "open", 6)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    expected_z = initial_bowl_z + plate_pos[2] - initial_plate_z
    placed = (
        hypot(bowl[0] - plate_pos[0], bowl[1] - plate_pos[1]) < 0.040
        and expected_z - 0.015 <= bowl[2] <= expected_z + 0.050
        and state["gripper_width"] > 0.04
        and state["eef_pos"][2] - bowl[2] > 0.085
    )
    return {
        "status": "done" if placed else "failed",
        "grounding": grounding,
        "final_bowl_position": list(bowl),
        "final_plate_position": list(plate_pos),
        "reason": "Placement verified geometrically" if placed else "Placement not verified",
    }
