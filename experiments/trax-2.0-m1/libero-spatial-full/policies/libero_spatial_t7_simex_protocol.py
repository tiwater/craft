def policy_move(sdk, target, yaw, gripper, limit=45, tol=0.012):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        target, yaw=yaw, gripper=gripper,
        max_steps=min(limit, remaining), tol=tol
    )
    return result["reached"] or result["pos_err_m"] <= tol


def policy_gripper(sdk, action, steps=12):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    sdk.set_gripper(action, steps=min(steps, remaining))
    return True


def run_policy(sdk):
    state = sdk.get_state()
    objects = state["objects"]
    sites = state["sites"]
    bowls = [name for name in objects if "black_bowl" in name]
    plates = [name for name in objects if "plate" in name]
    supports = [
        name for name in sites
        if "stove" in name and "burner" in name
    ]
    if not supports:
        supports = [
            name for name in sites
            if "stove" in name and "cook_region" in name
        ]
    if not bowls or not plates or not supports:
        return {"status": "failed", "reason": "Required scene entities unavailable"}

    ranked = []
    for name in bowls:
        bowl = objects[name]["pos"]
        for site_name in supports:
            support = sites[site_name]
            horizontal = hypot(bowl[0] - support[0], bowl[1] - support[1])
            vertical = abs(bowl[2] - support[2])
            if horizontal < 0.10 and vertical < 0.09:
                ranked.append((horizontal + vertical, name, site_name))
    if not ranked:
        return {"status": "failed", "reason": "No black bowl grounded on stove"}

    choice = sorted(ranked)[0]
    bowl_name = choice[1]
    source_site = choice[2]
    plate_name = sorted(plates)[0]
    if len(plates) > 1:
        regions = [
            name for name in sites
            if "plate_region" in name and "next_to" not in name
        ]
        if not regions:
            return {"status": "failed", "reason": "Ambiguous destination plate"}
        destinations = []
        for name in plates:
            pos = objects[name]["pos"]
            distance = min([
                hypot(pos[0] - sites[region][0], pos[1] - sites[region][1])
                for region in regions
            ])
            destinations.append((distance, name))
        plate_name = sorted(destinations)[0][1]

    grounding = {
        "object": bowl_name,
        "source_site": source_site,
        "source_site_pos": list(sites[source_site]),
        "source_pos": list(objects[bowl_name]["pos"]),
        "destination": plate_name,
        "destination_pos": list(objects[plate_name]["pos"])
    }
    yaw = state["eef_yaw"]
    held = False
    offset = [0.0, 0.0, 0.0]

    for attempt in range(2):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 190:
            break
        bowl = list(state["objects"][bowl_name]["pos"])
        eef = list(state["eef_pos"])
        clearance = max(eef[2], bowl[2] + 0.16)
        policy_gripper(sdk, "open", 10)
        policy_move(
            sdk, [eef[0], eef[1], clearance],
            yaw, "open", 40, 0.020
        )
        state = sdk.get_state()
        if state["eef_pos"][2] < bowl[2] + 0.09:
            return {
                "status": "failed",
                "reason": "Unable to clear source bowl",
                "grounding": grounding,
                "eef_pos": list(state["eef_pos"])
            }

        policy_move(
            sdk, [bowl[0], bowl[1], clearance],
            yaw, "open", 60, 0.015
        )
        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        eef = state["eef_pos"]
        if hypot(eef[0] - bowl[0], eef[1] - bowl[1]) > 0.025:
            continue

        grasp_height = 0.025 if attempt == 0 else 0.010
        policy_move(
            sdk, [bowl[0], bowl[1], bowl[2] + 0.065],
            yaw, "open", 35, 0.012
        )
        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        target = [bowl[0], bowl[1], bowl[2] + grasp_height]
        policy_move(sdk, target, yaw, "open", 45, 0.008)
        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        eef = list(state["eef_pos"])
        if (
            hypot(eef[0] - bowl[0], eef[1] - bowl[1]) > 0.018
            or abs(eef[2] - target[2]) > 0.025
        ):
            continue

        original_z = bowl[2]
        policy_gripper(sdk, "close", 18)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.065],
            yaw, "close", 35, 0.012
        )
        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        eef = list(state["eef_pos"])
        if bowl[2] - original_z < 0.030:
            continue
        first_offset = [bowl[i] - eef[i] for i in range(3)]
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.065],
            yaw, "close", 30, 0.012
        )
        state = sdk.get_state()
        bowl = state["objects"][bowl_name]["pos"]
        eef = state["eef_pos"]
        offset = [bowl[i] - eef[i] for i in range(3)]
        change = sqrt(sum([
            (offset[i] - first_offset[i]) ** 2 for i in range(3)
        ]))
        if bowl[2] - original_z > 0.075 and change < 0.025:
            held = True
            break

    if not held:
        return {
            "status": "failed",
            "reason": "Bowl pickup not verified",
            "grounding": grounding,
            "eef_pos": list(sdk.get_state()["eef_pos"])
        }

    state = sdk.get_state()
    plate = state["objects"][plate_name]["pos"]
    eef = list(state["eef_pos"])
    travel_height = max(eef[2], plate[2] + 0.17 - offset[2])
    policy_move(
        sdk, [eef[0], eef[1], travel_height],
        yaw, "close", 25, 0.015
    )
    if not policy_move(
        sdk,
        [plate[0] - offset[0], plate[1] - offset[1], travel_height],
        yaw, "close", 60, 0.012
    ):
        return {
            "status": "failed",
            "reason": "Plate approach not reached",
            "grounding": grounding
        }

    for height in [0.060, 0.025]:
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 50:
            return {"status": "failed", "reason": "Insufficient placement budget"}
        bowl = state["objects"][bowl_name]["pos"]
        plate = state["objects"][plate_name]["pos"]
        eef = state["eef_pos"]
        current_offset = [bowl[i] - eef[i] for i in range(3)]
        if sqrt(sum([
            (current_offset[i] - offset[i]) ** 2 for i in range(3)
        ])) > 0.040:
            return {"status": "failed", "reason": "Bowl slipped during transport"}
        target = [
            eef[0] + plate[0] - bowl[0],
            eef[1] + plate[1] - bowl[1],
            eef[2] + plate[2] + height - bowl[2]
        ]
        policy_move(sdk, target, yaw, "close", 30, 0.010)

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if (
        hypot(bowl[0] - plate[0], bowl[1] - plate[1]) > 0.025
        or bowl[2] - plate[2] > 0.060
        or bowl[2] - plate[2] < -0.010
    ):
        return {
            "status": "failed",
            "reason": "Release pose not verified",
            "grounding": grounding
        }

    policy_gripper(sdk, "open", 16)
    state = sdk.get_state()
    eef = list(state["eef_pos"])
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.12],
        yaw, "open", 25, 0.015
    )
    policy_gripper(sdk, "open", 10)
    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    centered = hypot(bowl[0] - plate[0], bowl[1] - plate[1]) < 0.040
    supported = -0.010 < bowl[2] - plate[2] < 0.065
    if centered and supported:
        return {"status": "done", "object": bowl_name, "grounding": grounding}
    return {
        "status": "failed",
        "reason": "Final placement could not be verified",
        "grounding": grounding
    }
