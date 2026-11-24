def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def distance(a, b):
        return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    def move(pos, yaw, grip, steps=35, tol=0.012):
        budget = remaining()
        if budget <= 0:
            return False
        result = sdk.move_to(
            pos, yaw=yaw, gripper=grip,
            max_steps=min(steps, budget), tol=tol
        )
        return result["reached"]

    def gripper(action, steps=10):
        budget = remaining()
        if budget > 0:
            return sdk.set_gripper(action, steps=min(steps, budget))
        return {}

    state = sdk.get_state()
    objects = state["objects"]
    fixtures = state["fixtures"]
    sites = state["sites"]
    candidates = []
    diagnostics = []

    for cabinet in fixtures:
        region_name = cabinet + "_top_region"
        roof_name = cabinet + "_top_side"
        if "wooden_cabinet" not in cabinet:
            continue
        if region_name not in sites or roof_name not in sites:
            continue
        region = sites[region_name]
        roof = sites[roof_name]
        for name in objects:
            if "black_bowl" not in name:
                continue
            pos = objects[name]["pos"]
            horizontal = hypot(pos[0] - region[0], pos[1] - region[1])
            vertical = pos[2] - region[2]
            eligible = (
                horizontal < 0.10
                and -0.085 < vertical < 0.025
                and pos[2] < roof[2] - 0.025
            )
            diagnostics.append({
                "object": name,
                "region": region_name,
                "object_pos": list(pos),
                "region_pos": list(region),
                "horizontal_distance": horizontal,
                "vertical_offset": vertical,
                "eligible": eligible
            })
            if eligible:
                candidates.append([
                    horizontal + 0.25 * abs(vertical),
                    cabinet, name, region_name
                ])

    plates = [name for name in objects if "plate" in name]
    if not candidates or not plates:
        return {
            "status": "failed",
            "reason": "Required object and top-drawer region not grounded",
            "grounding": diagnostics
        }

    candidates = sorted(candidates)
    cabinet = candidates[0][1]
    target = candidates[0][2]
    region_name = candidates[0][3]
    region = list(sites[region_name])
    roof = list(sites[cabinet + "_top_side"])
    plate_reference = sites.get("main_table_plate_region")
    if plate_reference is None:
        if len(plates) != 1:
            return {"status": "failed", "reason": "Ambiguous plate target"}
        plate = plates[0]
    else:
        plate = sorted([
            [distance(objects[name]["pos"], plate_reference), name]
            for name in plates
        ])[0][1]

    anchor = sites.get(cabinet + "_middle_region", fixtures[cabinet]["pos"])
    dx = region[0] - anchor[0]
    dy = region[1] - anchor[1]
    extension = hypot(dx, dy)
    if extension < 0.065:
        return {
            "status": "failed",
            "reason": "Top drawer lacks verified access",
            "grounding": diagnostics
        }

    front = [dx / extension, dy / extension]
    side = [-front[1], front[0]]
    initial_bowl = list(objects[target]["pos"])
    safe_z = max(roof[2] + 0.12, initial_bowl[2] + 0.17)
    grounding = {
        "target": target,
        "target_site": target + "_default_site",
        "region": region_name,
        "initial_bowl_pos": initial_bowl,
        "initial_region_pos": region,
        "plate": plate,
        "plate_pos": list(objects[plate]["pos"]),
        "candidates": diagnostics
    }
    yaw = atan2(front[1], front[0])
    held = False
    offset = [0.0, 0.0, 0.0]
    failures = []

    gripper("open", 8)
    for attempt in range(3):
        if remaining() < 140:
            break
        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        grasp_yaw = yaw + (pi / 2 if attempt == 1 else 0.0)
        grasp_yaw = atan2(sin(grasp_yaw), cos(grasp_yaw))
        gripper("open", 6)

        current = list(sdk.get_state()["eef_pos"])
        move(
            [current[0], current[1], max(current[2], safe_z)],
            state["eef_yaw"], "open", 35
        )

        approach = [
            bowl[0] + 0.09 * front[0],
            bowl[1] + 0.09 * front[1],
            safe_z
        ]
        move(approach, grasp_yaw, "open", 45)
        bowl = list(sdk.get_state()["objects"][target]["pos"])

        lateral = 0.0
        forward = 0.0
        if attempt == 2:
            forward = 0.025
        grasp_xy = [
            bowl[0] + forward * front[0] + lateral * side[0],
            bowl[1] + forward * front[1] + lateral * side[1]
        ]
        hover = [grasp_xy[0], grasp_xy[1], bowl[2] + 0.09]
        move(hover, grasp_yaw, "open", 35)

        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        height = 0.012
        if attempt == 1:
            height = 0.003
        if attempt == 2:
            height = 0.020
        grasp = [
            bowl[0] + forward * front[0],
            bowl[1] + forward * front[1],
            bowl[2] + height
        ]
        move(grasp, grasp_yaw, "open", 35, 0.006)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        if distance(eef, grasp) > 0.022:
            failures.append({
                "attempt": attempt,
                "phase": "grasp approach",
                "requested": grasp,
                "actual": eef
            })
            continue

        before = list(state["objects"][target]["pos"])
        gripper("close", 14)
        eef = list(sdk.get_state()["eef_pos"])
        move(
            [eef[0], eef[1], eef[2] + 0.045],
            grasp_yaw, "close", 25, 0.008
        )
        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        eef = list(state["eef_pos"])
        first_offset = [bowl[i] - eef[i] for i in range(3)]
        if bowl[2] - before[2] < 0.024 or distance(bowl, eef) > 0.11:
            failures.append({"attempt": attempt, "phase": "test lift"})
            continue

        move(
            [
                eef[0] + 0.055 * front[0],
                eef[1] + 0.055 * front[1],
                eef[2] + 0.025
            ],
            grasp_yaw, "close", 25
        )
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        move(
            [eef[0], eef[1], max(safe_z, eef[2])],
            grasp_yaw, "close", 35
        )
        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        eef = list(state["eef_pos"])
        offset = [bowl[i] - eef[i] for i in range(3)]
        if (
            bowl[2] - before[2] > 0.065
            and distance(offset, first_offset) < 0.028
        ):
            held = True
            yaw = grasp_yaw
            break
        failures.append({"attempt": attempt, "phase": "extraction"})

    if not held:
        return {
            "status": "failed",
            "reason": "Grounded bowl grasp unverified",
            "grounding": grounding,
            "failures": failures
        }

    state = sdk.get_state()
    plate_pos = list(state["objects"][plate]["pos"])
    clearance = max(state["eef_pos"][2], plate_pos[2] + 0.16 - offset[2])
    if not move(
        [plate_pos[0] - offset[0], plate_pos[1] - offset[1], clearance],
        yaw, "close", 50
    ):
        return {
            "status": "failed", "reason": "Plate approach failed",
            "grounding": grounding
        }

    for height in [0.065, 0.020]:
        if remaining() < 30:
            return {"status": "failed", "reason": "Placement budget exhausted"}
        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        eef = list(state["eef_pos"])
        measured = [bowl[i] - eef[i] for i in range(3)]
        if distance(measured, offset) > 0.03:
            return {"status": "failed", "reason": "Bowl slipped"}
        offset = measured
        plate_pos = list(state["objects"][plate]["pos"])
        move(
            [
                plate_pos[0] - offset[0],
                plate_pos[1] - offset[1],
                plate_pos[2] + height - offset[2]
            ],
            yaw, "close", min(30, remaining() - 20), 0.006
        )

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    if (
        hypot(bowl[0] - plate_pos[0], bowl[1] - plate_pos[1]) > 0.03
        or not -0.005 < bowl[2] - plate_pos[2] < 0.05
    ):
        return {"status": "failed", "reason": "Release pose unverified"}

    gripper("open", 10)
    retreat = list(sdk.get_state()["eef_pos"])
    retreat[2] += 0.12
    move(retreat, yaw, "open", 20)
    gripper("open", 8)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    verified = (
        hypot(bowl[0] - plate_pos[0], bowl[1] - plate_pos[1]) < 0.04
        and -0.005 < bowl[2] - plate_pos[2] < 0.055
        and distance(bowl, state["eef_pos"]) > 0.08
        and state["gripper_width"] > 0.04
    )
    return {
        "status": "done" if verified else "failed",
        "reason": "Placement verified" if verified else "Final placement unverified",
        "grounding": grounding,
        "final_bowl_pos": list(bowl),
        "final_plate_pos": list(plate_pos)
    }
