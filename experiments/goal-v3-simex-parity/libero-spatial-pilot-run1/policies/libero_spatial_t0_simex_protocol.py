def policy_move(sdk, pos, yaw, gripper, steps=35, tol=0.008):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        pos, yaw=yaw, gripper=gripper,
        max_steps=min(steps, remaining), tol=tol
    )
    return result["reached"]


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
    bowls = sorted([name for name in objects if "black_bowl" in name])
    plates = sorted([name for name in objects if "plate" in name])
    ramekins = sorted([name for name in objects if "ramekin" in name])
    if not bowls or not plates or not ramekins:
        return {"status": "failed", "reason": "Required objects missing"}

    region_names = sorted([
        name for name in sites if "between_plate_ramekin" in name
    ])
    region_name = region_names[0] if region_names else None
    plate_regions = sorted([
        name for name in sites
        if "plate_region" in name
        and "next_to" not in name
        and "between" not in name
    ])
    plate = plates[0]
    if plate_regions:
        region = sites[plate_regions[0]]
        plate = min(
            plates,
            key=lambda name: (
                (objects[name]["pos"][0] - region[0]) ** 2
                + (objects[name]["pos"][1] - region[1]) ** 2
            )
        )

    plate_pos = objects[plate]["pos"]
    target = None
    ramekin = None
    best_score = 1000000.0
    for ramekin_name in ramekins:
        ramekin_pos = objects[ramekin_name]["pos"]
        vx = ramekin_pos[0] - plate_pos[0]
        vy = ramekin_pos[1] - plate_pos[1]
        length_squared = max(vx * vx + vy * vy, 0.000001)
        for name in bowls:
            pos = objects[name]["pos"]
            t = (
                (pos[0] - plate_pos[0]) * vx
                + (pos[1] - plate_pos[1]) * vy
            ) / length_squared
            bounded = max(0.0, min(1.0, t))
            px = plate_pos[0] + bounded * vx
            py = plate_pos[1] + bounded * vy
            score = (pos[0] - px) ** 2 + (pos[1] - py) ** 2
            score += length_squared * (t - bounded) ** 2
            if region_name is not None:
                region = sites[region_name]
                score += 4.0 * (
                    (pos[0] - region[0]) ** 2
                    + (pos[1] - region[1]) ** 2
                )
            else:
                score += 0.12 * length_squared * (t - 0.5) ** 2
            if score < best_score:
                target = name
                ramekin = ramekin_name
                best_score = score

    initial_bowl_pos = list(objects[target]["pos"])
    initial_plate_pos = list(plate_pos)
    grounding = {
        "target": target,
        "target_site": (
            target + "_default_site"
            if target + "_default_site" in sites else None
        ),
        "plate": plate,
        "ramekin": ramekin,
        "source_region": region_name,
        "initial_target_pos": initial_bowl_pos,
        "initial_plate_pos": initial_plate_pos,
    }
    attempts = []
    base_yaw = state["eef_yaw"]
    yaw = base_yaw
    held = False

    for attempt in range(2):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 220:
            break
        yaw = base_yaw + attempt * pi / 2.0
        pos = list(state["objects"][target]["pos"])
        eef = state["eef_pos"]
        clearance = max(eef[2], pos[2] + 0.12)
        policy_move(
            sdk, [eef[0], eef[1], clearance],
            yaw, "open", 20, 0.012
        )
        policy_gripper(sdk, "open", 6)
        pos = sdk.get_state()["objects"][target]["pos"]
        policy_move(
            sdk, [pos[0], pos[1], clearance],
            yaw, "open", 30, 0.008
        )
        pos = list(sdk.get_state()["objects"][target]["pos"])
        grasp_pos = [pos[0], pos[1], pos[2] + (0.012 if attempt == 0 else 0.003)]
        policy_move(sdk, grasp_pos, yaw, "open", 38, 0.006)

        state = sdk.get_state()
        eef = state["eef_pos"]
        pos = state["objects"][target]["pos"]
        if (
            hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.018
            or abs(eef[2] - grasp_pos[2]) > 0.022
        ):
            attempts.append({"attempt": attempt, "reason": "Grasp pose missed"})
            continue

        baseline = list(pos)
        policy_gripper(sdk, "close", 16)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        before_lift = list(state["objects"][target]["pos"])
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.085],
            yaw, "close", 38, 0.008
        )
        state = sdk.get_state()
        pos = state["objects"][target]["pos"]
        lift = pos[2] - baseline[2]
        tracked_lift = pos[2] - before_lift[2]
        eef_lift = state["eef_pos"][2] - eef[2]
        attempts.append({
            "attempt": attempt,
            "lift": lift,
            "tracked_lift": tracked_lift,
            "eef_lift": eef_lift,
            "gripper_width": state["gripper_width"],
        })
        if (
            lift > 0.025
            and tracked_lift > 0.020
            and abs(tracked_lift - eef_lift) < 0.035
            and state["gripper_width"] > 0.012
        ):
            held = True
            break
        policy_gripper(sdk, "open", 6)

    if not held:
        return {
            "status": "failed",
            "reason": "No verified grasp",
            "grounding": grounding,
            "attempts": attempts,
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    offset = [bowl_pos[i] - eef[i] for i in range(3)]
    plate_pos = state["objects"][plate]["pos"]
    support_z = plate_pos[2] + initial_bowl_pos[2] - initial_plate_pos[2]
    transit_z = max(eef[2], support_z + 0.09 - offset[2])
    policy_move(
        sdk, [eef[0], eef[1], transit_z],
        yaw, "close", 20, 0.01
    )
    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    plate_pos = state["objects"][plate]["pos"]
    policy_move(
        sdk,
        [eef[0] + plate_pos[0] - bowl_pos[0],
         eef[1] + plate_pos[1] - bowl_pos[1], transit_z],
        yaw, "close", 42, 0.007
    )

    for correction in range(2):
        state = sdk.get_state()
        bowl_pos = state["objects"][target]["pos"]
        plate_pos = state["objects"][plate]["pos"]
        eef = state["eef_pos"]
        if bowl_pos[2] < initial_bowl_pos[2] + 0.025:
            return {
                "status": "failed", "reason": "Bowl slipped",
                "grounding": grounding, "attempts": attempts,
            }
        dx = plate_pos[0] - bowl_pos[0]
        dy = plate_pos[1] - bowl_pos[1]
        if hypot(dx, dy) < 0.009:
            break
        policy_move(
            sdk, [eef[0] + dx, eef[1] + dy, eef[2]],
            yaw, "close", 18, 0.006
        )

    for height in [0.04, 0.014]:
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 55:
            break
        bowl_pos = state["objects"][target]["pos"]
        plate_pos = state["objects"][plate]["pos"]
        eef = state["eef_pos"]
        support_z = plate_pos[2] + initial_bowl_pos[2] - initial_plate_pos[2]
        policy_move(
            sdk,
            [eef[0] + plate_pos[0] - bowl_pos[0],
             eef[1] + plate_pos[1] - bowl_pos[1],
             eef[2] + support_z + height - bowl_pos[2]],
            yaw, "close", 28, 0.006
        )

    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    support_z = plate_pos[2] + initial_bowl_pos[2] - initial_plate_pos[2]
    if (
        hypot(bowl_pos[0] - plate_pos[0], bowl_pos[1] - plate_pos[1]) > 0.025
        or bowl_pos[2] > support_z + 0.05
    ):
        return {
            "status": "failed", "reason": "Release pose missed",
            "grounding": grounding,
            "final_target_pos": list(bowl_pos),
            "plate_pos": list(plate_pos),
            "attempts": attempts,
        }

    policy_gripper(sdk, "open", 12)
    eef = sdk.get_state()["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.10],
        yaw, "open", 22, 0.012
    )
    policy_gripper(sdk, "open", 10)
    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    support_z = plate_pos[2] + initial_bowl_pos[2] - initial_plate_pos[2]
    centered = hypot(
        bowl_pos[0] - plate_pos[0], bowl_pos[1] - plate_pos[1]
    ) < 0.035
    settled = support_z - 0.008 < bowl_pos[2] < support_z + 0.035
    return {
        "status": "done" if centered and settled else "failed",
        "grounding": grounding,
        "final_target_pos": list(bowl_pos),
        "plate_pos": list(plate_pos),
        "attempts": attempts,
        "steps_used": state["steps_used"],
    }
