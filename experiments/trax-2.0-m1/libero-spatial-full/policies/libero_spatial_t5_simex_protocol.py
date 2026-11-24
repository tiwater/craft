def policy_move(sdk, pos, yaw, gripper, limit=45, tol=0.008):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        pos, yaw=yaw, gripper=gripper,
        max_steps=min(limit, remaining), tol=tol
    )
    return result["reached"]


def policy_gripper(sdk, action, steps=12):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    sdk.set_gripper(action, steps=min(steps, remaining))
    return True


def object_site_position(state, name):
    position = state["objects"][name]["pos"]
    site_name = name + "_default_site"
    if site_name in state["sites"]:
        site = state["sites"][site_name]
        if (
            hypot(site[0] - position[0], site[1] - position[1]) < 0.025
            and abs(site[2] - position[2]) < 0.025
        ):
            return list(site)
    return list(position)


def run_policy(sdk):
    state = sdk.get_state()
    bowls = []
    ramekins = []
    plates = []
    for name in state["objects"]:
        if "black_bowl" in name:
            bowls.append(name)
        if "ramekin" in name:
            ramekins.append(name)
        if "plate" in name:
            plates.append(name)

    if not bowls or not ramekins or not plates:
        return {"status": "failed", "reason": "Required objects unavailable"}

    bowl_name = ""
    ramekin_name = ""
    best_score = 1000000.0
    for bowl in bowls:
        bp = object_site_position(state, bowl)
        for ramekin in ramekins:
            rp = object_site_position(state, ramekin)
            separation = hypot(bp[0] - rp[0], bp[1] - rp[1])
            height = bp[2] - rp[2]
            if separation < 0.065 and 0.005 < height < 0.15:
                score = separation + 0.2 * abs(height - 0.035)
                if score < best_score:
                    best_score = score
                    bowl_name = bowl
                    ramekin_name = ramekin

    if not bowl_name:
        return {
            "status": "failed",
            "reason": "No black bowl geometrically supported by a ramekin",
        }

    plate_name = plates[0]
    if len(plates) > 1:
        region_name = "main_table_plate_region"
        if region_name not in state["sites"]:
            return {"status": "failed", "reason": "Ambiguous destination plate"}
        region = state["sites"][region_name]
        best_score = 1000000.0
        for plate in plates:
            pp = object_site_position(state, plate)
            score = hypot(pp[0] - region[0], pp[1] - region[1])
            if score < best_score:
                best_score = score
                plate_name = plate

    initial_bowl = object_site_position(state, bowl_name)
    initial_ramekin = object_site_position(state, ramekin_name)
    base_yaw = state["eef_yaw"]
    yaw = base_yaw
    held = False

    for attempt in range(2):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 175:
            break

        bp = object_site_position(state, bowl_name)
        pp = object_site_position(state, plate_name)
        safe_z = max(bp[2], pp[2]) + 0.15
        ep = state["eef_pos"]
        grasp_yaw = base_yaw + attempt * pi / 2.0

        policy_gripper(sdk, "open", 10)
        if ep[2] < safe_z - 0.015:
            if not policy_move(
                sdk, [ep[0], ep[1], safe_z], base_yaw, "open", 35
            ):
                return {"status": "failed", "reason": "Unable to gain clearance"}

        if not policy_move(
            sdk, [bp[0], bp[1], safe_z], grasp_yaw, "open", 40
        ):
            continue

        state = sdk.get_state()
        bp = object_site_position(state, bowl_name)
        grasp_height = 0.035 if attempt == 0 else 0.022
        target = [bp[0], bp[1], bp[2] + grasp_height]
        reached = policy_move(sdk, target, grasp_yaw, "open", 35, 0.007)

        state = sdk.get_state()
        ep = state["eef_pos"]
        if not reached:
            if (
                hypot(ep[0] - target[0], ep[1] - target[1]) > 0.012
                or abs(ep[2] - target[2]) > 0.015
            ):
                continue

        bowl_before = object_site_position(state, bowl_name)
        ramekin_before = object_site_position(state, ramekin_name)
        policy_gripper(sdk, "close", 16)

        state = sdk.get_state()
        ep = state["eef_pos"]
        policy_move(
            sdk,
            [ep[0], ep[1], max(safe_z, ep[2] + 0.11)],
            grasp_yaw, "close", 40
        )

        state = sdk.get_state()
        lifted = object_site_position(state, bowl_name)
        rp = object_site_position(state, ramekin_name)
        ep = state["eef_pos"]
        if (
            lifted[2] - bowl_before[2] > 0.05
            and hypot(lifted[0] - ep[0], lifted[1] - ep[1]) < 0.07
            and abs(lifted[2] - ep[2]) < 0.12
            and rp[2] - ramekin_before[2] < 0.025
        ):
            held = True
            yaw = grasp_yaw
            break

    if not held:
        return {
            "status": "failed",
            "object": bowl_name,
            "support": ramekin_name,
            "reason": "Selected bowl did not lift independently",
        }

    state = sdk.get_state()
    bp = object_site_position(state, bowl_name)
    pp = object_site_position(state, plate_name)
    ep = state["eef_pos"]
    offset = [bp[i] - ep[i] for i in range(3)]
    travel_z = max(ep[2], pp[2] + 0.15 - offset[2])

    reached = policy_move(
        sdk,
        [pp[0] - offset[0], pp[1] - offset[1], travel_z],
        yaw, "close", 55
    )
    state = sdk.get_state()
    bp = object_site_position(state, bowl_name)
    pp = object_site_position(state, plate_name)
    ep = state["eef_pos"]

    if (
        abs(bp[2] - ep[2] - offset[2]) > 0.035
        or hypot(bp[0] - ep[0] - offset[0], bp[1] - ep[1] - offset[1]) > 0.03
    ):
        return {"status": "failed", "reason": "Bowl slipped during transport"}

    if not reached or hypot(bp[0] - pp[0], bp[1] - pp[1]) > 0.008:
        policy_move(
            sdk,
            [ep[0] + pp[0] - bp[0], ep[1] + pp[1] - bp[1], travel_z],
            yaw, "close", 25, 0.005
        )

    state = sdk.get_state()
    bp = object_site_position(state, bowl_name)
    pp = object_site_position(state, plate_name)
    ep = state["eef_pos"]
    if hypot(bp[0] - pp[0], bp[1] - pp[1]) > 0.02:
        return {"status": "failed", "reason": "Unable to align above plate"}

    offset = [bp[i] - ep[i] for i in range(3)]
    target = [
        pp[0] - offset[0],
        pp[1] - offset[1],
        pp[2] + 0.028 - offset[2],
    ]
    policy_move(sdk, target, yaw, "close", 40, 0.007)

    for correction in range(2):
        state = sdk.get_state()
        bp = object_site_position(state, bowl_name)
        pp = object_site_position(state, plate_name)
        ep = state["eef_pos"]
        lateral_error = hypot(bp[0] - pp[0], bp[1] - pp[1])
        height = bp[2] - pp[2]

        if lateral_error < 0.007 and 0.012 < height < 0.035:
            break
        if height < 0.008 or height > 0.085:
            break
        if state["max_steps"] - state["steps_used"] < 65:
            break

        target = [
            ep[0] + max(-0.015, min(0.015, pp[0] - bp[0])),
            ep[1] + max(-0.015, min(0.015, pp[1] - bp[1])),
            ep[2] + max(-0.02, min(0.01, 0.024 - height)),
        ]
        policy_move(sdk, target, yaw, "close", 20, 0.005)

    state = sdk.get_state()
    bp = object_site_position(state, bowl_name)
    pp = object_site_position(state, plate_name)
    if (
        hypot(bp[0] - pp[0], bp[1] - pp[1]) > 0.025
        or not pp[2] - 0.005 < bp[2] < pp[2] + 0.05
    ):
        return {"status": "failed", "reason": "Release pose outside placement tolerance"}

    if not policy_gripper(sdk, "open", 15):
        return {"status": "failed", "reason": "Step budget exhausted"}

    state = sdk.get_state()
    ep = state["eef_pos"]
    policy_move(sdk, [ep[0], ep[1], ep[2] + 0.12], yaw, "open", 30)
    policy_gripper(sdk, "open", 12)

    state = sdk.get_state()
    bp = object_site_position(state, bowl_name)
    pp = object_site_position(state, plate_name)
    rp = object_site_position(state, ramekin_name)
    placed = (
        hypot(bp[0] - pp[0], bp[1] - pp[1]) < 0.045
        and pp[2] - 0.01 < bp[2] < pp[2] + 0.065
        and hypot(bp[0] - initial_bowl[0], bp[1] - initial_bowl[1]) > 0.04
        and abs(rp[2] - initial_ramekin[2]) < 0.025
    )
    return {
        "status": "done" if placed else "failed",
        "object": bowl_name,
        "support": ramekin_name,
        "destination": plate_name,
        "object_position": bp,
        "target_position": pp,
        "reason": "Placement verified from live poses" if placed else "Placement not verified",
    }
