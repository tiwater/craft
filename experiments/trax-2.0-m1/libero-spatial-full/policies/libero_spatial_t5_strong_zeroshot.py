def policy_move(sdk, pos, yaw, gripper, limit=50, tol=0.008):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        pos,
        yaw=yaw,
        gripper=gripper,
        max_steps=min(limit, remaining),
        tol=tol,
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
    bowls = []
    ramekins = []
    plates = []
    for name in objects:
        if "black_bowl" in name:
            bowls.append(name)
        if "ramekin" in name:
            ramekins.append(name)
        if "plate" in name:
            plates.append(name)

    if not bowls or not ramekins or not plates:
        return {"status": "failed", "reason": "Required objects unavailable"}

    best_score = 1000000.0
    bowl_name = bowls[0]
    for bowl in bowls:
        bp = objects[bowl]["pos"]
        for ramekin in ramekins:
            rp = objects[ramekin]["pos"]
            score = hypot(bp[0] - rp[0], bp[1] - rp[1])
            if bp[2] < rp[2]:
                score += 0.2
            if score < best_score:
                best_score = score
                bowl_name = bowl

    plate_name = plates[0]
    initial_bowl = list(objects[bowl_name]["pos"])
    yaw = state["eef_yaw"]
    held = False

    for attempt in range(2):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 170:
            break

        bp = list(state["objects"][bowl_name]["pos"])
        pp = state["objects"][plate_name]["pos"]
        safe_z = max(bp[2], pp[2]) + 0.15
        ep = state["eef_pos"]

        policy_gripper(sdk, "open", 10)
        if ep[2] < safe_z - 0.02:
            if not policy_move(
                sdk, [ep[0], ep[1], safe_z], yaw, "open", 35
            ):
                return {"status": "failed", "reason": "Unable to gain clearance"}

        grasp_yaw = yaw + attempt * pi / 2.0
        if not policy_move(
            sdk, [bp[0], bp[1], safe_z], grasp_yaw, "open", 45
        ):
            continue

        state = sdk.get_state()
        bp = list(state["objects"][bowl_name]["pos"])
        grasp_z = bp[2] + (0.008 if attempt == 0 else 0.0)
        if not policy_move(
            sdk, [bp[0], bp[1], grasp_z], grasp_yaw, "open", 40, 0.005
        ):
            continue

        policy_gripper(sdk, "close", 16)
        before = sdk.get_state()
        bowl_before = list(before["objects"][bowl_name]["pos"])
        ep = before["eef_pos"]
        policy_move(
            sdk,
            [ep[0], ep[1], max(safe_z, ep[2] + 0.12)],
            grasp_yaw,
            "close",
            45,
        )

        state = sdk.get_state()
        lifted = state["objects"][bowl_name]["pos"]
        ep = state["eef_pos"]
        if (
            lifted[2] - bowl_before[2] > 0.055
            and hypot(lifted[0] - ep[0], lifted[1] - ep[1]) < 0.08
            and abs(lifted[2] - ep[2]) < 0.12
        ):
            held = True
            yaw = grasp_yaw
            break

    if not held:
        return {"status": "failed", "reason": "Bowl did not lift with gripper"}

    state = sdk.get_state()
    bp = state["objects"][bowl_name]["pos"]
    ep = state["eef_pos"]
    offset = [bp[i] - ep[i] for i in range(3)]
    pp = state["objects"][plate_name]["pos"]
    travel_z = max(ep[2], pp[2] + 0.16 - offset[2])
    target = [pp[0] - offset[0], pp[1] - offset[1], travel_z]

    if not policy_move(sdk, target, yaw, "close", 60):
        return {"status": "failed", "reason": "Unable to reach plate"}

    state = sdk.get_state()
    bp = state["objects"][bowl_name]["pos"]
    ep = state["eef_pos"]
    if abs((bp[2] - ep[2]) - offset[2]) > 0.045:
        return {"status": "failed", "reason": "Bowl slipped during transport"}

    pp = state["objects"][plate_name]["pos"]
    target = [
        pp[0] - offset[0],
        pp[1] - offset[1],
        pp[2] + 0.028 - offset[2],
    ]
    if not policy_move(sdk, target, yaw, "close", 45, 0.006):
        return {"status": "failed", "reason": "Unable to lower bowl onto plate"}

    if not policy_gripper(sdk, "open", 15):
        return {"status": "failed", "reason": "Step budget exhausted"}

    state = sdk.get_state()
    ep = state["eef_pos"]
    policy_move(sdk, [ep[0], ep[1], ep[2] + 0.12], yaw, "open", 35)
    policy_gripper(sdk, "open", 10)

    state = sdk.get_state()
    bp = state["objects"][bowl_name]["pos"]
    pp = state["objects"][plate_name]["pos"]
    placed = (
        hypot(bp[0] - pp[0], bp[1] - pp[1]) < 0.045
        and pp[2] - 0.01 < bp[2] < pp[2] + 0.07
        and hypot(bp[0] - initial_bowl[0], bp[1] - initial_bowl[1]) > 0.04
    )
    return {
        "status": "done" if placed else "failed",
        "object": bowl_name,
        "reason": "Placement verified from live poses" if placed else "Placement not verified",
    }
