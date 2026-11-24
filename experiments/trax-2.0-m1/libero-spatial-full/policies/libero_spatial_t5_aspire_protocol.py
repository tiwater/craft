def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return state["max_steps"] - state["steps_used"]

    def move(pos, yaw, grip, steps=40, tol=0.008):
        budget = remaining()
        if budget <= 0:
            return False
        result = sdk.move_to(
            pos, yaw=yaw, gripper=grip,
            max_steps=min(steps, budget), tol=tol
        )
        return result["reached"]

    def gripper(action, steps=12):
        budget = remaining()
        if budget <= 0:
            return False
        sdk.set_gripper(action, steps=min(steps, budget))
        return True

    state = sdk.get_state()
    objects = state["objects"]
    bowls = [name for name in objects if "black_bowl" in name]
    ramekins = [name for name in objects if "ramekin" in name]
    plates = [name for name in objects if "plate" in name]
    if not bowls or not ramekins or not plates:
        return {"status": "failed", "reason": "Required objects unavailable"}

    bowl_name = bowls[0]
    best = 1000000.0
    for name in bowls:
        bp = objects[name]["pos"]
        for support in ramekins:
            rp = objects[support]["pos"]
            score = hypot(bp[0] - rp[0], bp[1] - rp[1])
            if bp[2] < rp[2]:
                score += 0.25
            if score < best:
                best = score
                bowl_name = name

    plate_name = plates[0]
    initial = list(objects[bowl_name]["pos"])
    base_yaw = state["eef_yaw"]
    held = False
    yaw = base_yaw

    # Body-origin grasps can hit the support before the fingers surround
    # the bowl. Approach higher, then try a single-wall rim pinch.
    strategies = [
        [0.0, 0.045, 0.0],
        [0.035, 0.035, 0.0],
        [0.035, 0.025, pi / 2.0],
    ]

    for strategy in strategies:
        if remaining() < 150:
            break
        state = sdk.get_state()
        bp = list(state["objects"][bowl_name]["pos"])
        pp = state["objects"][plate_name]["pos"]
        ep = state["eef_pos"]
        yaw = base_yaw + strategy[2]
        safe_z = max(bp[2] + 0.15, pp[2] + 0.16, ep[2])

        gripper("open", 8)
        if ep[2] < safe_z - 0.015:
            move([ep[0], ep[1], safe_z], yaw, "open", 28)

        # Panda finger separation is along the local y axis.
        gx = bp[0] - sin(yaw) * strategy[0]
        gy = bp[1] + cos(yaw) * strategy[0]
        if not move([gx, gy, safe_z], yaw, "open", 38):
            continue

        state = sdk.get_state()
        bp = list(state["objects"][bowl_name]["pos"])
        gx = bp[0] - sin(yaw) * strategy[0]
        gy = bp[1] + cos(yaw) * strategy[0]
        target = [gx, gy, bp[2] + strategy[1]]
        move(target, yaw, "open", 32, 0.005)
        state = sdk.get_state()
        ep = state["eef_pos"]
        if hypot(ep[0] - gx, ep[1] - gy) > 0.018:
            continue
        if abs(ep[2] - target[2]) > 0.025:
            continue

        before = list(state["objects"][bowl_name]["pos"])
        gripper("close", 16)
        state = sdk.get_state()
        ep = list(state["eef_pos"])
        move([ep[0], ep[1], ep[2] + 0.085], yaw, "close", 28)

        state = sdk.get_state()
        bp = state["objects"][bowl_name]["pos"]
        ep = state["eef_pos"]
        if (
            bp[2] - before[2] > 0.04
            and hypot(bp[0] - ep[0], bp[1] - ep[1]) < 0.10
            and abs(bp[2] - ep[2]) < 0.14
        ):
            held = True
            break

    if not held:
        return {"status": "failed", "reason": "No verified bowl grasp"}

    state = sdk.get_state()
    bp = state["objects"][bowl_name]["pos"]
    ep = state["eef_pos"]
    offset = [bp[i] - ep[i] for i in range(3)]
    pp = state["objects"][plate_name]["pos"]
    travel_z = max(ep[2], initial[2] + 0.14 - offset[2],
                   pp[2] + 0.17 - offset[2])

    if not move([ep[0], ep[1], travel_z], yaw, "close", 28):
        return {"status": "failed", "reason": "Unable to clear support"}

    state = sdk.get_state()
    bp = state["objects"][bowl_name]["pos"]
    ep = state["eef_pos"]
    if max(abs(bp[i] - ep[i] - offset[i]) for i in range(3)) > 0.035:
        return {"status": "failed", "reason": "Grasp lost during lift"}

    pp = state["objects"][plate_name]["pos"]
    if not move(
        [pp[0] - offset[0], pp[1] - offset[1], travel_z],
        yaw, "close", 55
    ):
        return {"status": "failed", "reason": "Unable to reach plate"}

    state = sdk.get_state()
    bp = state["objects"][bowl_name]["pos"]
    ep = state["eef_pos"]
    if max(abs(bp[i] - ep[i] - offset[i]) for i in range(3)) > 0.035:
        return {"status": "failed", "reason": "Grasp lost during transport"}

    offset = [bp[i] - ep[i] for i in range(3)]
    pp = state["objects"][plate_name]["pos"]
    target = [
        pp[0] - offset[0],
        pp[1] - offset[1],
        pp[2] + 0.045 - offset[2],
    ]
    move(target, yaw, "close", 38, 0.006)

    state = sdk.get_state()
    bp = state["objects"][bowl_name]["pos"]
    pp = state["objects"][plate_name]["pos"]
    if (
        hypot(bp[0] - pp[0], bp[1] - pp[1]) > 0.035
        or bp[2] > pp[2] + 0.09
    ):
        return {"status": "failed", "reason": "Release pose not reached"}

    gripper("open", 14)
    state = sdk.get_state()
    ep = state["eef_pos"]
    move([ep[0], ep[1], ep[2] + 0.12], yaw, "open", 28)
    gripper("open", 12)

    state = sdk.get_state()
    bp = state["objects"][bowl_name]["pos"]
    pp = state["objects"][plate_name]["pos"]
    placed = (
        hypot(bp[0] - pp[0], bp[1] - pp[1]) < 0.045
        and pp[2] - 0.01 < bp[2] < pp[2] + 0.075
    )
    return {
        "status": "done" if placed else "failed",
        "object": bowl_name,
        "reason": "Placement verified" if placed else "Placement not verified",
    }
