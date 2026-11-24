def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(450, state["max_steps"]) - state["steps_used"])


def distance_xyz(a, b):
    return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def move(sdk, pos, yaw, grip, steps=35, tol=0.012):
    budget = remaining_steps(sdk)
    if budget <= 0:
        return False
    result = sdk.move_to(
        pos, yaw=yaw, gripper=grip,
        max_steps=min(steps, budget), tol=tol
    )
    return result["reached"]


def actuate(sdk, action, steps=12):
    budget = remaining_steps(sdk)
    if budget <= 0:
        return False
    sdk.set_gripper(action, steps=min(steps, budget))
    return True


def radial_angle(state, target, attempted):
    bp = state["objects"][target]["pos"]
    obstacles = []
    for name in state["objects"]:
        if name != target:
            obstacles.append(state["objects"][name]["pos"])
    for name in state["fixtures"]:
        obstacles.append(state["fixtures"][name]["pos"])
        for site in state["sites"]:
            if name in site:
                obstacles.append(state["sites"][site])

    best_angle = None
    best_clearance = -1.0
    for index in range(72):
        angle = 2.0 * pi * index / 72.0
        distinct = True
        for previous in attempted:
            separation = abs(atan2(sin(angle - previous), cos(angle - previous)))
            if separation < pi / 4.0:
                distinct = False
        if distinct:
            contact = [
                bp[0] + 0.04 * cos(angle),
                bp[1] + 0.04 * sin(angle),
                bp[2] + 0.02,
            ]
            # The SDK exposes positions, not collision geometry.
            # Maximize minimum live positional clearance to every neighbor.
            clearance = 1000000.0
            for obstacle in obstacles:
                clearance = min(clearance, distance_xyz(contact, obstacle))
            if clearance > best_clearance:
                best_clearance = clearance
                best_angle = angle
    return best_angle


def corridor_height(state, start, finish):
    height = max(start[2], finish[2]) + 0.12
    dx = finish[0] - start[0]
    dy = finish[1] - start[1]
    length_squared = dx * dx + dy * dy
    points = []
    for name in state["objects"]:
        points.append(state["objects"][name]["pos"])
    for name in state["fixtures"]:
        points.append(state["fixtures"][name]["pos"])
        for site in state["sites"]:
            if name in site:
                points.append(state["sites"][site])
    for point in points:
        fraction = 0.0
        if length_squared > 0.000001:
            fraction = max(
                0.0,
                min(1.0, ((point[0] - start[0]) * dx +
                          (point[1] - start[1]) * dy) / length_squared)
            )
        separation = hypot(
            point[0] - start[0] - fraction * dx,
            point[1] - start[1] - fraction * dy
        )
        if separation < 0.16:
            height = max(height, point[2] + 0.12)
    return height


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

    bowl_name = bowls[0]
    best_score = 1000000.0
    for bowl in bowls:
        bp = state["objects"][bowl]["pos"]
        for ramekin in ramekins:
            rp = state["objects"][ramekin]["pos"]
            score = hypot(bp[0] - rp[0], bp[1] - rp[1])
            if bp[2] < rp[2]:
                score += 0.2
            if score < best_score:
                best_score = score
                bowl_name = bowl
    plate_name = plates[0]
    attempted = []
    held = False
    yaw = state["eef_yaw"]
    lift_reference = 0.0

    for attempt in range(3):
        if remaining_steps(sdk) < 210:
            break
        state = sdk.get_state()
        bp = list(state["objects"][bowl_name]["pos"])
        angle = radial_angle(state, bowl_name, attempted)
        if angle is None:
            break
        attempted.append(angle)

        # Probe-grounded radius, height, and perpendicular jaw alignment.
        yaw = angle + pi / 2.0
        while yaw > pi / 2.0:
            yaw -= pi
        while yaw < -pi / 2.0:
            yaw += pi

        ep = list(state["eef_pos"])
        safe_z = corridor_height(state, ep, bp)
        actuate(sdk, "open", 8)
        move(sdk, [ep[0], ep[1], safe_z],
             state["eef_yaw"], "open", 25)
        state = sdk.get_state()
        if state["eef_pos"][2] < bp[2] + 0.075:
            continue

        contact = [
            bp[0] + 0.04 * cos(angle),
            bp[1] + 0.04 * sin(angle),
            bp[2] + 0.02,
        ]
        move(sdk, [contact[0], contact[1], safe_z], yaw, "open", 32)
        state = sdk.get_state()
        if hypot(state["eef_pos"][0] - contact[0],
                 state["eef_pos"][1] - contact[1]) > 0.035:
            continue

        bp = list(state["objects"][bowl_name]["pos"])
        contact = [
            bp[0] + 0.04 * cos(angle),
            bp[1] + 0.04 * sin(angle),
            bp[2] + 0.02,
        ]
        move(sdk, contact, yaw, "open", 34, 0.008)
        state = sdk.get_state()
        if distance_xyz(state["eef_pos"], contact) > 0.025:
            bp = list(state["objects"][bowl_name]["pos"])
            contact = [
                bp[0] + 0.04 * cos(angle),
                bp[1] + 0.04 * sin(angle),
                bp[2] + 0.02,
            ]
            move(sdk, contact, yaw, "open", 10, 0.012)

        state = sdk.get_state()
        bp = list(state["objects"][bowl_name]["pos"])
        ep = list(state["eef_pos"])
        # Contact can prevent exact controller convergence; still actuate.
        if distance_xyz(ep, contact) > 0.05 or distance_xyz(ep, bp) > 0.095:
            continue

        lift_reference = bp[2]
        actuate(sdk, "close", 14)
        state = sdk.get_state()
        ep = list(state["eef_pos"])
        move(sdk, [ep[0], ep[1], ep[2] + 0.10],
             yaw, "close", 32, 0.01)
        state = sdk.get_state()
        bp = state["objects"][bowl_name]["pos"]
        ep = state["eef_pos"]
        if bp[2] - lift_reference >= 0.03 and distance_xyz(bp, ep) <= 0.11:
            held = True
            break

    if not held:
        return {"status": "failed", "reason": "No verified live bowl lift"}

    state = sdk.get_state()
    bp = list(state["objects"][bowl_name]["pos"])
    ep = list(state["eef_pos"])
    pp = list(state["objects"][plate_name]["pos"])
    offset = [bp[i] - ep[i] for i in range(3)]
    travel_z = max(ep[2], corridor_height(state, bp, pp) - offset[2])
    move(sdk, [ep[0], ep[1], travel_z], yaw, "close", 22)

    state = sdk.get_state()
    bp = state["objects"][bowl_name]["pos"]
    ep = state["eef_pos"]
    if bp[2] - lift_reference < 0.03 or distance_xyz(bp, ep) > 0.11:
        return {"status": "failed", "reason": "Lift lost before transport"}

    pp = state["objects"][plate_name]["pos"]
    target = [pp[0] - offset[0], pp[1] - offset[1], travel_z]
    move(sdk, target, yaw, "close", 45, 0.01)

    state = sdk.get_state()
    bp = state["objects"][bowl_name]["pos"]
    ep = state["eef_pos"]
    pp = state["objects"][plate_name]["pos"]
    if distance_xyz(bp, ep) > 0.11 or bp[2] - lift_reference < 0.03:
        return {"status": "failed", "reason": "Bowl lost during transport"}

    # Correct centering using the live bowl-to-EEF displacement.
    offset = [bp[i] - ep[i] for i in range(3)]
    if hypot(bp[0] - pp[0], bp[1] - pp[1]) > 0.018:
        move(sdk, [pp[0] - offset[0], pp[1] - offset[1], ep[2]],
             yaw, "close", 18, 0.008)

    state = sdk.get_state()
    bp = state["objects"][bowl_name]["pos"]
    ep = state["eef_pos"]
    pp = state["objects"][plate_name]["pos"]
    if hypot(bp[0] - pp[0], bp[1] - pp[1]) > 0.035:
        return {"status": "failed", "reason": "Bowl not centered above plate"}

    offset = [bp[i] - ep[i] for i in range(3)]
    move(sdk, [pp[0] - offset[0], pp[1] - offset[1],
               pp[2] + 0.028 - offset[2]],
         yaw, "close", 32, 0.008)

    state = sdk.get_state()
    bp = state["objects"][bowl_name]["pos"]
    pp = state["objects"][plate_name]["pos"]
    if hypot(bp[0] - pp[0], bp[1] - pp[1]) > 0.04 or bp[2] > pp[2] + 0.065:
        return {"status": "failed", "reason": "Release pose not reached"}
    if not actuate(sdk, "open", 12):
        return {"status": "failed", "reason": "Budget exhausted before release"}

    state = sdk.get_state()
    ep = state["eef_pos"]
    move(sdk, [ep[0], ep[1], ep[2] + 0.11], yaw, "open", 23)
    actuate(sdk, "open", 8)
    state = sdk.get_state()
    bp = state["objects"][bowl_name]["pos"]
    pp = state["objects"][plate_name]["pos"]
    placed = (
        hypot(bp[0] - pp[0], bp[1] - pp[1]) < 0.045
        and pp[2] - 0.01 < bp[2] < pp[2] + 0.07
        and state["eef_pos"][2] - bp[2] > 0.07
    )
    return {
        "status": "done" if placed else "failed",
        "object": bowl_name,
        "reason": "Placement verified from live poses" if placed else "Placement not verified",
    }
