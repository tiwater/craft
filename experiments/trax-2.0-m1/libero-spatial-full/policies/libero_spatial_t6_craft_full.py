def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def distance(a, b):
        return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    def move(pos, yaw, grip, steps=35, tol=0.015):
        budget = min(steps, remaining())
        if budget <= 0:
            return False
        return sdk.move_to(
            pos, yaw=yaw, gripper=grip, max_steps=budget, tol=tol
        )["reached"]

    def actuate(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def obstacles(state, target):
        points = []
        for name in state["objects"]:
            if name != target:
                points.append(list(state["objects"][name]["pos"]))
        for name in state["fixtures"]:
            points.append(list(state["fixtures"][name]["pos"]))
            for site in state["sites"]:
                if name in site:
                    points.append(list(state["sites"][site]))
        return points

    def segment_distance(point, start, end):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_squared = dx * dx + dy * dy
        fraction = 0.0
        if length_squared > 0.000001:
            fraction = max(0.0, min(
                1.0,
                ((point[0] - start[0]) * dx +
                 (point[1] - start[1]) * dy) / length_squared
            ))
        return hypot(
            point[0] - start[0] - fraction * dx,
            point[1] - start[1] - fraction * dy
        )

    def travel_height(state, target, start, end, margin):
        height = max(start[2], end[2])
        for point in obstacles(state, target):
            if segment_distance(point, start, end) < 0.14:
                height = max(height, point[2] + margin)
        return height

    def choose_angle(state, target, radius, used):
        center = state["objects"][target]["pos"]
        points = obstacles(state, target)
        best_angle = None
        best_clearance = -1.0
        for index in range(144):
            angle = 2.0 * pi * index / 144.0
            separated = True
            for previous in used:
                separation = abs(atan2(
                    sin(angle - previous), cos(angle - previous)
                ))
                if separation < pi / 3.0:
                    separated = False
            if not separated:
                continue
            x = center[0] + radius * cos(angle)
            y = center[1] + radius * sin(angle)
            clearance = 1000.0
            for point in points:
                clearance = min(
                    clearance, hypot(x - point[0], y - point[1])
                )
            if clearance > best_clearance:
                best_clearance = clearance
                best_angle = angle
        return best_angle

    state = sdk.get_state()
    objects = state["objects"]
    bowls = [
        name for name in objects
        if "black" in name.lower() and "bowl" in name.lower()
    ]
    cookies = [name for name in objects if "cookie" in name.lower()]
    plates = [name for name in objects if "plate" in name.lower()]
    if not bowls or not cookies or not plates:
        return {"status": "failed", "reason": "Required objects are missing"}

    target = bowls[0]
    nearest = 1000.0
    for bowl in bowls:
        for cookie in cookies:
            bowl_pos = objects[bowl]["pos"]
            cookie_pos = objects[cookie]["pos"]
            separation = hypot(
                bowl_pos[0] - cookie_pos[0],
                bowl_pos[1] - cookie_pos[1]
            )
            if separation < nearest:
                nearest = separation
                target = bowl
    plate = plates[0]
    used_angles = []
    held = False
    grasp_yaw = 0.0
    lift_reference = objects[target]["pos"][2]

    for attempt in range(3):
        if remaining() < 165:
            break

        state = sdk.get_state()
        radius = 0.04
        z_offset = 0.02
        if attempt == 2:
            radius = 0.045
            z_offset = 0.025

        angle = choose_angle(state, target, radius, used_angles)
        if angle is None:
            break
        used_angles.append(angle)

        # Parallel jaws are equivalent modulo pi. Keep the measured
        # tangential alignment without using the object's arbitrary yaw.
        grasp_yaw = angle + pi / 2.0
        while grasp_yaw > pi / 2.0:
            grasp_yaw -= pi
        while grasp_yaw < -pi / 2.0:
            grasp_yaw += pi

        actuate("open", 6)
        state = sdk.get_state()
        bowl_pos = list(state["objects"][target]["pos"])
        contact = [
            bowl_pos[0] + radius * cos(angle),
            bowl_pos[1] + radius * sin(angle),
            bowl_pos[2] + z_offset
        ]
        eef = list(state["eef_pos"])
        hover = [contact[0], contact[1], bowl_pos[2] + 0.14]
        height = travel_height(state, target, eef, hover, 0.075)

        if eef[2] < height - 0.025:
            move([eef[0], eef[1], height], grasp_yaw, "open", 20, 0.025)
        move([hover[0], hover[1], height], grasp_yaw, "open", 40, 0.02)

        state = sdk.get_state()
        eef = state["eef_pos"]
        lateral_error = hypot(
            eef[0] - hover[0], eef[1] - hover[1]
        )
        if lateral_error > 0.035:
            move([hover[0], hover[1], height], grasp_yaw, "open", 18, 0.02)
            eef = sdk.get_state()["eef_pos"]
            if hypot(eef[0] - hover[0], eef[1] - hover[1]) > 0.045:
                continue

        state = sdk.get_state()
        bowl_pos = list(state["objects"][target]["pos"])
        lift_reference = bowl_pos[2]
        contact = [
            bowl_pos[0] + radius * cos(angle),
            bowl_pos[1] + radius * sin(angle),
            bowl_pos[2] + z_offset
        ]
        move(contact, grasp_yaw, "open", 40, 0.01)

        state = sdk.get_state()
        bowl_pos = state["objects"][target]["pos"]
        contact = [
            bowl_pos[0] + radius * cos(angle),
            bowl_pos[1] + radius * sin(angle),
            bowl_pos[2] + z_offset
        ]
        if distance(state["eef_pos"], contact) > 0.018 and remaining() > 145:
            move(contact, grasp_yaw, "open", 15, 0.012)

        state = sdk.get_state()
        # Use live positional error, not the controller's reached flag.
        if distance(state["eef_pos"], contact) > 0.035:
            continue

        actuate("close", 12)
        eef = list(sdk.get_state()["eef_pos"])
        move(
            [eef[0], eef[1], eef[2] + 0.10],
            grasp_yaw, "close", 28, 0.015
        )
        state = sdk.get_state()
        bowl_pos = state["objects"][target]["pos"]
        held = (
            bowl_pos[2] - lift_reference >= 0.03 and
            distance(bowl_pos, state["eef_pos"]) <= 0.10
        )
        if held:
            break

    if not held:
        return {"status": "failed", "reason": "No verified object lift"}

    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    eef = list(state["eef_pos"])
    offset = [bowl_pos[i] - eef[i] for i in range(3)]
    plate_pos = state["objects"][plate]["pos"]
    destination = [
        plate_pos[0] - offset[0],
        plate_pos[1] - offset[1],
        plate_pos[2] + 0.14 - offset[2]
    ]
    height = travel_height(
        state, target, eef, destination, 0.10 + max(0.0, -offset[2])
    )
    move([eef[0], eef[1], height], grasp_yaw, "close", 22, 0.02)
    eef = sdk.get_state()["eef_pos"]
    if eef[2] < height - 0.035:
        return {"status": "failed", "reason": "Transport clearance not reached"}

    move(
        [destination[0], destination[1], height],
        grasp_yaw, "close", 45, 0.015
    )

    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    if (
        bowl_pos[2] - lift_reference < 0.03 or
        distance(bowl_pos, eef) > 0.10
    ):
        return {"status": "failed", "reason": "Lost verified lift during transport"}

    plate_pos = state["objects"][plate]["pos"]
    offset = [bowl_pos[i] - eef[i] for i in range(3)]
    if hypot(bowl_pos[0] - plate_pos[0], bowl_pos[1] - plate_pos[1]) > 0.025:
        move(
            [plate_pos[0] - offset[0], plate_pos[1] - offset[1], eef[2]],
            grasp_yaw, "close", 20, 0.012
        )

    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    eef = state["eef_pos"]
    if hypot(bowl_pos[0] - plate_pos[0], bowl_pos[1] - plate_pos[1]) > 0.04:
        return {"status": "failed", "reason": "Bowl not aligned over plate"}
    if remaining() < 35:
        return {"status": "failed", "reason": "Insufficient placement budget"}

    offset = [bowl_pos[i] - eef[i] for i in range(3)]
    release = [
        plate_pos[0] - offset[0],
        plate_pos[1] - offset[1],
        plate_pos[2] + 0.025 - offset[2]
    ]
    move(release, grasp_yaw, "close", min(32, remaining() - 22), 0.012)

    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    if (
        hypot(bowl_pos[0] - plate_pos[0], bowl_pos[1] - plate_pos[1]) > 0.04 or
        bowl_pos[2] - plate_pos[2] > 0.065
    ):
        return {"status": "failed", "reason": "Release pose not reached"}

    actuate("open", 10)
    eef = list(sdk.get_state()["eef_pos"])
    move(
        [eef[0], eef[1], eef[2] + 0.10],
        grasp_yaw, "open", min(18, max(0, remaining() - 5)), 0.02
    )
    actuate("open", 5)

    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    horizontal_error = hypot(
        bowl_pos[0] - plate_pos[0], bowl_pos[1] - plate_pos[1]
    )
    height_difference = bowl_pos[2] - plate_pos[2]
    placed = horizontal_error < 0.045 and -0.015 < height_difference < 0.07
    return {
        "status": "done" if placed else "failed",
        "target": target,
        "horizontal_error_m": horizontal_error,
        "height_above_plate_m": height_difference,
        "attempts": len(used_angles)
    }
