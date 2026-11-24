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


def actuate(sdk, action, steps=10):
    budget = remaining_steps(sdk)
    if budget <= 0:
        return False
    sdk.set_gripper(action, steps=min(steps, budget))
    return True


def angle_difference(a, b):
    return abs(atan2(sin(a - b), cos(a - b)))


def free_space_angle(state, target_name, attempted):
    bowl = state["objects"][target_name]["pos"]
    neighbors = []
    for name in state["objects"]:
        if name != target_name:
            neighbors.append(state["objects"][name]["pos"])
    for name in state["fixtures"]:
        neighbors.append(state["fixtures"][name]["pos"])

    best_angle = None
    best_clearance = -1.0
    for index in range(144):
        angle = 2.0 * pi * index / 144.0
        if any(angle_difference(angle, previous) < pi / 3.0
               for previous in attempted):
            continue
        x = bowl[0] + 0.04 * cos(angle)
        y = bowl[1] + 0.04 * sin(angle)
        clearance = min(
            [hypot(x - point[0], y - point[1]) for point in neighbors]
            + [1000.0]
        )
        if clearance > best_clearance:
            best_clearance = clearance
            best_angle = angle
    return best_angle


def attachment_consistent(state, name, offset):
    bowl = state["objects"][name]["pos"]
    eef = state["eef_pos"]
    current = [bowl[i] - eef[i] for i in range(3)]
    return (distance_xyz(bowl, eef) <= 0.12
            and distance_xyz(current, offset) <= 0.035)


def run_policy(sdk):
    state = sdk.get_state()
    bowls = [name for name in state["objects"] if "black_bowl" in name]
    plates = [name for name in state["objects"] if "plate" in name]
    if not bowls or not plates:
        return {"status": "failed", "reason": "Required objects unavailable"}

    stove_points = []
    for name in state["sites"]:
        if "stove" in name and ("burner" in name or "cook_region" in name):
            stove_points.append(state["sites"][name])
    if not stove_points:
        for name in state["fixtures"]:
            if "stove" in name:
                stove_points.append(state["fixtures"][name]["pos"])
    if not stove_points:
        return {"status": "failed", "reason": "Stove pose unavailable"}

    ranked_bowls = []
    for name in bowls:
        pos = state["objects"][name]["pos"]
        score = min(distance_xyz(pos, point) for point in stove_points)
        ranked_bowls.append((score, name))
    bowl_name = sorted(ranked_bowls)[0][1]
    plate_name = sorted(plates)[0]

    attempted = []
    held = False
    yaw = state["eef_yaw"]
    offset = [0.0, 0.0, 0.0]

    for attempt in range(3):
        if remaining_steps(sdk) < 205:
            break
        state = sdk.get_state()
        angle = free_space_angle(state, bowl_name, attempted)
        if angle is None:
            break
        attempted.append(angle)

        # Probe-grounded radial contact and perpendicular jaw alignment.
        yaw = angle + pi / 2.0
        while yaw > pi / 2.0:
            yaw = yaw - pi
        while yaw < -pi / 2.0:
            yaw = yaw + pi

        actuate(sdk, "open", 8)
        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        eef = list(state["eef_pos"])
        approach_z = max(eef[2], bowl[2] + 0.14)
        move(sdk, [eef[0], eef[1], approach_z], yaw, "open", 22)
        move(
            sdk,
            [bowl[0] + 0.04 * cos(angle),
             bowl[1] + 0.04 * sin(angle), approach_z],
            yaw, "open", 32
        )

        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        contact = [
            bowl[0] + 0.04 * cos(angle),
            bowl[1] + 0.04 * sin(angle),
            bowl[2] + 0.02
        ]
        move(sdk, contact, yaw, "open", 32, 0.008)
        state = sdk.get_state()

        # A contact-limited move need not report reached to permit closing.
        if distance_xyz(state["eef_pos"], contact) > 0.04:
            move(sdk, contact, yaw, "open", 12, 0.012)
            state = sdk.get_state()
        if distance_xyz(state["eef_pos"], contact) > 0.055:
            continue

        before_z = state["objects"][bowl_name]["pos"][2]
        actuate(sdk, "close", 12)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        move(
            sdk, [eef[0], eef[1], eef[2] + 0.11],
            yaw, "close", 30, 0.012
        )
        state = sdk.get_state()
        bowl = state["objects"][bowl_name]["pos"]
        eef = state["eef_pos"]
        lift = bowl[2] - before_z

        # Width and grasp hints are deliberately excluded from verification.
        if lift >= 0.03 and distance_xyz(bowl, eef) <= 0.12:
            held = True
            offset = [bowl[i] - eef[i] for i in range(3)]
            break

        # Lower a partial pickup before opening for another radial approach.
        if lift > 0.012 and distance_xyz(bowl, eef) <= 0.12:
            move(
                sdk,
                [eef[0], eef[1], eef[2] - lift],
                yaw, "close", 16
            )
        actuate(sdk, "open", 6)

    if not held:
        return {"status": "failed", "reason": "Could not verify bowl pickup"}

    state = sdk.get_state()
    eef = list(state["eef_pos"])
    plate = state["objects"][plate_name]["pos"]

    obstacle_top = plate[2]
    for name in state["objects"]:
        if name != bowl_name:
            obstacle_top = max(obstacle_top, state["objects"][name]["pos"][2])
    for name in state["fixtures"]:
        obstacle_top = max(obstacle_top, state["fixtures"][name]["pos"][2])
    for site in state["sites"]:
        if any(name in site for name in state["fixtures"]):
            obstacle_top = max(obstacle_top, state["sites"][site][2])

    travel_z = max(eef[2], obstacle_top + 0.10 - offset[2])
    move(sdk, [eef[0], eef[1], travel_z], yaw, "close", 30)
    state = sdk.get_state()
    if not attachment_consistent(state, bowl_name, offset):
        return {"status": "failed", "reason": "Bowl slipped during ascent"}
    if state["objects"][bowl_name]["pos"][2] < obstacle_top + 0.065:
        return {"status": "failed", "reason": "Insufficient transport clearance"}

    plate = state["objects"][plate_name]["pos"]
    destination = [plate[0] - offset[0], plate[1] - offset[1], travel_z]
    move(sdk, destination, yaw, "close", 48)
    state = sdk.get_state()
    if not attachment_consistent(state, bowl_name, offset):
        return {"status": "failed", "reason": "Bowl slipped during transport"}

    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if hypot(bowl[0] - plate[0], bowl[1] - plate[1]) > 0.025:
        eef = state["eef_pos"]
        move(
            sdk,
            [eef[0] + plate[0] - bowl[0],
             eef[1] + plate[1] - bowl[1], eef[2]],
            yaw, "close", 18
        )

    state = sdk.get_state()
    if not attachment_consistent(state, bowl_name, offset):
        return {"status": "failed", "reason": "Bowl attachment lost"}
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if hypot(bowl[0] - plate[0], bowl[1] - plate[1]) > 0.035:
        return {"status": "failed", "reason": "Could not align bowl over plate"}

    eef = state["eef_pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    target = [
        plate[0] - offset[0],
        plate[1] - offset[1],
        plate[2] + 0.028 - offset[2]
    ]
    move(sdk, target, yaw, "close", 36, 0.008)
    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if (hypot(bowl[0] - plate[0], bowl[1] - plate[1]) > 0.04
            or bowl[2] - plate[2] > 0.065):
        return {"status": "failed", "reason": "Could not reach release position"}

    actuate(sdk, "open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(sdk, [eef[0], eef[1], eef[2] + 0.12], yaw, "open", 24)
    actuate(sdk, "open", 10)

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    centered = hypot(bowl[0] - plate[0], bowl[1] - plate[1]) < 0.045
    supported = -0.01 < bowl[2] - plate[2] < 0.065
    released = distance_xyz(bowl, state["eef_pos"]) > 0.09
    if centered and supported and released:
        return {"status": "done", "object": bowl_name}
    return {"status": "failed", "reason": "Final placement could not be verified"}
