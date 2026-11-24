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


def actuate(sdk, action, steps):
    budget = remaining_steps(sdk)
    if budget <= 0:
        return False
    sdk.set_gripper(action, steps=min(steps, budget))
    return True


def angle_difference(a, b):
    return abs(atan2(sin(a - b), cos(a - b)))


def choose_angle(state, bowl, cabinet, radius, height, previous):
    center = state["objects"][bowl]["pos"]
    obstacles = []
    for name in state["objects"]:
        if name != bowl:
            obstacles.append(list(state["objects"][name]["pos"]))

    support_height = state["fixtures"][cabinet]["pos"][2]
    support_site = cabinet + "_top_region"
    if support_site in state["sites"]:
        support_height = state["sites"][support_site][2]

    for name in state["fixtures"]:
        if name != cabinet:
            obstacles.append(list(state["fixtures"][name]["pos"]))
            for site in state["sites"]:
                if site.startswith(name + "_"):
                    obstacles.append(list(state["sites"][site]))

    best_angle = 0.0
    best_score = None
    for index in range(72):
        angle = 2.0 * pi * index / 72.0
        if any(angle_difference(angle, used) < pi / 6.0 for used in previous):
            continue

        contact = [
            center[0] + radius * cos(angle),
            center[1] + radius * sin(angle),
            center[2] + height,
        ]
        clearance = 100.0
        total_clearance = 0.0
        for obstacle in obstacles:
            separation = hypot(
                contact[0] - obstacle[0],
                contact[1] - obstacle[1],
            )
            clearance = min(clearance, separation)
            total_clearance += separation

        # The supporting cabinet is a surface beneath the contact,
        # not a lateral obstacle at its base-frame origin.
        support_clearance = contact[2] - support_height
        score = (
            min(clearance, support_clearance),
            clearance,
            total_clearance,
        )
        if best_score is None or score > best_score:
            best_score = score
            best_angle = angle

    return best_angle


def travel_height(state, bowl):
    height = state["objects"][bowl]["pos"][2] + 0.14
    for name in state["objects"]:
        height = max(height, state["objects"][name]["pos"][2] + 0.12)
    for name in state["fixtures"]:
        height = max(height, state["fixtures"][name]["pos"][2] + 0.12)
        for site in state["sites"]:
            if site.startswith(name + "_"):
                height = max(height, state["sites"][site][2] + 0.12)
    return height


def lifted_near_hand(state, bowl, baseline):
    position = state["objects"][bowl]["pos"]
    return (
        position[2] - baseline >= 0.03
        and distance_xyz(position, state["eef_pos"]) <= 0.11
    )


def run_policy(sdk):
    state = sdk.get_state()
    bowls = [name for name in state["objects"] if "black_bowl" in name]
    plates = [name for name in state["objects"] if "plate" in name]
    cabinets = [
        name for name in state["fixtures"] if "wooden_cabinet" in name
    ]
    if not bowls or not plates or not cabinets:
        return {"status": "failed", "reason": "Required scene entities missing"}

    cabinet = cabinets[0]
    anchor = state["fixtures"][cabinet]["pos"]
    top_site = cabinet + "_top_region"
    if top_site in state["sites"]:
        anchor = state["sites"][top_site]

    bowl = bowls[0]
    nearest = float("inf")
    for name in bowls:
        separation = distance_xyz(state["objects"][name]["pos"], anchor)
        if separation < nearest:
            nearest = separation
            bowl = name

    plate = plates[0]
    initial = list(state["objects"][bowl]["pos"])
    bottom_offset = 0.035
    if top_site in state["sites"]:
        measured = initial[2] - state["sites"][top_site][2]
        if 0.015 <= measured <= 0.065:
            bottom_offset = measured

    previous_angles = []
    held = False
    baseline = initial[2]
    yaw = state["eef_yaw"]
    attempts = 0

    for attempt in range(3):
        if remaining_steps(sdk) < 190:
            break
        attempts += 1
        state = sdk.get_state()

        # Both contact geometries produced measured lifts in the active probe.
        radius = 0.04
        z_offset = 0.02
        if attempt > 0:
            radius = 0.045
            z_offset = 0.025

        angle = choose_angle(
            state, bowl, cabinet, radius, z_offset, previous_angles
        )
        previous_angles.append(angle)

        # Probe-grounded perpendicular jaw alignment in world coordinates.
        yaw = atan2(sin(angle + pi / 2.0), cos(angle + pi / 2.0))
        actuate(sdk, "open", 6)

        state = sdk.get_state()
        eef = state["eef_pos"]
        high = travel_height(state, bowl)
        move(sdk, [eef[0], eef[1], high], yaw, "open", 25)

        state = sdk.get_state()
        position = state["objects"][bowl]["pos"]
        contact = [
            position[0] + radius * cos(angle),
            position[1] + radius * sin(angle),
            position[2] + z_offset,
        ]
        move(sdk, [contact[0], contact[1], high], yaw, "open", 35)
        state = sdk.get_state()
        if hypot(
            state["eef_pos"][0] - contact[0],
            state["eef_pos"][1] - contact[1],
        ) > 0.035:
            continue

        position = state["objects"][bowl]["pos"]
        baseline = position[2]
        contact = [
            position[0] + radius * cos(angle),
            position[1] + radius * sin(angle),
            position[2] + z_offset,
        ]
        move(sdk, contact, yaw, "open", 30, 0.008)

        state = sdk.get_state()
        if distance_xyz(state["eef_pos"], contact) > 0.018:
            move(sdk, contact, yaw, "open", 10, 0.012)
            state = sdk.get_state()

        # A move_to convergence flag must not suppress contact actuation.
        if (
            hypot(
                state["eef_pos"][0] - contact[0],
                state["eef_pos"][1] - contact[1],
            ) > 0.035
            or abs(state["eef_pos"][2] - contact[2]) > 0.045
        ):
            continue

        actuate(sdk, "close", 12)
        closed = sdk.get_state()
        eef = closed["eef_pos"]
        move(
            sdk, [eef[0], eef[1], eef[2] + 0.085],
            yaw, "close", 25, 0.01
        )
        state = sdk.get_state()
        if lifted_near_hand(state, bowl, baseline):
            held = True
            break

        # Lower an unverified contact before opening for another angle.
        eef = state["eef_pos"]
        move(
            sdk, [eef[0], eef[1], contact[2]],
            yaw, "close", 12, 0.015
        )
        actuate(sdk, "open", 6)

    if not held:
        return {
            "status": "failed",
            "reason": "No live Z lift of at least 0.03 m near the EEF",
            "attempts": attempts,
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    position = state["objects"][bowl]["pos"]
    eef = state["eef_pos"]
    offset = [eef[i] - position[i] for i in range(3)]
    high = max(travel_height(state, bowl) + max(0.0, offset[2]), eef[2])

    move(sdk, [eef[0], eef[1], high], yaw, "close", 30, 0.012)
    state = sdk.get_state()
    if (
        not lifted_near_hand(state, bowl, baseline)
        or state["eef_pos"][2] < high - 0.025
    ):
        return {"status": "failed", "reason": "Could not clear cabinet with bowl"}

    target = state["objects"][plate]["pos"]
    transit = [target[0] + offset[0], target[1] + offset[1], high]
    move(sdk, transit, yaw, "close", 50, 0.012)

    state = sdk.get_state()
    position = state["objects"][bowl]["pos"]
    eef = state["eef_pos"]
    if (
        not lifted_near_hand(state, bowl, baseline)
        or distance_xyz([eef[i] - position[i] for i in range(3)], offset) > 0.04
    ):
        return {"status": "failed", "reason": "Bowl slipped during transport"}

    # Correct placement using the live hand-to-bowl displacement.
    offset = [eef[i] - position[i] for i in range(3)]
    target = state["objects"][plate]["pos"]
    aligned = [target[0] + offset[0], target[1] + offset[1], eef[2]]
    move(sdk, aligned, yaw, "close", 15, 0.009)

    state = sdk.get_state()
    position = state["objects"][bowl]["pos"]
    target = state["objects"][plate]["pos"]
    if hypot(position[0] - target[0], position[1] - target[1]) > 0.035:
        return {"status": "failed", "reason": "Bowl not aligned above plate"}

    placement = [
        target[0] + offset[0],
        target[1] + offset[1],
        target[2] + bottom_offset + 0.008 + offset[2],
    ]
    move(sdk, placement, yaw, "close", 45, 0.008)

    state = sdk.get_state()
    position = state["objects"][bowl]["pos"]
    target = state["objects"][plate]["pos"]
    if (
        hypot(position[0] - target[0], position[1] - target[1]) > 0.04
        or abs(position[2] - target[2] - bottom_offset - 0.008) > 0.035
    ):
        return {"status": "failed", "reason": "Placement pose not reached"}

    if not actuate(sdk, "open", 12):
        return {"status": "failed", "reason": "No budget to release bowl"}

    eef = sdk.get_state()["eef_pos"]
    move(sdk, [eef[0], eef[1], eef[2] + 0.12], yaw, "open", 20)
    actuate(sdk, "open", 8)

    state = sdk.get_state()
    position = state["objects"][bowl]["pos"]
    target = state["objects"][plate]["pos"]
    horizontal_error = hypot(position[0] - target[0], position[1] - target[1])
    height_difference = position[2] - target[2]
    placed = (
        horizontal_error < 0.055
        and 0.012 < height_difference < bottom_offset + 0.035
    )
    return {
        "status": "done" if placed else "failed",
        "object": bowl,
        "target": plate,
        "placement_verified_by_pose": placed,
        "horizontal_error_m": horizontal_error,
        "attempts": attempts,
        "steps_used": state["steps_used"],
    }
