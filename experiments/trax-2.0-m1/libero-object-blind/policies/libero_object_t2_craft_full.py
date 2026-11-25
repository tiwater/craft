def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, state["max_steps"] - state["steps_used"])


def move(sdk, position, yaw, grip, steps, tolerance=0.012):
    available = remaining_steps(sdk)
    if available > 0:
        sdk.move_to(
            position,
            yaw=yaw,
            gripper=grip,
            max_steps=min(steps, available),
            tol=tolerance,
        )


def grip(sdk, action, steps):
    available = remaining_steps(sdk)
    if available > 0:
        sdk.set_gripper(action, steps=min(steps, available))


def distance(a, b):
    return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def destination(state, basket):
    for name in state["sites"]:
        if basket in name and "contain" in name:
            return list(state["sites"][name])
    for group in ("objects", "fixtures"):
        if basket in state[group]:
            pos = state[group][basket]["pos"]
            return [pos[0], pos[1], pos[2] + 0.075]
    return None


def free_angle(state, target, tried):
    center = state["objects"][target]["pos"]
    neighbors = []
    for group in ("objects", "fixtures"):
        for name in state[group]:
            if name != target:
                neighbors.append(state[group][name]["pos"])

    best_angle = None
    best_clearance = -1.0
    for index in range(72):
        angle = index * pi / 36.0
        distinct = True
        for previous in tried:
            separation = abs(atan2(sin(angle - previous), cos(angle - previous)))
            separation = min(separation, abs(pi - separation))
            if separation < pi / 6.0:
                distinct = False
        if not distinct:
            continue

        clearance = 1000.0
        for neighbor in neighbors:
            dx = neighbor[0] - center[0]
            dy = neighbor[1] - center[1]
            along = dx * cos(angle) + dy * sin(angle)
            across = -dx * sin(angle) + dy * cos(angle)
            closest = max(-0.039, min(0.039, along))
            clearance = min(clearance, hypot(along - closest, across))
        if clearance > best_clearance:
            best_clearance = clearance
            best_angle = angle
    return best_angle


def radial_fallback_angle(state, target, tried):
    center = state["objects"][target]["pos"]
    best_angle = None
    best_clearance = -1.0
    for index in range(72):
        angle = index * pi / 36.0
        distinct = True
        for previous in tried:
            separation = abs(atan2(sin(angle - previous), cos(angle - previous)))
            separation = min(separation, abs(pi - separation))
            if separation < pi / 6.0:
                distinct = False
        if not distinct:
            continue

        clearance = 1000.0
        for group in ("objects", "fixtures"):
            for name in state[group]:
                if name != target:
                    neighbor = state[group][name]["pos"]
                    dx = neighbor[0] - center[0]
                    dy = neighbor[1] - center[1]
                    along = dx * cos(angle) + dy * sin(angle)
                    across = -dx * sin(angle) + dy * cos(angle)
                    closest = max(-0.039, min(0.079, along))
                    clearance = min(clearance, hypot(along - closest, across))
        if clearance > best_clearance:
            best_clearance = clearance
            best_angle = angle
    return best_angle


def holding(state, target, baseline_z):
    pos = state["objects"][target]["pos"]
    return (
        pos[2] - baseline_z >= 0.03
        and distance(pos, state["eef_pos"]) <= 0.12
    )


def run_policy(sdk):
    state = sdk.get_state()
    target = None
    basket = None
    for name in state["objects"]:
        if "salad_dressing" in name:
            target = name
    for group in ("objects", "fixtures"):
        for name in state[group]:
            if "basket" in name:
                basket = name
    if target is None or basket is None:
        return {"status": "failed", "reason": "Required objects are missing"}

    tried = []
    verified = False
    compact = False
    lift_failed = False
    baseline_z = state["objects"][target]["pos"][2]
    grasp_yaw = 0.0

    # Preserve the complete baseline sequence, including successful retries.
    for attempt in range(3):
        if remaining_steps(sdk) < 195:
            break

        state = sdk.get_state()
        angle = free_angle(state, target, tried)
        if angle is None:
            break
        tried.append(angle)
        grasp_yaw = atan2(sin(angle + 3.143), cos(angle + 3.143))
        pos = list(state["objects"][target]["pos"])
        dest = destination(state, basket)
        safe_z = max(pos[2] + 0.18, dest[2] + 0.16, state["eef_pos"][2])

        grip(sdk, "open", 6)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move(sdk, [eef[0], eef[1], safe_z], grasp_yaw, "open", 24)
        state = sdk.get_state()
        pos = list(state["objects"][target]["pos"])
        move(sdk, [pos[0], pos[1], safe_z], grasp_yaw, "open", 30)

        state = sdk.get_state()
        pos = list(state["objects"][target]["pos"])
        baseline_z = pos[2]
        move(sdk, pos, grasp_yaw, "open", 34, 0.008)

        state = sdk.get_state()
        pos = list(state["objects"][target]["pos"])
        eef = state["eef_pos"]
        if hypot(pos[0] - eef[0], pos[1] - eef[1]) > 0.03:
            move(sdk, pos, grasp_yaw, "open", 10, 0.01)
            state = sdk.get_state()
            pos = list(state["objects"][target]["pos"])
            eef = state["eef_pos"]

        if (
            hypot(pos[0] - eef[0], pos[1] - eef[1]) <= 0.035
            and abs(pos[2] - eef[2]) <= 0.065
        ):
            grip(sdk, "close", 12)
            state = sdk.get_state()
            eef = state["eef_pos"]
            move(
                sdk,
                [eef[0], eef[1], eef[2] + 0.10],
                grasp_yaw,
                "close",
                25,
            )
            state = sdk.get_state()
            if holding(state, target, baseline_z):
                verified = True
                break
            lift_failed = True

    # Only failed baseline executions enter this bounded extra attempt.
    if not verified and lift_failed and remaining_steps(sdk) >= 92:
        state = sdk.get_state()
        angle = radial_fallback_angle(state, target, tried)
        if angle is not None:
            compact = True
            grasp_yaw = atan2(sin(angle + 3.143), cos(angle + 3.143))
            grip(sdk, "open", 4)
            state = sdk.get_state()
            pos = list(state["objects"][target]["pos"])
            eef = state["eef_pos"]
            stage_z = max(eef[2], pos[2] + 0.10)

            # The radial offset is an approach waypoint, not a grasp offset.
            move(
                sdk,
                [
                    pos[0] + 0.04 * cos(angle),
                    pos[1] + 0.04 * sin(angle),
                    stage_z,
                ],
                grasp_yaw,
                "open",
                8,
            )
            state = sdk.get_state()
            pos = list(state["objects"][target]["pos"])
            baseline_z = pos[2]
            if state["gripper_width"] >= 0.06:
                move(sdk, pos, grasp_yaw, "open", 12, 0.008)
                state = sdk.get_state()
                pos = state["objects"][target]["pos"]
                eef = state["eef_pos"]
                if (
                    hypot(pos[0] - eef[0], pos[1] - eef[1]) <= 0.025
                    and abs(pos[2] - eef[2]) <= 0.04
                ):
                    grip(sdk, "close", 6)
                    state = sdk.get_state()
                    eef = state["eef_pos"]
                    move(
                        sdk,
                        [eef[0], eef[1], eef[2] + 0.075],
                        grasp_yaw,
                        "close",
                        8,
                        0.01,
                    )
                    state = sdk.get_state()
                    verified = holding(state, target, baseline_z)

    if not verified:
        return {"status": "failed", "reason": "No verified object lift"}

    state = sdk.get_state()
    pos = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    offset = [pos[i] - eef[i] for i in range(3)]
    dest = destination(state, basket)
    obstacle_z = dest[2]
    for group in ("objects", "fixtures"):
        for name in state[group]:
            if name != target:
                obstacle_z = max(obstacle_z, state[group][name]["pos"][2])
    travel_z = max(eef[2], obstacle_z + 0.18 - offset[2])

    move(
        sdk, [eef[0], eef[1], travel_z], grasp_yaw, "close",
        8 if compact else 25,
    )
    state = sdk.get_state()
    if not holding(state, target, baseline_z):
        return {"status": "failed", "reason": "Lift lost before transport"}
    if compact and state["eef_pos"][2] < travel_z - 0.025:
        return {"status": "failed", "reason": "Transport clearance not reached"}

    pos = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    offset = [pos[i] - eef[i] for i in range(3)]
    dest = destination(state, basket)
    move(
        sdk,
        [dest[0] - offset[0], dest[1] - offset[1], travel_z],
        grasp_yaw,
        "close",
        16 if compact else 45,
    )

    state = sdk.get_state()
    if not holding(state, target, baseline_z):
        return {"status": "failed", "reason": "Object lost during transport"}
    pos = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    offset = [pos[i] - eef[i] for i in range(3)]
    dest = destination(state, basket)
    if hypot(pos[0] - dest[0], pos[1] - dest[1]) > 0.04:
        correction_steps = 14
        if compact:
            correction_steps = min(14, max(0, remaining_steps(sdk) - 30))
        move(
            sdk,
            [dest[0] - offset[0], dest[1] - offset[1], eef[2]],
            grasp_yaw,
            "close",
            correction_steps,
        )
        if compact:
            state = sdk.get_state()
            pos = state["objects"][target]["pos"]
            dest = destination(state, basket)
            if hypot(pos[0] - dest[0], pos[1] - dest[1]) > 0.04:
                return {"status": "failed", "reason": "Basket approach not reached"}

    state = sdk.get_state()
    pos = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    offset = [pos[i] - eef[i] for i in range(3)]
    dest = destination(state, basket)
    move(
        sdk,
        [
            dest[0] - offset[0],
            dest[1] - offset[1],
            dest[2] + 0.045 - offset[2],
        ],
        grasp_yaw,
        "close",
        12 if compact else 30,
        0.012,
    )

    state = sdk.get_state()
    pos = state["objects"][target]["pos"]
    dest = destination(state, basket)
    if (
        hypot(pos[0] - dest[0], pos[1] - dest[1]) > 0.055
        or abs(pos[2] - (dest[2] + 0.045)) > 0.05
        or distance(pos, state["eef_pos"]) > 0.12
    ):
        return {"status": "failed", "reason": "Release position not verified"}

    grip(sdk, "open", 6 if compact else 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        sdk, [eef[0], eef[1], eef[2] + 0.16], grasp_yaw, "open",
        8 if compact else 22,
    )
    grip(sdk, "open", 4 if compact else 10)

    state = sdk.get_state()
    pos = list(state["objects"][target]["pos"])
    dest = destination(state, basket)
    centered = hypot(pos[0] - dest[0], pos[1] - dest[1]) < 0.075
    inside_height = dest[2] - 0.045 <= pos[2] <= dest[2] + 0.13
    separated = state["eef_pos"][2] - pos[2] > 0.09

    if centered and inside_height and separated:
        return {"status": "done"}
    return {"status": "failed", "reason": "Released placement not verified"}
