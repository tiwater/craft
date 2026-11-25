def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, state["max_steps"] - state["steps_used"])

    def move(position, yaw, action, steps, tolerance=0.018):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.move_to(
                position,
                yaw=yaw,
                gripper=action,
                max_steps=budget,
                tol=tolerance,
            )

    def grip(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def distance(a, b):
        return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    def basket_position(state, name):
        if name in state["objects"]:
            return list(state["objects"][name]["pos"])
        return list(state["fixtures"][name]["pos"])

    def destination(state, name):
        for site in state["sites"]:
            if name in site and "contain" in site:
                return list(state["sites"][site])
        pos = basket_position(state, name)
        return [pos[0], pos[1], pos[2] + 0.075]

    def free_angle(state, target, tried):
        center = state["objects"][target]["pos"]
        neighbors = []
        for group in ("objects", "fixtures"):
            for name in state[group]:
                if name != target:
                    neighbors.append(state[group][name]["pos"])

        best_angle = 0.0
        best_score = -1.0
        found = False
        for index in range(64):
            angle = 2.0 * pi * index / 64.0
            distinct = True
            for previous in tried:
                # Parallel jaws have the same alignment after a half turn.
                difference = abs(atan2(
                    sin(angle - previous), cos(angle - previous)
                ))
                difference = min(difference, abs(pi - difference))
                if difference < pi / 6.0:
                    distinct = False
            if not distinct:
                continue

            ux = cos(angle)
            uy = sin(angle)
            clearance = 1000.0
            for neighbor in neighbors:
                dx = neighbor[0] - center[0]
                dy = neighbor[1] - center[1]
                # Score the open-jaw axis and the outward approach corridor.
                # These are clearance envelopes, not grasp offsets.
                projection = max(-0.04, min(0.10, dx * ux + dy * uy))
                gap = hypot(dx - projection * ux, dy - projection * uy)
                clearance = min(clearance, gap)
            if not found or clearance > best_score:
                best_angle = angle
                best_score = clearance
                found = True
        return best_angle

    state = sdk.get_state()
    sauce = None
    basket = None
    for name in state["objects"]:
        if "bbq" in name and "sauce" in name:
            sauce = name
    for group in ("objects", "fixtures"):
        for name in state[group]:
            if "basket" in name:
                basket = name
    if sauce is None or basket is None:
        return {"status": "failed", "reason": "Required scene entities unavailable"}

    tried = []
    held = False
    baseline_z = 0.0
    grasp_yaw = 0.0

    for attempt in range(3):
        if remaining() < 205:
            break

        state = sdk.get_state()
        angle = free_angle(state, sauce, tried)
        tried.append(angle)

        # Successful probe: radius=0, z_offset=0, jaw_rel=0.
        # Its world-angle-zero yaw was pi; preserve that alignment.
        grasp_yaw = angle + pi
        while grasp_yaw - state["eef_yaw"] > pi:
            grasp_yaw -= 2.0 * pi
        while grasp_yaw - state["eef_yaw"] < -pi:
            grasp_yaw += 2.0 * pi

        pos = list(state["objects"][sauce]["pos"])
        hover_z = max(pos[2] + 0.16, state["eef_pos"][2])
        grip("open", 6)
        move(
            [state["eef_pos"][0], state["eef_pos"][1], hover_z],
            grasp_yaw, "open", 22,
        )
        state = sdk.get_state()
        pos = list(state["objects"][sauce]["pos"])
        move([pos[0], pos[1], hover_z], grasp_yaw, "open", 32)

        state = sdk.get_state()
        pos = list(state["objects"][sauce]["pos"])
        move(pos, grasp_yaw, "open", 40, 0.012)

        state = sdk.get_state()
        pos = list(state["objects"][sauce]["pos"])
        if distance(state["eef_pos"], pos) > 0.035:
            move(pos, grasp_yaw, "open", 14, 0.015)
            state = sdk.get_state()
            pos = list(state["objects"][sauce]["pos"])

        # A small controller residual must not prevent gripper actuation.
        if distance(state["eef_pos"], pos) > 0.045:
            continue

        baseline_z = pos[2]
        grip("close", 14)
        state = sdk.get_state()
        move(
            [
                state["eef_pos"][0],
                state["eef_pos"][1],
                state["eef_pos"][2] + 0.11,
            ],
            grasp_yaw, "close", 30, 0.018,
        )
        state = sdk.get_state()
        pos = state["objects"][sauce]["pos"]
        held = (
            pos[2] - baseline_z >= 0.03
            and distance(state["eef_pos"], pos) <= 0.13
        )
        if held:
            break

    if not held:
        return {"status": "failed", "reason": "No verified object lift"}

    state = sdk.get_state()
    pos = state["objects"][sauce]["pos"]
    offset = [state["eef_pos"][i] - pos[i] for i in range(3)]
    target = destination(state, basket)

    obstacle_z = target[2]
    for group in ("objects", "fixtures"):
        for name in state[group]:
            if name != sauce:
                obstacle_z = max(obstacle_z, state[group][name]["pos"][2])
    transit_z = max(
        state["eef_pos"][2],
        obstacle_z + 0.17 + max(0.0, offset[2]),
        target[2] + 0.17 + max(0.0, offset[2]),
    )
    move(
        [state["eef_pos"][0], state["eef_pos"][1], transit_z],
        grasp_yaw, "close", 25,
    )

    state = sdk.get_state()
    if state["objects"][sauce]["pos"][2] < obstacle_z + 0.10:
        return {"status": "failed", "reason": "Insufficient transfer clearance"}

    # Correct using live object position, tolerating move_to residuals.
    for correction in range(3):
        state = sdk.get_state()
        pos = state["objects"][sauce]["pos"]
        if (
            pos[2] - baseline_z < 0.03
            or distance(state["eef_pos"], pos) > 0.13
        ):
            return {"status": "failed", "reason": "Object lost during transfer"}
        target = destination(state, basket)
        error_x = target[0] - pos[0]
        error_y = target[1] - pos[1]
        if hypot(error_x, error_y) <= 0.022:
            break
        if remaining() < 65:
            break
        move(
            [
                state["eef_pos"][0] + error_x,
                state["eef_pos"][1] + error_y,
                transit_z,
            ],
            grasp_yaw, "close", min(38, remaining() - 65), 0.015,
        )

    state = sdk.get_state()
    pos = state["objects"][sauce]["pos"]
    target = destination(state, basket)
    if hypot(pos[0] - target[0], pos[1] - target[1]) > 0.045:
        return {"status": "failed", "reason": "Object not above basket"}

    if remaining() < 40:
        return {"status": "failed", "reason": "Insufficient release budget"}

    offset = [state["eef_pos"][i] - pos[i] for i in range(3)]
    move(
        [
            target[0] + offset[0],
            target[1] + offset[1],
            target[2] + 0.035 + offset[2],
        ],
        grasp_yaw, "close", min(32, remaining() - 28), 0.015,
    )

    state = sdk.get_state()
    pos = state["objects"][sauce]["pos"]
    target = destination(state, basket)
    if (
        hypot(pos[0] - target[0], pos[1] - target[1]) > 0.045
        or pos[2] > target[2] + 0.09
        or distance(state["eef_pos"], pos) > 0.13
    ):
        return {"status": "failed", "reason": "Release pose not verified"}

    grip("open", 10)
    state = sdk.get_state()
    move(
        [
            state["eef_pos"][0],
            state["eef_pos"][1],
            state["eef_pos"][2] + 0.12,
        ],
        grasp_yaw, "open", min(20, max(0, remaining() - 12)),
    )
    budget = min(20, remaining())
    if budget > 0:
        sdk.step_raw([0, 0, 0, 0, 0, 0, -1], steps=budget)

    state = sdk.get_state()
    pos = state["objects"][sauce]["pos"]
    target = destination(state, basket)
    base = basket_position(state, basket)
    centered = hypot(pos[0] - target[0], pos[1] - target[1]) < 0.075
    settled = base[2] + 0.025 < pos[2] < target[2] + 0.075
    released = distance(state["eef_pos"], pos) > 0.075
    if centered and settled and released:
        return {"status": "done"}
    return {"status": "failed", "reason": "Final placement not verified"}
