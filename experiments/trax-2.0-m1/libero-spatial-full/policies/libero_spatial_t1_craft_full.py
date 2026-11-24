def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def move(position, yaw, grip, steps, tolerance=0.012):
        budget = min(steps, remaining())
        if budget <= 0:
            return False
        result = sdk.move_to(
            position, yaw=yaw, gripper=grip,
            max_steps=budget, tol=tolerance
        )
        return result["reached"]

    def actuate(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def planar(first, second):
        return hypot(first[0] - second[0], first[1] - second[1])

    def distance(first, second):
        return sqrt(sum((first[index] - second[index]) ** 2 for index in range(3)))

    def segment_distance(point, start, finish):
        dx = finish[0] - start[0]
        dy = finish[1] - start[1]
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

    state = sdk.get_state()
    if "next to the ramekin" not in state["task_language"].lower():
        return {"status": "failed", "reason": "Task relation does not match policy"}

    entities = dict(state["fixtures"])
    entities.update(state["objects"])
    bowls = [name for name in state["objects"] if "black_bowl" in name]
    ramekins = [name for name in entities if "ramekin" in name]
    plates = [name for name in entities if "plate" in name]
    if not bowls or not ramekins or not plates:
        return {"status": "failed", "reason": "Required scene entities missing"}

    sites = state["sites"]
    relation_region = sites.get("main_table_next_to_ramekin_region")
    ramekin_region = sites.get("main_table_ramekin_region")
    plate_region = sites.get("main_table_plate_region")

    ramekin = ramekins[0]
    if ramekin_region is not None:
        best = float("inf")
        for name in ramekins:
            score = planar(entities[name]["pos"], ramekin_region)
            if score < best:
                best = score
                ramekin = name

    anchor = entities[ramekin]["pos"]
    expected = relation_region
    if relation_region is not None and ramekin_region is not None:
        expected = [
            anchor[index] + relation_region[index] - ramekin_region[index]
            for index in range(3)
        ]

    target = bowls[0]
    best = float("inf")
    for name in bowls:
        position = entities[name]["pos"]
        if expected is not None:
            score = planar(position, expected) + 0.15 * planar(position, anchor)
        else:
            score = planar(position, anchor)
        if score < best:
            best = score
            target = name

    plate = plates[0]
    if plate_region is not None:
        best = float("inf")
        for name in plates:
            score = planar(entities[name]["pos"], plate_region)
            if score < best:
                best = score
                plate = name

    def destination(state):
        if plate in state["objects"]:
            return list(state["objects"][plate]["pos"])
        return list(state["fixtures"][plate]["pos"])

    def placed(state):
        bowl = state["objects"][target]["pos"]
        goal = destination(state)
        return (
            planar(bowl, goal) < 0.045
            and goal[2] - 0.02 <= bowl[2] <= goal[2] + 0.065
        )

    yaw = -pi / 2.0
    grasped = False

    # The successful probe offsets are in world coordinates.
    for attempt in range(2):
        if remaining() < 205:
            break
        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        support_z = bowl[2]
        radius = 0.04 if attempt == 0 else 0.045
        z_offset = 0.02 if attempt == 0 else 0.025
        hover_z = bowl[2] + 0.12
        eef = state["eef_pos"]

        move([eef[0], eef[1], max(eef[2], hover_z)],
             yaw, "open", 18)
        actuate("open", 5)
        move([bowl[0], bowl[1] + radius, hover_z],
             yaw, "open", 32)

        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        contact = [bowl[0], bowl[1] + radius, bowl[2] + z_offset]
        move(contact, yaw, "open", 28, 0.01)

        # Rim contact can prevent exact convergence; close without an abort.
        actuate("close", 16)
        state = sdk.get_state()
        before_lift = state["objects"][target]["pos"][2]
        eef = list(state["eef_pos"])
        move([eef[0], eef[1], eef[2] + 0.075],
             yaw, "close", 24)

        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        grasped = (
            bowl[2] - before_lift > 0.025
            and distance(bowl, state["eef_pos"]) < 0.14
        )
        if grasped:
            break

        eef = state["eef_pos"]
        move([eef[0], eef[1], support_z + z_offset],
             yaw, "close", 16)
        actuate("open", 6)

    if not grasped:
        return {"status": "failed", "target": target, "reason": "No sustained lift"}

    state = sdk.get_state()
    bowl = list(state["objects"][target]["pos"])
    eef = list(state["eef_pos"])
    goal = destination(state)
    offset = [bowl[index] - eef[index] for index in range(3)]

    # Only obstacles near the actual transfer corridor determine clearance.
    bowl_travel_z = max(bowl[2], goal[2] + 0.085)
    for name in state["objects"]:
        if name != target and name != plate:
            position = state["objects"][name]["pos"]
            if segment_distance(position, bowl, goal) < 0.085:
                bowl_travel_z = max(bowl_travel_z, position[2] + 0.085)
    for name in state["fixtures"]:
        position = state["fixtures"][name]["pos"]
        if segment_distance(position, bowl, goal) < 0.12:
            bowl_travel_z = max(bowl_travel_z, position[2] + 0.10)
        for site_name in state["sites"]:
            if site_name.startswith(name):
                position = state["sites"][site_name]
                if segment_distance(position, bowl, goal) < 0.10:
                    bowl_travel_z = max(bowl_travel_z, position[2] + 0.08)

    travel_z = bowl_travel_z - offset[2]
    if travel_z > eef[2] + 0.015:
        move([eef[0], eef[1], travel_z], yaw, "close", 16)

    # Divide the lateral transfer to limit abrupt motion and refresh the offset.
    for stage in range(2):
        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        eef = list(state["eef_pos"])
        goal = destination(state)
        if distance(bowl, eef) > 0.16:
            if placed(state):
                break
            return {"status": "failed", "target": target, "reason": "Grasp lost"}

        fraction = 0.5 if stage == 0 else 1.0
        move([
            eef[0] + fraction * (goal[0] - bowl[0]),
            eef[1] + fraction * (goal[1] - bowl[1]),
            travel_z,
        ], yaw, "close", 25, 0.01)

    state = sdk.get_state()
    if not placed(state) or distance(
        state["objects"][target]["pos"], state["eef_pos"]
    ) < 0.14:
        for correction in range(2):
            state = sdk.get_state()
            bowl = state["objects"][target]["pos"]
            eef = state["eef_pos"]
            goal = destination(state)
            if planar(bowl, goal) <= 0.012:
                break
            move([
                eef[0] + goal[0] - bowl[0],
                eef[1] + goal[1] - bowl[1],
                eef[2],
            ], yaw, "close", 14, 0.008)

        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        eef = state["eef_pos"]
        goal = destination(state)
        if planar(bowl, goal) > 0.05:
            return {"status": "failed", "target": target, "reason": "Transfer misaligned"}

        move([
            eef[0] + goal[0] - bowl[0],
            eef[1] + goal[1] - bowl[1],
            eef[2] + goal[2] + 0.012 - bowl[2],
        ], yaw, "close", min(28, max(0, remaining() - 32)), 0.008)

    actuate("open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.10],
         yaw, "open", min(16, max(0, remaining() - 4)))
    actuate("open", 4)

    state = sdk.get_state()
    success = placed(state) and distance(
        state["objects"][target]["pos"], state["eef_pos"]
    ) > 0.075
    return {
        "status": "done" if success else "failed",
        "target": target,
        "reason": "Placement verified" if success else "Placement unverified",
    }
