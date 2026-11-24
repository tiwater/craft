def run_policy(sdk):
    def move(pos, yaw, grip, limit=40, tol=0.012):
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining <= 0:
            return False
        sdk.move_to(
            pos, yaw=yaw, gripper=grip,
            max_steps=min(limit, remaining), tol=tol
        )
        state = sdk.get_state()
        error = sqrt(sum(
            (state["eef_pos"][i] - pos[i]) ** 2 for i in range(3)
        ))
        return error < max(tol * 1.8, 0.018)

    def grip(action, steps):
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining > 0:
            sdk.set_gripper(action, steps=min(steps, remaining))

    state = sdk.get_state()
    objects = state["objects"]
    bowls = [name for name in objects if "black_bowl" in name]
    ramekins = [name for name in objects if "ramekin" in name]
    plates = [name for name in objects if "plate" in name]
    if not bowls or not ramekins or not plates:
        return {"status": "failed", "reason": "Required objects unavailable"}

    ramekin = objects[ramekins[0]]["pos"]
    region = state["sites"].get("main_table_next_to_ramekin_region")

    def target_score(name):
        pos = objects[name]["pos"]
        score = hypot(pos[0] - ramekin[0], pos[1] - ramekin[1])
        if region is not None:
            score += hypot(pos[0] - region[0], pos[1] - region[1])
        return score

    def clearance_score(item):
        return item[0]

    # The previous rollouts never executed: lambda expressions are forbidden.
    # Named functions preserve the policy without that sandbox violation.
    target = min(bowls, key=target_score)
    plate = plates[0]
    initial_z = objects[target]["pos"][2]
    initial_plate_z = objects[plate]["pos"][2]
    held = False
    yaw = state["eef_yaw"]

    for attempt in range(3):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 160:
            break
        bowl = list(state["objects"][target]["pos"])
        baseline = bowl[2]

        candidates = [
            [0.0, 1.0], [0.0, -1.0],
            [1.0, 0.0], [-1.0, 0.0]
        ]
        ranked = []
        for direction in candidates:
            point = [
                bowl[0] + 0.045 * direction[0],
                bowl[1] + 0.045 * direction[1]
            ]
            clearance = 1.0
            for name in state["objects"]:
                if name != target:
                    other = state["objects"][name]["pos"]
                    clearance = min(
                        clearance,
                        hypot(point[0] - other[0], point[1] - other[1])
                    )
            ranked.append([clearance, direction])
        ranked = sorted(ranked, key=clearance_score, reverse=True)
        direction = ranked[min(attempt, len(ranked) - 1)][1]

        yaw = atan2(direction[1], direction[0]) - pi / 2.0
        radius = 0.043 if attempt != 2 else 0.052
        height = 0.012 if attempt == 0 else 0.002
        safe_z = max(
            state["eef_pos"][2],
            bowl[2] + 0.14,
            state["objects"][plate]["pos"][2] + 0.14
        )
        eef = state["eef_pos"]
        move([eef[0], eef[1], safe_z], yaw, "open", 25)
        grip("open", 5)

        grasp_xy = [
            bowl[0] + radius * direction[0],
            bowl[1] + radius * direction[1]
        ]
        if not move(
            [grasp_xy[0], grasp_xy[1], safe_z], yaw, "open", 35
        ):
            continue

        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        baseline = bowl[2]
        grasp = [
            bowl[0] + radius * direction[0],
            bowl[1] + radius * direction[1],
            bowl[2] + height
        ]
        if not move(grasp, yaw, "open", 35, 0.008):
            continue

        grip("close", 16)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        move(
            [eef[0], eef[1], eef[2] + 0.065],
            yaw, "close", 25, 0.01
        )
        state = sdk.get_state()
        lifted = state["objects"][target]["pos"]
        if lifted[2] > baseline + 0.035:
            held = True
            break
        grip("open", 6)

    if not held:
        return {"status": "failed", "reason": "Rim grasp did not lift bowl"}

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    plate_pos = state["objects"][plate]["pos"]
    safe_z = max(
        eef[2],
        initial_z + 0.15 - offset[2],
        plate_pos[2] + 0.15 - offset[2]
    )
    if not move([eef[0], eef[1], safe_z], yaw, "close", 30):
        return {"status": "failed", "reason": "Lift clearance unavailable"}

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    plate_pos = state["objects"][plate]["pos"]
    destination = [
        plate_pos[0] - offset[0],
        plate_pos[1] - offset[1],
        safe_z
    ]
    if not move(destination, yaw, "close", 55, 0.009):
        return {"status": "failed", "reason": "Transfer incomplete"}

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    if bowl[2] < initial_z + 0.035:
        return {"status": "failed", "reason": "Bowl slipped"}

    eef = state["eef_pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    plate_pos = state["objects"][plate]["pos"]
    resting_z = initial_z + plate_pos[2] - initial_plate_z + 0.025
    release = [
        plate_pos[0] - offset[0],
        plate_pos[1] - offset[1],
        resting_z - offset[2]
    ]
    move(release, yaw, "close", 40, 0.008)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    if (
        hypot(bowl[0] - plate_pos[0], bowl[1] - plate_pos[1]) > 0.035
        or bowl[2] > resting_z + 0.04
    ):
        return {"status": "failed", "reason": "Release pose not reached"}

    grip("open", 14)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.12], yaw, "open", 30)
    grip("open", 10)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    expected_z = initial_z + plate_pos[2] - initial_plate_z
    placed = (
        hypot(bowl[0] - plate_pos[0], bowl[1] - plate_pos[1]) < 0.045
        and expected_z - 0.015 <= bowl[2] <= expected_z + 0.065
        and state["gripper_width"] > 0.04
    )
    return {
        "status": "done" if placed else "failed",
        "object": target,
        "reason": "Placement verified" if placed else "Placement not verified"
    }
