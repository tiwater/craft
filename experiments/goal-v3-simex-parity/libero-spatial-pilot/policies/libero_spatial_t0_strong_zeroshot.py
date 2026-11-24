def policy_move(sdk, pos, yaw, gripper, limit=55, tol=0.008):
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


def policy_grip(sdk, action, steps=12):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining > 0:
        sdk.set_gripper(action, steps=min(steps, remaining))


def run_policy(sdk):
    state = sdk.get_state()
    objects = state["objects"]
    bowls = [name for name in objects if "black_bowl" in name]
    plates = [name for name in objects if "plate" in name]
    ramekins = [name for name in objects if "ramekin" in name]

    if not bowls or not plates or not ramekins:
        return {"status": "failed", "reason": "Required objects unavailable"}

    plate_name = plates[0]
    ramekin_name = ramekins[0]
    plate = objects[plate_name]["pos"]
    ramekin = objects[ramekin_name]["pos"]
    vx = ramekin[0] - plate[0]
    vy = ramekin[1] - plate[1]
    length2 = max(vx * vx + vy * vy, 0.000001)
    region = None
    for name in state["sites"]:
        if "between_plate_ramekin" in name:
            region = state["sites"][name]
            break

    target = bowls[0]
    best_score = 1000000.0
    for name in bowls:
        pos = objects[name]["pos"]
        t = ((pos[0] - plate[0]) * vx + (pos[1] - plate[1]) * vy) / length2
        projection = min(1.0, max(0.0, t))
        dx = pos[0] - plate[0] - projection * vx
        dy = pos[1] - plate[1] - projection * vy
        score = dx * dx + dy * dy + 0.15 * length2 * (t - 0.5) ** 2
        if t < 0.0 or t > 1.0:
            score += length2
        if region is not None:
            score += 0.25 * (
                (pos[0] - region[0]) ** 2 + (pos[1] - region[1]) ** 2
            )
        if score < best_score:
            best_score = score
            target = name

    yaw = state["eef_yaw"]
    initial_z = objects[target]["pos"][2]
    grasped = False

    for attempt in range(3):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 160:
            break

        bowl = state["objects"][target]["pos"]
        plate = state["objects"][plate_name]["pos"]
        clearance = max(bowl[2], plate[2], initial_z) + 0.17
        eef = state["eef_pos"]

        if not policy_move(
            sdk, [eef[0], eef[1], clearance], yaw, "open", 35
        ):
            continue

        policy_grip(sdk, "open", 8)
        if not policy_move(
            sdk, [bowl[0], bowl[1], clearance], yaw, "open", 45
        ):
            continue

        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        grasp_offset = [0.015, -0.005, 0.03][attempt]
        if not policy_move(
            sdk,
            [bowl[0], bowl[1], bowl[2] + grasp_offset],
            yaw,
            "open",
            45,
            0.006,
        ):
            continue

        before = sdk.get_state()["objects"][target]["pos"]
        policy_grip(sdk, "close", 15)
        eef = sdk.get_state()["eef_pos"]
        policy_move(
            sdk, [eef[0], eef[1], clearance], yaw, "close", 45
        )
        state = sdk.get_state()
        lifted = state["objects"][target]["pos"]
        if lifted[2] - before[2] > 0.065:
            grasped = True
            break

        policy_grip(sdk, "open", 8)
        yaw += pi / 2.0

    if not grasped:
        return {"status": "failed", "reason": "Bowl did not lift"}

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    plate = state["objects"][plate_name]["pos"]
    transport_z = max(bowl[2], plate[2] + 0.16)

    if not policy_move(
        sdk,
        [
            plate[0] - offset[0],
            plate[1] - offset[1],
            transport_z - offset[2],
        ],
        yaw,
        "close",
        55,
    ):
        return {"status": "failed", "reason": "Could not reach above plate"}

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    if bowl[2] < initial_z + 0.05:
        return {"status": "failed", "reason": "Bowl slipped during transport"}

    placement = [
        eef[0] + plate[0] - bowl[0],
        eef[1] + plate[1] - bowl[1],
        eef[2] + plate[2] + 0.025 - bowl[2],
    ]
    if not policy_move(sdk, placement, yaw, "close", 50, 0.006):
        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        plate = state["objects"][plate_name]["pos"]
        if (
            hypot(bowl[0] - plate[0], bowl[1] - plate[1]) > 0.035
            or bowl[2] > plate[2] + 0.065
        ):
            return {"status": "failed", "reason": "Placement pose unreachable"}

    policy_grip(sdk, "open", 15)
    eef = sdk.get_state()["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.14], yaw, "open", 35
    )
    policy_grip(sdk, "open", 10)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate = state["objects"][plate_name]["pos"]
    centered = hypot(bowl[0] - plate[0], bowl[1] - plate[1]) < 0.04
    settled = plate[2] - 0.025 <= bowl[2] <= plate[2] + 0.07
    return {
        "status": "done" if centered and settled else "failed",
        "target": target,
        "reason": "Placement verified by live poses"
        if centered and settled
        else "Final bowl pose outside plate placement tolerance",
    }
