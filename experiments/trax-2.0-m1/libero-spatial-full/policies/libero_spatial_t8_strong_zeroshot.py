def policy_move(sdk, pos, yaw, grip, limit=50, tol=0.01):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        pos,
        yaw=yaw,
        gripper=grip,
        max_steps=min(limit, remaining),
        tol=tol,
    )
    return result["reached"]


def policy_grip(sdk, action, steps=15):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    sdk.set_gripper(action, steps=min(steps, remaining))
    return True


def run_policy(sdk):
    state = sdk.get_state()
    objects = state["objects"]
    plates = [name for name in objects if "plate" in name]
    bowls = [
        name for name in objects
        if "black" in name and "bowl" in name
    ]
    if not plates or not bowls:
        return {"status": "failed", "reason": "Required objects unavailable"}

    sites = state["sites"]
    plate_region = sites.get("main_table_plate_region")
    plate_name = plates[0]
    if plate_region is not None:
        best = float("inf")
        for name in plates:
            pos = objects[name]["pos"]
            distance = hypot(pos[0] - plate_region[0], pos[1] - plate_region[1])
            if distance < best:
                best = distance
                plate_name = name

    reference = sites.get(
        "main_table_next_to_plate_region", objects[plate_name]["pos"]
    )
    bowl_name = bowls[0]
    best = float("inf")
    for name in bowls:
        pos = objects[name]["pos"]
        distance = hypot(pos[0] - reference[0], pos[1] - reference[1])
        if distance < best:
            best = distance
            bowl_name = name

    held = False
    yaw = state["eef_yaw"]
    for attempt in range(3):
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining < 175:
            break

        bowl = list(state["objects"][bowl_name]["pos"])
        plate = state["objects"][plate_name]["pos"]
        clearance = max(bowl[2], plate[2]) + 0.16
        eef = state["eef_pos"]
        if not policy_move(
            sdk, [eef[0], eef[1], max(eef[2], clearance)],
            yaw, "open", 35
        ):
            continue
        policy_grip(sdk, "open", 8)

        yaw = state["objects"][bowl_name]["yaw"] + attempt * pi / 2
        if not policy_move(
            sdk, [bowl[0], bowl[1], clearance], yaw, "open", 45
        ):
            continue

        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        grasp_offset = [0.025, 0.01, 0.04][attempt]
        if not policy_move(
            sdk, [bowl[0], bowl[1], bowl[2] + grasp_offset],
            yaw, "open", 40, 0.006
        ):
            continue

        policy_grip(sdk, "close", 20)
        state = sdk.get_state()
        lift_start = list(state["objects"][bowl_name]["pos"])
        eef = state["eef_pos"]
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.12],
            yaw, "close", 40, 0.008
        )
        state = sdk.get_state()
        lifted = state["objects"][bowl_name]["pos"]
        eef = state["eef_pos"]
        if (
            lifted[2] - lift_start[2] > 0.055
            and hypot(lifted[0] - eef[0], lifted[1] - eef[1]) < 0.075
        ):
            held = True
            break

    if not held:
        return {"status": "failed", "reason": "Could not verify bowl pickup"}

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    eef = state["eef_pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    plate = state["objects"][plate_name]["pos"]
    carry_z = max(eef[2], plate[2] + 0.15 - offset[2])

    if not policy_move(
        sdk, [eef[0], eef[1], carry_z], yaw, "close", 30
    ):
        return {"status": "failed", "reason": "Could not reach transport height"}

    target = [plate[0] - offset[0], plate[1] - offset[1], carry_z]
    if not policy_move(sdk, target, yaw, "close", 55, 0.008):
        return {"status": "failed", "reason": "Could not reach plate"}

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    eef = state["eef_pos"]
    plate = state["objects"][plate_name]["pos"]
    if bowl[2] < plate[2] + 0.05:
        return {"status": "failed", "reason": "Bowl lost during transport"}

    offset = [bowl[i] - eef[i] for i in range(3)]
    target = [
        plate[0] - offset[0],
        plate[1] - offset[1],
        plate[2] + 0.035 - offset[2],
    ]
    if not policy_move(sdk, target, yaw, "close", 45, 0.008):
        return {"status": "failed", "reason": "Placement approach incomplete"}

    if not policy_grip(sdk, "open", 18):
        return {"status": "failed", "reason": "Horizon exhausted before release"}

    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.12], yaw, "open", 30
    )
    policy_grip(sdk, "open", 12)

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    centered = hypot(bowl[0] - plate[0], bowl[1] - plate[1]) < 0.045
    supported_height = -0.015 <= bowl[2] - plate[2] <= 0.065
    released = state["gripper_width"] > 0.045
    if centered and supported_height and released:
        return {"status": "done", "object": bowl_name, "target": plate_name}
    return {"status": "failed", "reason": "Final placement could not be verified"}
