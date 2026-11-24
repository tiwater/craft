def policy_move(sdk, position, yaw, gripper, steps=50, tol=0.008):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        position,
        yaw=yaw,
        gripper=gripper,
        max_steps=min(steps, remaining),
        tol=tol,
    )
    return result["reached"]


def policy_gripper(sdk, action, steps=12):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    sdk.set_gripper(action, steps=min(steps, remaining))
    return True


def run_policy(sdk):
    state = sdk.get_state()
    objects = state["objects"]
    bowls = [name for name in objects if "black_bowl" in name]
    plates = [name for name in objects if "plate" in name]
    ramekins = [name for name in objects if "ramekin" in name]
    if not bowls or not plates or not ramekins:
        return {"status": "failed", "reason": "Required objects are missing"}

    plate_name = plates[0]
    plate = objects[plate_name]["pos"]
    ramekin = objects[ramekins[0]]["pos"]
    midpoint = [
        (plate[0] + ramekin[0]) * 0.5,
        (plate[1] + ramekin[1]) * 0.5,
    ]
    region = None
    for name in state["sites"]:
        if "between_plate_ramekin" in name:
            region = state["sites"][name]
            break

    target_name = bowls[0]
    best_score = None
    for name in bowls:
        position = objects[name]["pos"]
        score = (
            (position[0] - midpoint[0]) ** 2
            + (position[1] - midpoint[1]) ** 2
        )
        if region is not None:
            score += (
                (position[0] - region[0]) ** 2
                + (position[1] - region[1]) ** 2
            )
        if best_score is None or score < best_score:
            best_score = score
            target_name = name

    held = False
    offset = None
    grasp_yaw = state["eef_yaw"]

    for attempt in range(3):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 150:
            break

        bowl = list(state["objects"][target_name]["pos"])
        plate = state["objects"][plate_name]["pos"]
        grasp_yaw = state["eef_yaw"] if attempt == 0 else grasp_yaw + pi / 2
        clearance = max(bowl[2], plate[2]) + 0.16
        eef = state["eef_pos"]

        if not policy_move(
            sdk, [eef[0], eef[1], max(eef[2], clearance)],
            grasp_yaw, "open", 35
        ):
            continue

        policy_gripper(sdk, "open", 8)
        if not policy_move(
            sdk, [bowl[0], bowl[1], clearance],
            grasp_yaw, "open", 45
        ):
            continue

        state = sdk.get_state()
        bowl = list(state["objects"][target_name]["pos"])
        grasp_height = [0.015, 0.005, 0.025][attempt]
        if not policy_move(
            sdk, [bowl[0], bowl[1], bowl[2] + grasp_height],
            grasp_yaw, "open", 40, 0.005
        ):
            continue

        policy_gripper(sdk, "close", 15)
        state = sdk.get_state()
        eef = state["eef_pos"]
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.12],
            grasp_yaw, "close", 40
        )
        state = sdk.get_state()
        lifted = state["objects"][target_name]["pos"]
        eef = state["eef_pos"]

        if (
            lifted[2] - bowl[2] > 0.055
            and hypot(lifted[0] - eef[0], lifted[1] - eef[1]) < 0.075
        ):
            offset = [lifted[i] - eef[i] for i in range(3)]
            held = True
            break

        policy_gripper(sdk, "open", 8)

    if not held:
        return {"status": "failed", "reason": "Could not verify a lifted bowl"}

    state = sdk.get_state()
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    transit_height = max(eef[2], plate[2] + 0.15 - offset[2])
    if not policy_move(
        sdk, [eef[0], eef[1], transit_height],
        grasp_yaw, "close", 30
    ):
        return {"status": "failed", "reason": "Could not reach transport clearance"}

    if not policy_move(
        sdk,
        [plate[0] - offset[0], plate[1] - offset[1], transit_height],
        grasp_yaw, "close", 55
    ):
        return {"status": "failed", "reason": "Could not reach above the plate"}

    state = sdk.get_state()
    plate = state["objects"][plate_name]["pos"]
    bowl = state["objects"][target_name]["pos"]
    eef = state["eef_pos"]
    if bowl[2] < plate[2] + 0.05:
        return {"status": "failed", "reason": "Bowl lost during transport"}

    offset = [bowl[i] - eef[i] for i in range(3)]
    destination = [
        plate[0] - offset[0],
        plate[1] - offset[1],
        plate[2] + 0.025 - offset[2],
    ]
    if not policy_move(sdk, destination, grasp_yaw, "close", 45, 0.006):
        return {"status": "failed", "reason": "Could not lower bowl onto plate"}

    if not policy_gripper(sdk, "open", 15):
        return {"status": "failed", "reason": "Step budget exhausted before release"}

    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.13],
        grasp_yaw, "open", 35
    )
    policy_gripper(sdk, "open", 10)

    state = sdk.get_state()
    bowl = state["objects"][target_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    centered = hypot(bowl[0] - plate[0], bowl[1] - plate[1]) < 0.045
    resting_height = plate[2] - 0.02 < bowl[2] < plate[2] + 0.06
    if centered and resting_height:
        return {"status": "done", "object": target_name}
    return {"status": "failed", "reason": "Final placement could not be verified"}
