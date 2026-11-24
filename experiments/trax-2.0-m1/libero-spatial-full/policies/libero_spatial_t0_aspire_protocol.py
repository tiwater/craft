def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return state["max_steps"] - state["steps_used"]

    def move(position, yaw, grip, steps=35, tolerance=0.012):
        budget = remaining()
        if budget <= 0:
            return False
        sdk.move_to(
            position,
            yaw=yaw,
            gripper=grip,
            max_steps=min(steps, budget),
            tol=tolerance,
        )
        state = sdk.get_state()
        actual = state["eef_pos"]
        return sqrt(sum((actual[i] - position[i]) ** 2 for i in range(3))) < 0.025

    def grip(action, steps=10):
        budget = remaining()
        if budget > 0:
            sdk.set_gripper(action, steps=min(steps, budget))

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
    region = None
    for name in state["sites"]:
        if "between" in name and "plate" in name and "ramekin" in name:
            region = state["sites"][name]
            break

    target_name = bowls[0]
    best_score = None
    span = [ramekin[i] - plate[i] for i in range(2)]
    span_squared = max(sum(value * value for value in span), 0.000001)
    for name in bowls:
        position = objects[name]["pos"]
        fraction = sum(
            (position[i] - plate[i]) * span[i] for i in range(2)
        ) / span_squared
        projection = [
            plate[i] + min(1.0, max(0.0, fraction)) * span[i]
            for i in range(2)
        ]
        score = sum((position[i] - projection[i]) ** 2 for i in range(2))
        score += 0.15 * span_squared * (fraction - 0.5) ** 2
        if region is not None:
            score += 2.0 * sum(
                (position[i] - region[i]) ** 2 for i in range(2)
            )
        if best_score is None or score < best_score:
            best_score = score
            target_name = name

    initial_bowl_height = objects[target_name]["pos"][2]
    initial_plate_height = plate[2]
    yaw = state["eef_yaw"]
    held = False

    # Pinch a wall of the bowl rather than closing across its full diameter.
    # Keep wrist rotation fixed so the measured carrying offset stays valid.
    jaw_direction = [-sin(yaw), cos(yaw)]
    preferred_side = 1.0
    best_clearance = None
    bowl = objects[target_name]["pos"]
    for side in [1.0, -1.0]:
        point = [
            bowl[0] + side * 0.035 * jaw_direction[0],
            bowl[1] + side * 0.035 * jaw_direction[1],
        ]
        clearance = None
        for name in objects:
            if name != target_name:
                other = objects[name]["pos"]
                distance = hypot(point[0] - other[0], point[1] - other[1])
                if clearance is None or distance < clearance:
                    clearance = distance
        if best_clearance is None or clearance > best_clearance:
            best_clearance = clearance
            preferred_side = side

    for attempt in range(3):
        if remaining() < 110:
            break
        state = sdk.get_state()
        bowl = list(state["objects"][target_name]["pos"])
        plate = state["objects"][plate_name]["pos"]
        eef = state["eef_pos"]
        clearance_height = max(bowl[2], plate[2]) + 0.15

        # Do not abandon a useful approach solely because a tight SDK
        # convergence tolerance was missed.
        move(
            [eef[0], eef[1], max(eef[2], clearance_height)],
            yaw, "open", 25,
        )
        grip("open", 5)

        side = preferred_side if attempt != 1 else -preferred_side
        radial_offset = 0.035 if attempt < 2 else 0.025
        height_offset = 0.025 if attempt < 2 else 0.012
        grasp_xy = [
            bowl[0] + side * radial_offset * jaw_direction[0],
            bowl[1] + side * radial_offset * jaw_direction[1],
        ]
        move(
            [grasp_xy[0], grasp_xy[1], clearance_height],
            yaw, "open", 35,
        )
        state = sdk.get_state()
        eef = state["eef_pos"]
        if hypot(eef[0] - grasp_xy[0], eef[1] - grasp_xy[1]) > 0.025:
            continue

        bowl = list(state["objects"][target_name]["pos"])
        grasp_xy = [
            bowl[0] + side * radial_offset * jaw_direction[0],
            bowl[1] + side * radial_offset * jaw_direction[1],
        ]
        move(
            [grasp_xy[0], grasp_xy[1], bowl[2] + 0.065],
            yaw, "open", 20, 0.008,
        )
        move(
            [grasp_xy[0], grasp_xy[1], bowl[2] + height_offset],
            yaw, "open", 25, 0.006,
        )
        state = sdk.get_state()
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - grasp_xy[0], eef[1] - grasp_xy[1]) > 0.02
            or abs(eef[2] - bowl[2] - height_offset) > 0.025
        ):
            continue

        before_lift = list(state["objects"][target_name]["pos"])
        grip("close", 12)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move([eef[0], eef[1], eef[2] + 0.095], yaw, "close", 30, 0.01)
        state = sdk.get_state()
        lifted = state["objects"][target_name]["pos"]
        eef = state["eef_pos"]
        if (
            lifted[2] > before_lift[2] + 0.045
            and hypot(lifted[0] - eef[0], lifted[1] - eef[1]) < 0.10
        ):
            held = True
            break
        grip("open", 6)

    if not held:
        return {"status": "failed", "reason": "Bowl did not follow the test lift"}

    state = sdk.get_state()
    bowl = state["objects"][target_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    transit_height = max(eef[2], plate[2] + 0.14 - offset[2])
    move([eef[0], eef[1], transit_height], yaw, "close", 20)
    move(
        [plate[0] - offset[0], plate[1] - offset[1], transit_height],
        yaw, "close", 45, 0.01,
    )

    state = sdk.get_state()
    bowl = state["objects"][target_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    if bowl[2] < initial_bowl_height + 0.035:
        return {"status": "failed", "reason": "Bowl lost during transport"}

    # Correct using the object's actual location, including any grasp slip.
    correction = [plate[i] - bowl[i] for i in range(2)]
    if hypot(correction[0], correction[1]) > 0.012:
        move(
            [eef[0] + correction[0], eef[1] + correction[1], eef[2]],
            yaw, "close", 25, 0.007,
        )

    state = sdk.get_state()
    bowl = state["objects"][target_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    if hypot(bowl[0] - plate[0], bowl[1] - plate[1]) > 0.035:
        return {"status": "failed", "reason": "Could not center bowl over plate"}

    # Preserve the bowl's native resting-height offset and allow a short,
    # controlled settling drop rather than forcing it into the plate.
    resting_offset = max(0.012, initial_bowl_height - initial_plate_height)
    release_height = plate[2] + resting_offset + 0.025
    offset = [bowl[i] - eef[i] for i in range(3)]
    move(
        [
            plate[0] - offset[0],
            plate[1] - offset[1],
            release_height - offset[2],
        ],
        yaw, "close", 35, 0.008,
    )
    if remaining() < 8:
        return {"status": "failed", "reason": "Insufficient budget to release"}

    grip("open", 10)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.12], yaw, "open", 25)
    grip("open", 12)

    state = sdk.get_state()
    bowl = state["objects"][target_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if (
        hypot(bowl[0] - plate[0], bowl[1] - plate[1]) < 0.045
        and plate[2] - 0.015 < bowl[2] < plate[2] + resting_offset + 0.04
    ):
        return {"status": "done", "object": target_name}
    return {"status": "failed", "reason": "Final placement could not be verified"}
