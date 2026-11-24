def policy_move(sdk, pos, yaw, gripper, limit=30, tol=0.005, reserve=0):
    state = sdk.get_state()
    available = state["max_steps"] - state["steps_used"] - reserve
    if available <= 0:
        return False
    result = sdk.move_to(
        pos, yaw=yaw, gripper=gripper,
        max_steps=min(limit, available), tol=tol,
    )
    return result["reached"]


def policy_grip(sdk, action, steps=12, reserve=0):
    state = sdk.get_state()
    available = state["max_steps"] - state["steps_used"] - reserve
    if available <= 0:
        return False
    sdk.set_gripper(action, steps=min(steps, available))
    return True


def rim_contact(state, target, offset, height):
    obj = state["objects"][target]
    return [
        obj["pos"][0] + offset * cos(obj["yaw"]),
        obj["pos"][1] + offset * sin(obj["yaw"]),
        obj["pos"][2] + height,
    ]


def approach_and_contact(sdk, target, plate_name, yaw):
    reference = sdk.get_state()["objects"][target]["pos"][2]

    for offset in [-0.05, 0.045]:
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 245:
            break

        bowl = state["objects"][target]["pos"]
        plate = state["objects"][plate_name]["pos"]
        reference = bowl[2]
        clearance = max(bowl[2], plate[2]) + 0.135
        eef = state["eef_pos"]

        policy_grip(sdk, "open", 6, 140)
        policy_move(
            sdk, [eef[0], eef[1], max(eef[2], clearance)],
            yaw, "open", 20, 0.006, 140,
        )
        state = sdk.get_state()
        contact = rim_contact(state, target, offset, 0.015)
        policy_move(
            sdk, [contact[0], contact[1], clearance],
            yaw, "open", 30, 0.004, 140,
        )

        state = sdk.get_state()
        contact = rim_contact(state, target, offset, 0.015)
        if hypot(
            state["eef_pos"][0] - contact[0],
            state["eef_pos"][1] - contact[1],
        ) > 0.008:
            continue

        # Establish alignment above the rim before entering contact.
        precontact = rim_contact(state, target, offset, 0.055)
        policy_move(sdk, precontact, yaw, "open", 18, 0.004, 140)
        state = sdk.get_state()
        contact = rim_contact(state, target, offset, 0.015)
        policy_move(sdk, contact, yaw, "open", 24, 0.003, 140)

        state = sdk.get_state()
        contact = rim_contact(state, target, offset, 0.015)
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - contact[0], eef[1] - contact[1]) > 0.004
            or abs(eef[2] - contact[2]) > 0.005
        ):
            policy_move(sdk, contact, yaw, "open", 12, 0.003, 140)

        state = sdk.get_state()
        contact = rim_contact(state, target, offset, 0.015)
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - contact[0], eef[1] - contact[1]) > 0.008
            or abs(eef[2] - contact[2]) > 0.009
        ):
            continue

        reference = state["objects"][target]["pos"][2]
        policy_grip(sdk, "close", 18, 140)
        state = sdk.get_state()
        origin = list(state["eef_pos"])

        # Load the rim gradually, then verify sustained object motion.
        policy_move(
            sdk, [origin[0], origin[1], origin[2] + 0.025],
            yaw, "close", 14, 0.004, 140,
        )
        state = sdk.get_state()
        first_bowl = list(state["objects"][target]["pos"])
        first_eef = list(state["eef_pos"])
        first_lift = first_bowl[2] - reference

        if first_lift >= 0.009 and state["gripper_width"] >= 0.0045:
            policy_move(
                sdk, [origin[0], origin[1], origin[2] + 0.080],
                yaw, "close", 22, 0.004, 140,
            )
            policy_grip(sdk, "close", 5, 140)
            state = sdk.get_state()
            bowl = state["objects"][target]["pos"]
            eef = state["eef_pos"]
            drift = sqrt(sum(
                ((bowl[i] - first_bowl[i]) -
                 (eef[i] - first_eef[i])) ** 2
                for i in range(3)
            ))
            if (
                bowl[2] - reference >= 0.04
                and 0.0045 <= state["gripper_width"] <= 0.045
                and drift < 0.018
            ):
                return {"grasped": True, "reference": reference}

        # Return a partially lifted object to support before opening.
        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        eef = state["eef_pos"]
        if bowl[2] > reference + 0.01:
            policy_move(
                sdk,
                [eef[0], eef[1], eef[2] - (bowl[2] - reference)],
                yaw, "close", 14, 0.005, 140,
            )
        policy_grip(sdk, "open", 8, 140)

    return {"grasped": False, "reference": reference}


def run_policy(sdk):
    state = sdk.get_state()
    objects = state["objects"]
    bowls = [name for name in objects if "black_bowl" in name]
    plates = [name for name in objects if "plate" in name]
    ramekins = [name for name in objects if "ramekin" in name]
    if not bowls or not plates or not ramekins:
        return {"status": "failed", "reason": "Required objects unavailable"}

    plate_name = plates[0]
    plate = objects[plate_name]["pos"]
    ramekin = objects[ramekins[0]]["pos"]
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
    grasp = approach_and_contact(sdk, target, plate_name, yaw)
    if not grasp["grasped"]:
        return {
            "status": "failed",
            "target": target,
            "reason": "No verified sustained rim grasp",
        }

    for segment in range(4):
        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        plate = state["objects"][plate_name]["pos"]
        eef = state["eef_pos"]
        dx = plate[0] - bowl[0]
        dy = plate[1] - bowl[1]
        distance = hypot(dx, dy)
        if distance <= 0.006:
            break
        if state["max_steps"] - state["steps_used"] <= 95:
            break
        if (
            bowl[2] < grasp["reference"] + 0.03
            or state["gripper_width"] < 0.004
        ):
            return {"status": "failed", "reason": "Grasp lost during transport"}

        fraction = min(1.0, 0.075 / max(distance, 0.000001))
        policy_move(
            sdk,
            [
                eef[0] + fraction * dx,
                eef[1] + fraction * dy,
                eef[2] + max(0.0, plate[2] + 0.085 - bowl[2]),
            ],
            yaw, "close", 24, 0.005, 95,
        )

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    if hypot(bowl[0] - plate[0], bowl[1] - plate[1]) > 0.006:
        policy_move(
            sdk,
            [eef[0] + plate[0] - bowl[0],
             eef[1] + plate[1] - bowl[1], eef[2]],
            yaw, "close", 24, 0.004, 75,
        )

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if hypot(bowl[0] - plate[0], bowl[1] - plate[1]) > 0.018:
        return {"status": "failed", "reason": "Plate alignment failed"}

    for height in [0.045, 0.012]:
        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        plate = state["objects"][plate_name]["pos"]
        eef = state["eef_pos"]
        policy_move(
            sdk,
            [
                eef[0] + plate[0] - bowl[0],
                eef[1] + plate[1] - bowl[1],
                eef[2] + plate[2] + height - bowl[2],
            ],
            yaw, "close", 23, 0.004, 35,
        )

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if (
        hypot(bowl[0] - plate[0], bowl[1] - plate[1]) > 0.022
        or bowl[2] > plate[2] + 0.045
        or bowl[2] < plate[2] - 0.02
        or state["max_steps"] - state["steps_used"] < 35
    ):
        return {"status": "failed", "reason": "Safe release geometry unavailable"}

    policy_grip(sdk, "open", 12, 23)
    state = sdk.get_state()
    if state["gripper_width"] < 0.05:
        policy_grip(sdk, "open", 5, 18)

    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk, [eef[0], eef[1], eef[2] + 0.10],
        yaw, "open", 20, 0.008, 8,
    )
    policy_grip(sdk, "open", 2, 6)
    before = list(sdk.get_state()["objects"][target]["pos"])
    policy_grip(sdk, "open", 6)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate = state["objects"][plate_name]["pos"]
    success = (
        hypot(bowl[0] - plate[0], bowl[1] - plate[1]) < 0.03
        and plate[2] - 0.015 <= bowl[2] <= plate[2] + 0.045
        and sqrt(sum((bowl[i] - before[i]) ** 2 for i in range(3))) < 0.008
        and state["gripper_width"] > 0.05
        and state["eef_pos"][2] - bowl[2] > 0.07
    )
    return {
        "status": "done" if success else "failed",
        "target": target,
        "reason": "Bowl released and settled on plate"
        if success else "Final placement verification failed",
        "steps_used": state["steps_used"],
    }
