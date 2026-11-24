def run_policy(sdk):
    def move(pos, yaw, grip, steps=35, tol=0.015):
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining <= 0:
            return False
        sdk.move_to(
            pos, yaw=yaw, gripper=grip,
            max_steps=min(steps, remaining), tol=tol
        )
        return True

    def gripper(action, steps=10):
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining <= 0:
            return False
        sdk.set_gripper(action, steps=min(steps, remaining))
        return True

    state = sdk.get_state()
    objects = state["objects"]
    bowls = [name for name in objects if "black_bowl" in name]
    plates = [name for name in objects if "plate" in name]
    ramekins = [name for name in objects if "ramekin" in name]
    if not bowls or not plates or not ramekins:
        return {"status": "failed", "reason": "Required objects missing"}

    plate = plates[0]
    ramekin = ramekins[0]
    plate_pos = objects[plate]["pos"]
    ramekin_pos = objects[ramekin]["pos"]
    vx = ramekin_pos[0] - plate_pos[0]
    vy = ramekin_pos[1] - plate_pos[1]
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
        t = ((pos[0] - plate_pos[0]) * vx +
             (pos[1] - plate_pos[1]) * vy) / length2
        bounded = max(0.0, min(1.0, t))
        score = (
            (pos[0] - plate_pos[0] - bounded * vx) ** 2 +
            (pos[1] - plate_pos[1] - bounded * vy) ** 2 +
            0.12 * length2 * (t - 0.5) ** 2
        )
        if region is not None:
            score += (
                (pos[0] - region[0]) ** 2 +
                (pos[1] - region[1]) ** 2
            )
        if score < best_score:
            target = name
            best_score = score

    original_z = objects[target]["pos"][2]
    original_plate_z = plate_pos[2]
    base_yaw = state["eef_yaw"]
    held = False
    yaw = base_yaw

    # Judge approach geometry from feedback; contact can prevent a strict
    # Cartesian convergence test from succeeding even at a usable grasp pose.
    for attempt in range(3):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 145:
            break

        bowl = state["objects"][target]["pos"]
        baseline = bowl[2]
        eef = state["eef_pos"]
        yaw = base_yaw + (pi / 2.0 if attempt == 1 else 0.0)
        clearance = max(bowl[2] + 0.13,
                        state["objects"][plate]["pos"][2] + 0.13)
        gripper("open", 6)
        if eef[2] < clearance - 0.025:
            move([eef[0], eef[1], clearance], yaw, "open", 22)

        # The last attempt pinches a single rim instead of spanning the bowl.
        radial = 0.045 if attempt == 2 else 0.0
        gx = bowl[0] + radial * cos(yaw)
        gy = bowl[1] + radial * sin(yaw)
        move([gx, gy, clearance], yaw, "open", 32)
        state = sdk.get_state()
        eef = state["eef_pos"]
        if hypot(eef[0] - gx, eef[1] - gy) > 0.035:
            move([gx, gy, clearance], yaw, "open", 18)

        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        gx = bowl[0] + radial * cos(yaw)
        gy = bowl[1] + radial * sin(yaw)
        height = 0.012 if attempt == 1 else 0.025
        gz = bowl[2] + height
        move([gx, gy, gz], yaw, "open", 38, 0.012)
        state = sdk.get_state()
        eef = state["eef_pos"]

        if (hypot(eef[0] - gx, eef[1] - gy) > 0.03 or
                abs(eef[2] - gz) > 0.045):
            continue

        gripper("close", 14)
        state = sdk.get_state()
        eef = state["eef_pos"]
        move([eef[0], eef[1], eef[2] + 0.10],
             yaw, "close", 28)
        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        if bowl[2] > baseline + 0.045:
            # Verify that the object continues following the hand.
            before = list(bowl)
            eef = state["eef_pos"]
            move([eef[0], eef[1], eef[2] + 0.035],
                 yaw, "close", 15, 0.012)
            state = sdk.get_state()
            bowl = state["objects"][target]["pos"]
            if bowl[2] > baseline + 0.065 and bowl[2] > before[2] + 0.008:
                held = True
                break

    if not held:
        return {"status": "failed", "reason": "No verified bowl lift",
                "steps_used": sdk.get_state()["steps_used"]}

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    plate_pos = state["objects"][plate]["pos"]
    resting_z = plate_pos[2] + original_z - original_plate_z + 0.018
    transit_z = max(eef[2], resting_z + 0.10 - offset[2])
    move([eef[0], eef[1], transit_z], yaw, "close", 18)
    move([plate_pos[0] - offset[0],
          plate_pos[1] - offset[1], transit_z],
         yaw, "close", 48, 0.012)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    if bowl[2] < original_z + 0.04:
        return {"status": "failed", "reason": "Bowl slipped"}

    eef = state["eef_pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    if hypot(bowl[0] - plate_pos[0], bowl[1] - plate_pos[1]) > 0.02:
        move([plate_pos[0] - offset[0],
              plate_pos[1] - offset[1], eef[2]],
             yaw, "close", 24, 0.01)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    if hypot(bowl[0] - plate_pos[0], bowl[1] - plate_pos[1]) > 0.035:
        return {"status": "failed", "reason": "Plate alignment failed"}

    eef = state["eef_pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    resting_z = plate_pos[2] + original_z - original_plate_z + 0.018
    move([plate_pos[0] - offset[0],
          plate_pos[1] - offset[1],
          resting_z + 0.008 - offset[2]],
         yaw, "close", 32, 0.012)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    if (hypot(bowl[0] - plate_pos[0], bowl[1] - plate_pos[1]) > 0.04 or
            bowl[2] > resting_z + 0.055):
        return {"status": "failed", "reason": "Placement pose too far away"}

    if not gripper("open", 12):
        return {"status": "failed", "reason": "No release budget"}
    state = sdk.get_state()
    eef = state["eef_pos"]
    move([eef[0], eef[1], eef[2] + 0.11], yaw, "open", 24)
    gripper("open", 8)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    expected_z = plate_pos[2] + original_z - original_plate_z + 0.018
    placed = (
        hypot(bowl[0] - plate_pos[0], bowl[1] - plate_pos[1]) < 0.04 and
        abs(bowl[2] - expected_z) < 0.035
    )
    return {"status": "done" if placed else "failed",
            "target": target, "steps_used": state["steps_used"]}
