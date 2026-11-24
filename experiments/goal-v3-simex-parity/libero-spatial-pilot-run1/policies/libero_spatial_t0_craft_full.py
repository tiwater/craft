def policy_move(sdk, pos, yaw, gripper, steps=40, tol=0.008):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        pos, yaw=yaw, gripper=gripper,
        max_steps=min(steps, remaining), tol=tol
    )
    return result["reached"]


def policy_gripper(sdk, action, steps=12):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    sdk.set_gripper(action, steps=min(steps, remaining))
    return True


def policy_refine(sdk, destination, closed, steps=16, tol=0.006):
    previous_error = None
    stalled = 0
    for index in range(steps):
        state = sdk.get_state()
        if state["steps_used"] >= state["max_steps"]:
            return False
        error = [destination[i] - state["eef_pos"][i] for i in range(3)]
        distance = sqrt(sum(value * value for value in error))
        if distance < tol:
            return True
        if previous_error is not None and previous_error - distance < 0.0003:
            stalled += 1
        else:
            stalled = 0
        if stalled >= 5:
            return False
        previous_error = distance
        action = [max(-0.28, min(0.28, value * 16.0)) for value in error]
        sdk.step_raw(
            action + [0.0, 0.0, 0.0, 1.0 if closed else -1.0],
            steps=1
        )
    return False


def run_policy(sdk):
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
    region = None
    for name in state["sites"]:
        if "between_plate_ramekin" in name:
            region = state["sites"][name]
            break

    vx = ramekin_pos[0] - plate_pos[0]
    vy = ramekin_pos[1] - plate_pos[1]
    length_squared = max(vx * vx + vy * vy, 0.000001)
    target = bowls[0]
    best_score = 1000000.0
    for name in bowls:
        pos = objects[name]["pos"]
        fraction = (
            (pos[0] - plate_pos[0]) * vx +
            (pos[1] - plate_pos[1]) * vy
        ) / length_squared
        bounded = max(0.0, min(1.0, fraction))
        score = (
            (pos[0] - plate_pos[0] - bounded * vx) ** 2 +
            (pos[1] - plate_pos[1] - bounded * vy) ** 2 +
            0.12 * (fraction - 0.5) ** 2 * length_squared
        )
        if region is not None:
            score += (
                (pos[0] - region[0]) ** 2 +
                (pos[1] - region[1]) ** 2
            )
        if score < best_score:
            target = name
            best_score = score

    initial_bowl_z = objects[target]["pos"][2]
    initial_plate_z = plate_pos[2]
    support_z = initial_bowl_z
    if "table_top" in state["sites"]:
        support_z = state["sites"]["table_top"][2]

    yaw = state["eef_yaw"]
    closing_axis = [-sin(yaw), cos(yaw)]
    held = False
    diagnostics = []

    for attempt in range(2):
        state = sdk.get_state()
        if state["max_steps"] - state["steps_used"] < 190:
            break

        pos = state["objects"][target]["pos"]
        baseline_z = pos[2]
        eef = state["eef_pos"]
        clearance = max(eef[2], support_z + 0.17, pos[2] + 0.16)

        policy_move(
            sdk, [eef[0], eef[1], clearance],
            yaw, "close" if attempt else "open", 24
        )
        policy_gripper(sdk, "open", 8)

        state = sdk.get_state()
        pos = state["objects"][target]["pos"]
        sign = 1.0
        if attempt == 0:
            best_clearance = -1.0
            for candidate_sign in [1.0, -1.0]:
                candidate = [
                    pos[0] + candidate_sign * 0.043 * closing_axis[0],
                    pos[1] + candidate_sign * 0.043 * closing_axis[1]
                ]
                nearest = 1000.0
                for name in state["objects"]:
                    if name != target:
                        other = state["objects"][name]["pos"]
                        nearest = min(
                            nearest,
                            hypot(candidate[0] - other[0], candidate[1] - other[1])
                        )
                if nearest > best_clearance:
                    best_clearance = nearest
                    sign = candidate_sign
            first_sign = sign
        else:
            sign = -first_sign

        radial_offset = 0.043 if attempt == 0 else 0.050
        rim_offset = [
            sign * radial_offset * closing_axis[0],
            sign * radial_offset * closing_axis[1]
        ]
        overhead = [
            pos[0] + rim_offset[0],
            pos[1] + rim_offset[1],
            clearance
        ]
        policy_move(sdk, overhead, yaw, "open", 30)
        policy_refine(sdk, overhead, False, 8)

        state = sdk.get_state()
        pos = state["objects"][target]["pos"]
        precontact = [
            pos[0] + rim_offset[0],
            pos[1] + rim_offset[1],
            pos[2] + 0.080
        ]
        policy_move(sdk, precontact, yaw, "open", 20)

        state = sdk.get_state()
        pos = state["objects"][target]["pos"]
        contact = [
            pos[0] + rim_offset[0],
            pos[1] + rim_offset[1],
            max(pos[2] + 0.030, support_z + 0.028)
        ]
        policy_move(sdk, contact, yaw, "open", 22, 0.005)
        policy_refine(sdk, contact, False, 12, 0.005)

        state = sdk.get_state()
        pos = state["objects"][target]["pos"]
        eef = state["eef_pos"]
        lateral_error = hypot(
            eef[0] - pos[0] - rim_offset[0],
            eef[1] - pos[1] - rim_offset[1]
        )
        if lateral_error > 0.020 or eef[2] - pos[2] > 0.075:
            diagnostics.append({"attempt": attempt, "contact_aligned": False})
            continue

        policy_gripper(sdk, "close", 16)
        state = sdk.get_state()
        eef = state["eef_pos"]
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.080],
            yaw, "close", 26, 0.008
        )
        state = sdk.get_state()
        bowl_pos = state["objects"][target]["pos"]
        lift = bowl_pos[2] - baseline_z
        width = state["gripper_width"]
        diagnostics.append({
            "attempt": attempt,
            "lift": lift,
            "gripper_width": width
        })
        if lift < 0.045 or width <= 0.010 or width >= 0.077:
            continue

        offset = [
            bowl_pos[i] - state["eef_pos"][i]
            for i in range(3)
        ]
        policy_gripper(sdk, "close", 5)
        state = sdk.get_state()
        bowl_pos = state["objects"][target]["pos"]
        current_offset = [
            bowl_pos[i] - state["eef_pos"][i]
            for i in range(3)
        ]
        if (
            bowl_pos[2] - baseline_z > 0.040 and
            state["gripper_width"] > 0.010 and
            sqrt(sum(
                (current_offset[i] - offset[i]) ** 2 for i in range(3)
            )) < 0.015
        ):
            held = True
            break

    # A thin rim can give a narrow width despite a successful lift.
    # Validate its measured attachment without requiring a second large lift.
    if not held:
        state = sdk.get_state()
        before_bowl = list(state["objects"][target]["pos"])
        before_eef = list(state["eef_pos"])
        if (
            before_bowl[2] - initial_bowl_z > 0.040 and
            0.003 < state["gripper_width"] < 0.077 and
            state["max_steps"] - state["steps_used"] >= 110
        ):
            destination = [
                before_eef[0], before_eef[1], before_eef[2] + 0.020
            ]
            policy_move(sdk, destination, yaw, "close", 16, 0.003)
            policy_refine(sdk, destination, True, 6, 0.003)
            state = sdk.get_state()
            after_bowl = state["objects"][target]["pos"]
            after_eef = state["eef_pos"]
            motion = sqrt(sum(
                (after_eef[i] - before_eef[i]) ** 2 for i in range(3)
            ))
            tracking_error = sqrt(sum(
                (
                    after_bowl[i] - before_bowl[i] -
                    after_eef[i] + before_eef[i]
                ) ** 2 for i in range(3)
            ))
            held = (
                after_bowl[2] - initial_bowl_z > 0.040 and
                motion > 0.004 and
                after_bowl[2] - before_bowl[2] > 0.003 and
                tracking_error < 0.009 and
                0.003 < state["gripper_width"] < 0.077
            )
            diagnostics.append({
                "tracking_error": tracking_error,
                "eef_motion": motion,
                "held": held
            })

    if not held:
        return {
            "status": "failed",
            "reason": "Rim grasp did not pass lift verification",
            "probes": diagnostics,
            "steps_used": sdk.get_state()["steps_used"]
        }

    state = sdk.get_state()
    if state["max_steps"] - state["steps_used"] < 85:
        return {"status": "failed", "reason": "Insufficient transport horizon"}

    bowl_pos = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    carry_z = max(bowl_pos[2], plate_pos[2] + 0.090)
    eef = state["eef_pos"]
    policy_move(
        sdk,
        [eef[0], eef[1], eef[2] + carry_z - bowl_pos[2]],
        yaw, "close", 12, 0.006
    )

    # Close the loop on the bowl center, including the actual rim-grasp offset.
    for segment in range(3):
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining <= 68:
            break
        bowl_pos = state["objects"][target]["pos"]
        plate_pos = state["objects"][plate]["pos"]
        eef = state["eef_pos"]
        if (
            bowl_pos[2] < initial_bowl_z + 0.030 or
            not 0.003 < state["gripper_width"] < 0.077
        ):
            return {"status": "failed", "reason": "Grasp lost during transport"}
        if hypot(
            plate_pos[0] - bowl_pos[0], plate_pos[1] - bowl_pos[1]
        ) < 0.005:
            break
        destination = [
            eef[0] + plate_pos[0] - bowl_pos[0],
            eef[1] + plate_pos[1] - bowl_pos[1],
            eef[2] + carry_z - bowl_pos[2]
        ]
        policy_move(
            sdk, destination, yaw, "close",
            min(30, remaining - 68), 0.003
        )

    # Reserve release, retreat and settling time before spending descent steps.
    # Table region sites are placement regions, not the plate's contact surface.
    seat_allowance = 0.012
    seating_started = False
    for index in range(65):
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining <= 36:
            break
        bowl_pos = state["objects"][target]["pos"]
        plate_pos = state["objects"][plate]["pos"]
        error_x = plate_pos[0] - bowl_pos[0]
        error_y = plate_pos[1] - bowl_pos[1]
        lateral = hypot(error_x, error_y)
        resting_z = (
            plate_pos[2] + initial_bowl_z - initial_plate_z + seat_allowance
        )
        desired_z = resting_z + 0.004
        if lateral > 0.012:
            desired_z = max(desired_z + 0.045, bowl_pos[2])
        else:
            seating_started = True
        error_z = desired_z - bowl_pos[2]
        if lateral < 0.006 and abs(error_z) < 0.005:
            break
        sdk.step_raw([
            max(-0.24, min(0.24, error_x * 18.0)),
            max(-0.24, min(0.24, error_y * 18.0)),
            max(-0.20, min(0.20, error_z * 16.0)),
            0.0, 0.0, 0.0, 1.0
        ], steps=1)

    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    resting_z = (
        plate_pos[2] + initial_bowl_z - initial_plate_z + seat_allowance
    )
    lateral = hypot(
        bowl_pos[0] - plate_pos[0], bowl_pos[1] - plate_pos[1]
    )
    diagnostics.append({
        "release_xy_error": lateral,
        "release_height": bowl_pos[2] - resting_z,
        "seating_started": seating_started
    })
    if lateral > 0.018 or bowl_pos[2] > resting_z + 0.045:
        return {
            "status": "failed",
            "reason": "Could not reach release pose",
            "probes": diagnostics,
            "steps_used": state["steps_used"]
        }

    # Open completely before moving the fingers past the rim.
    policy_gripper(sdk, "open", 12)
    state = sdk.get_state()
    eef = state["eef_pos"]
    remaining = state["max_steps"] - state["steps_used"]
    retreat = [eef[0], eef[1], eef[2] + 0.100]
    if remaining > 8:
        policy_move(
            sdk, retreat, yaw, "open", min(18, remaining - 8), 0.008
        )
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining > 0:
        policy_gripper(sdk, "open", min(12, remaining))

    state = sdk.get_state()
    bowl_pos = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    expected_z = (
        plate_pos[2] + initial_bowl_z - initial_plate_z + seat_allowance
    )
    success = (
        hypot(bowl_pos[0] - plate_pos[0], bowl_pos[1] - plate_pos[1]) < 0.025 and
        abs(bowl_pos[2] - expected_z) < 0.020 and
        state["gripper_width"] > 0.050 and
        state["eef_pos"][2] - bowl_pos[2] > 0.065
    )
    return {
        "status": "done" if success else "failed",
        "target": target,
        "probes": diagnostics,
        "steps_used": state["steps_used"]
    }
