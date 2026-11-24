def policy_angle(angle):
    return atan2(sin(angle), cos(angle))


def policy_remaining(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def policy_move(sdk, pos, yaw, gripper, steps=30, tol=0.005):
    remaining = policy_remaining(sdk)
    if remaining <= 0:
        return False
    result = sdk.move_to(
        pos,
        yaw=yaw,
        gripper=gripper,
        max_steps=min(steps, remaining),
        tol=tol,
    )
    return result["reached"]


def policy_delta(sdk, displacement, rotation=0.0, steps=20, reserve=0):
    available = policy_remaining(sdk) - reserve
    if available <= 0:
        return False
    sdk.move_delta(
        displacement,
        dyaw=rotation,
        gripper="close",
        steps=min(steps, available),
    )
    return True


def policy_grip(sdk, action, steps):
    remaining = policy_remaining(sdk)
    if remaining > 0:
        sdk.set_gripper(action, steps=min(steps, remaining))


def policy_peg_yaw(state):
    for name in ("peg", "square_peg", "peg1"):
        if name in state["objects"]:
            return state["objects"][name]["yaw"]
    return 0.0


def policy_center_delta(state, center, rotation):
    nut = state["objects"]["nut"]["pos"]
    eef = state["eef_pos"]
    offset_x = eef[0] - nut[0]
    offset_y = eef[1] - nut[1]
    return [
        center[0] - nut[0]
        + (cos(rotation) - 1.0) * offset_x
        - sin(rotation) * offset_y,
        center[1] - nut[1]
        + sin(rotation) * offset_x
        + (cos(rotation) - 1.0) * offset_y,
        center[2] - nut[2],
    ]


def policy_align(sdk, clearance_z, reserve):
    for iteration in range(8):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        yaw_error = policy_angle(
            4.0 * (policy_peg_yaw(state) - nut["yaw"])
        ) / 4.0
        xy_error = hypot(
            goal[0] - nut["pos"][0], goal[1] - nut["pos"][1]
        )
        if xy_error < 0.004 and abs(yaw_error) < 0.035:
            return True
        if state["gripper_width"] < 0.010:
            return False
        rotation = max(-0.40, min(0.40, yaw_error))
        displacement = policy_center_delta(
            state, [goal[0], goal[1], clearance_z], rotation
        )
        if not policy_delta(
            sdk,
            displacement,
            rotation,
            40 if xy_error > 0.10 else 12,
            reserve,
        ):
            break

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    return (
        hypot(goal[0] - nut["pos"][0], goal[1] - nut["pos"][1]) < 0.006
        and abs(policy_angle(
            4.0 * (policy_peg_yaw(state) - nut["yaw"])
        ) / 4.0) < 0.06
    )


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or goal pose"}

    resting_z = state["objects"]["nut"]["pos"][2]
    grasped = False

    for attempt in range(2):
        if policy_remaining(sdk) < 190:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        approach_z = max(state["eef_pos"][2], nut["pos"][2] + 0.10)
        policy_move(
            sdk,
            [state["eef_pos"][0], state["eef_pos"][1], approach_z],
            state["eef_yaw"],
            "open",
            20,
            0.007,
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        grasp_yaw = state["eef_yaw"] + policy_angle(
            2.0 * (yaw + pi / 2.0 - state["eef_yaw"])
        ) / 2.0
        target = [
            nut["pos"][0] + 0.055 * cos(yaw),
            nut["pos"][1] + 0.055 * sin(yaw),
            approach_z,
        ]
        reached = policy_move(sdk, target, grasp_yaw, "open", 55, 0.007)
        if not reached:
            policy_move(sdk, target, grasp_yaw, "open", 12, 0.007)

        state = sdk.get_state()
        if hypot(
            state["eef_pos"][0] - target[0],
            state["eef_pos"][1] - target[1],
        ) > 0.009:
            return {"status": "failed", "reason": "Handle approach did not converge"}

        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        grasp_yaw = state["eef_yaw"] + policy_angle(
            2.0 * (yaw + pi / 2.0 - state["eef_yaw"])
        ) / 2.0
        target = [
            nut["pos"][0] + 0.055 * cos(yaw),
            nut["pos"][1] + 0.055 * sin(yaw),
            nut["pos"][2],
        ]
        policy_move(sdk, target, grasp_yaw, "open", 42, 0.006)

        # Re-measure the handle before correcting an unfinished descent.
        for correction in range(2):
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            yaw = nut["yaw"]
            target = [
                nut["pos"][0] + 0.055 * cos(yaw),
                nut["pos"][1] + 0.055 * sin(yaw),
                nut["pos"][2],
            ]
            displacement = [
                target[axis] - state["eef_pos"][axis]
                for axis in range(3)
            ]
            if sqrt(sum(value * value for value in displacement)) < 0.006:
                break
            remaining = policy_remaining(sdk)
            if remaining < 190:
                break
            sdk.move_delta(
                displacement,
                dyaw=policy_angle(
                    2.0 * (yaw + pi / 2.0 - state["eef_yaw"])
                ) / 2.0,
                gripper="open",
                steps=10,
            )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        target = [
            nut["pos"][0] + 0.055 * cos(nut["yaw"]),
            nut["pos"][1] + 0.055 * sin(nut["yaw"]),
            nut["pos"][2],
        ]
        if sqrt(sum(
            (target[axis] - state["eef_pos"][axis]) ** 2
            for axis in range(3)
        )) > 0.009:
            return {"status": "failed", "reason": "Handle descent did not converge"}

        before_z = nut["pos"][2]
        policy_grip(sdk, "close", 18)

        # Lift directly to clearance, avoiding two slow hover convergences.
        state = sdk.get_state()
        clearance_z = max(1.065, state["goal_pos"][2] + 0.215)
        policy_delta(
            sdk,
            [0.0, 0.0, clearance_z - state["objects"]["nut"]["pos"][2]],
            steps=38,
            reserve=135,
        )

        state = sdk.get_state()
        grasped = (
            state["objects"]["nut"]["pos"][2] - before_z >= 0.05
            and state["gripper_width"] > 0.012
        )
        if grasped:
            break
        policy_grip(sdk, "open", 6)

    if not grasped:
        return {"status": "failed", "reason": "Could not verify a 0.05 m nut lift"}

    # The previous clearance failures were only millimeters short.
    for correction in range(3):
        state = sdk.get_state()
        nut_z = state["objects"]["nut"]["pos"][2]
        if nut_z >= 1.05:
            break
        if not policy_delta(
            sdk,
            [0.0, 0.0, clearance_z - nut_z],
            steps=10,
            reserve=125,
        ):
            break

    state = sdk.get_state()
    if state["objects"]["nut"]["pos"][2] < 1.05:
        return {"status": "failed", "reason": "Could not establish insertion clearance"}

    if not policy_align(sdk, clearance_z, 100):
        return {"status": "failed", "reason": "Unable to align nut hole above peg"}

    state = sdk.get_state()
    insertion_z = state["goal_pos"][2] + 0.015
    upper_z = state["objects"]["nut"]["pos"][2]
    stage_heights = [(upper_z + insertion_z) / 2.0, insertion_z]

    # Use two sustained vertical trajectories, with live checks between them.
    for stage in range(2):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        yaw_error = policy_angle(
            4.0 * (policy_peg_yaw(state) - nut["yaw"])
        ) / 4.0
        if (
            hypot(goal[0] - nut["pos"][0], goal[1] - nut["pos"][1]) >= 0.006
            or abs(yaw_error) >= 0.06
        ):
            if not policy_align(
                sdk, nut["pos"][2], 80 if stage == 0 else 50
            ):
                return {"status": "failed", "reason": "Alignment drift before descent"}

        state = sdk.get_state()
        if state["gripper_width"] < 0.010:
            return {"status": "failed", "reason": "Grasp lost during insertion"}

        if not policy_delta(
            sdk,
            [0.0, 0.0, stage_heights[stage] - state["objects"]["nut"]["pos"][2]],
            steps=28,
            reserve=56 if stage == 0 else 28,
        ):
            return {"status": "failed", "reason": "Insufficient insertion budget"}

    # Finish residual tracking error against the fixed seated target.
    for correction in range(3):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        yaw_error = policy_angle(
            4.0 * (policy_peg_yaw(state) - nut["yaw"])
        ) / 4.0
        xy_error = hypot(goal[0] - nut["pos"][0], goal[1] - nut["pos"][1])
        if (
            abs(nut["pos"][2] - insertion_z) < 0.005
            and xy_error < 0.006
            and abs(yaw_error) < 0.06
        ):
            break
        if state["gripper_width"] < 0.010:
            return {"status": "failed", "reason": "Grasp lost during insertion"}

        next_z = insertion_z
        if xy_error >= 0.006 or abs(yaw_error) >= 0.06:
            next_z = nut["pos"][2]
        rotation = max(-0.06, min(0.06, yaw_error))
        if not policy_delta(
            sdk,
            policy_center_delta(state, [goal[0], goal[1], next_z], rotation),
            rotation,
            12,
            28,
        ):
            break

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    if (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) >= 0.006
        or abs(nut["pos"][2] - insertion_z) > 0.006
        or abs(policy_angle(
            4.0 * (policy_peg_yaw(state) - nut["yaw"])
        ) / 4.0) > 0.06
    ):
        return {"status": "failed", "reason": "Nut did not reach release pose"}

    policy_grip(sdk, "open", 10)
    state = sdk.get_state()
    retreat = list(state["eef_pos"])
    retreat[2] += 0.08
    policy_move(sdk, retreat, state["eef_yaw"], "open", 18)

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    xy_error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    yaw_error = abs(policy_angle(
        4.0 * (policy_peg_yaw(state) - nut["yaw"])
    ) / 4.0)
    seated = (
        xy_error < 0.012
        and abs(nut["pos"][2] - resting_z) < 0.015
        and yaw_error < 0.12
    )
    return {
        "status": "done" if seated else "failed",
        "reason": "Released and geometrically seated" if seated else "Seating verification failed",
        "xy_error_m": xy_error,
        "yaw_error_rad": yaw_error,
        "steps_used": state["steps_used"],
    }
