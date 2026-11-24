def angle_difference(target, current):
    return atan2(sin(target - current), cos(target - current))


def square_alignment(yaw):
    candidates = [angle_difference(k * pi / 2.0, yaw) for k in range(-2, 3)]
    return min(candidates, key=abs)


def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def bounded_move(sdk, position, yaw, gripper, steps, tolerance):
    budget = min(steps, remaining_steps(sdk))
    if budget <= 0:
        return {"reached": False}
    return sdk.move_to(
        position, yaw=yaw, gripper=gripper,
        max_steps=budget, tol=tolerance,
    )


def actuate(sdk, action, steps):
    budget = min(steps, remaining_steps(sdk))
    if budget > 0:
        sdk.set_gripper(action, steps=budget)


def move_nut_center(sdk, target, steps, tolerance=0.003):
    state = sdk.get_state()
    nut = state["objects"]["nut"]["pos"]
    eef = state["eef_pos"]
    target_eef = [eef[i] + target[i] - nut[i] for i in range(3)]
    return bounded_move(
        sdk, target_eef, state["eef_yaw"], "close", steps, tolerance
    )


def handle_point(nut, height):
    return [
        nut["pos"][0] + 0.055 * cos(nut["yaw"]),
        nut["pos"][1] + 0.055 * sin(nut["yaw"]),
        nut["pos"][2] + height,
    ]


def nut_rotation_error(nut, target_yaw):
    x, y, z, w = nut["quat"]
    norm = sqrt(x * x + y * y + z * z + w * w)
    if norm < 0.000001:
        return [0.0, 0.0, angle_difference(target_yaw, nut["yaw"])]
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    sz = sin(target_yaw / 2.0)
    cw = cos(target_yaw / 2.0)
    error = [
        -cw * x + sz * y,
        -cw * y - sz * x,
        sz * w - cw * z,
    ]
    scalar = cw * w + sz * z
    if scalar < 0.0:
        error = [-value for value in error]
        scalar = -scalar
    length = sqrt(sum(value * value for value in error))
    if length < 0.000001:
        return [2.0 * value for value in error]
    scale = 2.0 * atan2(length, scalar) / length
    return [scale * value for value in error]


def servo_nut(sdk, state, target, target_yaw, descent=False):
    nut = state["objects"]["nut"]
    rotation = nut_rotation_error(nut, target_yaw)
    eef = state["eef_pos"]
    offset = [nut["pos"][i] - eef[i] for i in range(3)]
    angular = [max(-0.10, min(0.10, 0.35 * value)) for value in rotation]
    sweep = [
        angular[1] * offset[2] - angular[2] * offset[1],
        angular[2] * offset[0] - angular[0] * offset[2],
        angular[0] * offset[1] - angular[1] * offset[0],
    ]
    displacement = [
        0.45 * (target[i] - nut["pos"][i]) - sweep[i]
        for i in range(3)
    ]
    limit = 0.012 if descent else 0.035
    displacement = [max(-limit, min(limit, value)) for value in displacement]
    if descent:
        displacement[2] = max(-0.004, min(0.006, displacement[2]))
    action = [value / 0.05 for value in displacement]
    action += [value / 0.5 for value in angular]
    action += [1.0]
    if remaining_steps(sdk) > 0:
        sdk.step_raw(action, steps=1)


def seating_servo(sdk, state, target, target_yaw, memory, descent):
    nut = state["objects"]["nut"]
    pos = nut["pos"]
    rotation = nut_rotation_error(nut, target_yaw)
    offset = [pos[i] - state["eef_pos"][i] for i in range(3)]
    error = [target[i] - pos[i] for i in range(3)]

    for i in range(3):
        if abs(error[i]) < 0.025:
            memory[i] = max(-0.008, min(0.008, memory[i] + 0.055 * error[i]))
        else:
            memory[i] *= 0.9

    gain = 0.50 if descent else 0.30
    angular = [gain * value for value in rotation]
    angular_norm = sqrt(sum(value * value for value in angular))
    angular_limit = 0.065 if descent else 0.045
    if angular_norm > angular_limit:
        angular = [value * angular_limit / angular_norm for value in angular]

    sweep = [
        angular[1] * offset[2] - angular[2] * offset[1],
        angular[2] * offset[0] - angular[0] * offset[2],
        angular[0] * offset[1] - angular[1] * offset[0],
    ]
    gain = 0.80 if descent else 0.60
    displacement = [
        gain * error[i] + memory[i] - sweep[i]
        for i in range(3)
    ]

    if descent:
        displacement = [max(-0.012, min(0.012, value)) for value in displacement]
        displacement[2] = max(-0.006, min(0.008, displacement[2]))
    else:
        length = sqrt(sum(value * value for value in displacement))
        if length > 0.025:
            displacement = [value * 0.025 / length for value in displacement]

    action = [value / 0.05 for value in displacement]
    action += [value / 0.5 for value in angular]
    action += [1.0]
    if remaining_steps(sdk) > 0:
        sdk.step_raw(action, steps=1)


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    grasped = False
    lift_height = None

    for attempt in range(2):
        if remaining_steps(sdk) < 185:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        approach_z = max(nut["pos"][2] + 0.10, state["eef_pos"][2])
        actuate(sdk, "open", 10)
        bounded_move(
            sdk,
            [state["eef_pos"][0], state["eef_pos"][1], approach_z],
            state["eef_yaw"], "open", 16, 0.008,
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        grasp_yaw = state["eef_yaw"] + angle_difference(
            nut["yaw"], state["eef_yaw"]
        )
        point = handle_point(nut, 0.0)
        bounded_move(
            sdk, [point[0], point[1], approach_z],
            grasp_yaw, "open", 30, 0.003,
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        grasp_yaw = state["eef_yaw"] + angle_difference(
            nut["yaw"], state["eef_yaw"]
        )
        bounded_move(
            sdk, handle_point(nut, 0.025),
            grasp_yaw, "open", 18, 0.003,
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        grasp_yaw = state["eef_yaw"] + angle_difference(
            nut["yaw"], state["eef_yaw"]
        )
        contact_height = 0.0 if attempt == 0 else -0.003
        bounded_move(
            sdk, handle_point(nut, contact_height),
            grasp_yaw, "open", 24, 0.002,
        )

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        point = handle_point(nut, contact_height)
        eef = state["eef_pos"]
        contact_error = sqrt(sum((eef[i] - point[i]) ** 2 for i in range(3)))
        yaw_error = abs(angle_difference(nut["yaw"], state["eef_yaw"]))
        if contact_error > 0.005 or yaw_error > 0.06:
            grasp_yaw = state["eef_yaw"] + angle_difference(
                nut["yaw"], state["eef_yaw"]
            )
            bounded_move(
                sdk, point, grasp_yaw, "open", 12, 0.002,
            )

        before = sdk.get_state()
        point = handle_point(before["objects"]["nut"], contact_height)
        eef = before["eef_pos"]
        if (
            hypot(eef[0] - point[0], eef[1] - point[1]) > 0.010
            or abs(eef[2] - point[2]) > 0.018
            or abs(angle_difference(
                before["objects"]["nut"]["yaw"], before["eef_yaw"]
            )) > 0.15
        ):
            continue

        baseline_z = before["objects"]["nut"]["pos"][2]
        actuate(sdk, "close", 14)

        state = sdk.get_state()
        if state["gripper_width"] < 0.012:
            actuate(sdk, "open", 8)
            continue

        eef = state["eef_pos"]
        bounded_move(
            sdk, [eef[0], eef[1], eef[2] + 0.105],
            state["eef_yaw"], "close", 28, 0.006,
        )
        state = sdk.get_state()
        rise = state["objects"]["nut"]["pos"][2] - baseline_z
        if rise > 0.055 and 0.012 < state["gripper_width"] < 0.065:
            grasped = True
            lift_height = state["objects"]["nut"]["pos"][2]
            break

        actuate(sdk, "open", 6)

    if not grasped:
        return {"status": "failed", "reason": "Nut did not follow the lift"}

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    target_yaw = nut["yaw"] + square_alignment(nut["yaw"])
    clearance_z = max(nut["pos"][2], state["goal_pos"][2] + 0.100)
    memory = [0.0, 0.0, 0.0]
    aligned = False
    seated = False
    stable_ticks = 0
    stalled_ticks = 0
    recovery_ticks = 0
    recovery_count = 0
    recovery_z = clearance_z
    previous_z = nut["pos"][2]

    # Translate, clear the peg, and level concurrently. Avoid waiting for
    # precise posture at the grasp location near the workspace boundary.
    for tick in range(400):
        if remaining_steps(sdk) <= 24:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        pos = nut["pos"]
        goal = state["goal_pos"]
        rotation = nut_rotation_error(nut, target_yaw)
        orientation_error = sqrt(sum(value * value for value in rotation))
        error_xy = hypot(pos[0] - goal[0], pos[1] - goal[1])
        seat_z = goal[2] + 0.011

        if (
            error_xy < 0.006
            and goal[2] - 0.010 <= pos[2] <= seat_z + 0.006
            and orientation_error < 0.075
        ):
            stable_ticks += 1
        else:
            stable_ticks = 0

        if stable_ticks >= 4:
            seated = True
            break

        if state["gripper_width"] < 0.010 and pos[2] > seat_z + 0.025:
            return {
                "status": "failed",
                "reason": "Grasp lost during seating",
                "steps_used": state["steps_used"],
            }

        if not aligned:
            if error_xy < 0.0035 and orientation_error < 0.070:
                aligned = True
                memory[2] = 0.0
                stalled_ticks = 0
            else:
                target_x = goal[0]
                target_y = goal[1]
                # Maintain lateral clearance until the nut's lower face
                # clears the peg, without holding at the grasp location.
                safe_height = goal[2] + 0.090
                if pos[2] < safe_height and error_xy > 0.075:
                    fraction = min(1.0, max(0.0, (error_xy - 0.075) / error_xy))
                    target_x = pos[0] + fraction * (goal[0] - pos[0])
                    target_y = pos[1] + fraction * (goal[1] - pos[1])
                seating_servo(
                    sdk, state, [target_x, target_y, clearance_z],
                    target_yaw, memory, False,
                )
                previous_z = pos[2]
                continue

        if previous_z - pos[2] < 0.00025 and pos[2] > seat_z + 0.008:
            stalled_ticks += 1
        else:
            stalled_ticks = 0
        previous_z = pos[2]

        target_z = max(seat_z, pos[2] - 0.018)
        if error_xy > 0.0045 or orientation_error > 0.085:
            target_z = pos[2]
            memory[2] = max(0.0, memory[2])
        if error_xy > 0.012 or orientation_error > 0.18:
            target_z = min(clearance_z, pos[2] + 0.008)
            memory[2] = 0.0

        if (
            stalled_ticks >= 14
            and recovery_ticks == 0
            and recovery_count < 1
            and remaining_steps(sdk) > 58
        ):
            recovery_z = min(clearance_z, pos[2] + 0.009)
            recovery_ticks = 8
            recovery_count += 1
            memory[2] = 0.0

        if recovery_ticks > 0:
            target_z = recovery_z
            recovery_ticks -= 1
            stalled_ticks = 0
            if recovery_ticks == 0:
                memory[2] = 0.0

        seating_servo(
            sdk, state, [goal[0], goal[1], target_z],
            target_yaw, memory, True,
        )

    if not seated:
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        return {
            "status": "failed",
            "reason": (
                "Insertion depth was not verified"
                if aligned else "Could not verify level alignment above peg"
            ),
            "steps_used": state["steps_used"],
            "center_error_m": hypot(
                nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]
            ),
            "height_above_goal_m": nut["pos"][2] - goal[2],
            "orientation_error_rad": sqrt(sum(
                value * value for value in nut_rotation_error(nut, target_yaw)
            )),
        }

    state = sdk.get_state()
    release_pos = list(state["eef_pos"])
    release_quat = list(state["eef_quat"])
    for tick in range(min(12, remaining_steps(sdk))):
        state = sdk.get_state()
        x, y, z, w = state["eef_quat"]
        tx, ty, tz, tw = release_quat
        rotation = [
            tx * w - tw * x - ty * z + tz * y,
            ty * w - tw * y - tz * x + tx * z,
            tz * w - tw * z - tx * y + ty * x,
        ]
        scalar = tw * w + tx * x + ty * y + tz * z
        sign = 1.0 if scalar >= 0.0 else -1.0
        action = [
            max(-0.2, min(0.2, 8.0 * (release_pos[i] - state["eef_pos"][i])))
            for i in range(3)
        ]
        action += [max(-0.2, min(0.2, 1.4 * sign * value)) for value in rotation]
        action += [-1.0]
        sdk.step_raw(action, steps=1)

    retract_pos = [release_pos[0], release_pos[1], release_pos[2] + 0.08]
    for tick in range(min(18, remaining_steps(sdk))):
        state = sdk.get_state()
        x, y, z, w = state["eef_quat"]
        tx, ty, tz, tw = release_quat
        rotation = [
            tx * w - tw * x - ty * z + tz * y,
            ty * w - tw * y - tz * x + tx * z,
            tz * w - tw * z - tx * y + ty * x,
        ]
        scalar = tw * w + tx * x + ty * y + tz * z
        sign = 1.0 if scalar >= 0.0 else -1.0
        action = [
            max(-0.5, min(0.5, 14.0 * (retract_pos[i] - state["eef_pos"][i])))
            for i in range(3)
        ]
        action += [max(-0.2, min(0.2, 1.4 * sign * value)) for value in rotation]
        action += [-1.0]
        sdk.step_raw(action, steps=1)

    state = sdk.get_state()
    nut = state["objects"]["nut"]["pos"]
    goal = state["goal_pos"]
    centered = hypot(nut[0] - goal[0], nut[1] - goal[1]) < 0.012
    lowered = nut[2] < lift_height - 0.045
    at_seat = goal[2] - 0.015 <= nut[2] <= goal[2] + 0.025
    released = state["gripper_width"] > 0.055
    success = centered and lowered and at_seat and released
    return {
        "status": "done" if success else "failed",
        "reason": "Verified released nut pose" if success else "Final pose check failed",
        "steps_used": state["steps_used"],
    }
