def run_policy(sdk):
    def wrap(angle):
        return atan2(sin(angle), cos(angle))

    def square_error(angle):
        return (angle + pi / 4.0) % (pi / 2.0) - pi / 4.0

    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def rotation_error(target, current):
        tx, ty, tz, tw = target
        cx, cy, cz, cw = current
        error = [
            -tw * cx + tx * cw - ty * cz + tz * cy,
            -tw * cy + tx * cz + ty * cw - tz * cx,
            -tw * cz - tx * cy + ty * cx + tz * cw,
        ]
        scalar = tw * cw + tx * cx + ty * cy + tz * cz
        sign = 1.0 if scalar >= 0.0 else -1.0
        return [2.0 * sign * value for value in error]

    def down_quaternion(yaw):
        return [cos(yaw / 2.0), sin(yaw / 2.0), 0.0, 0.0]

    def command(position_error, orientation_error, grip, speed):
        translation = [clamp(5.0 * value, speed) for value in position_error]
        rotation = [clamp(2.0 * value, 0.45) for value in orientation_error]
        if remaining() > 0:
            sdk.step_raw(translation + rotation + [grip], steps=1)

    def move_eef(target, yaw, grip, ticks, tolerance=0.005):
        desired = down_quaternion(yaw)
        stable = 0
        for tick in range(min(ticks, remaining())):
            state = sdk.get_state()
            error = [target[i] - state["eef_pos"][i] for i in range(3)]
            rotation = rotation_error(desired, state["eef_quat"])
            if (
                sqrt(sum(value * value for value in error)) < tolerance
                and sqrt(sum(value * value for value in rotation)) < 0.045
            ):
                stable += 1
                if stable >= 2:
                    return True
            else:
                stable = 0
            command(error, rotation, grip, 0.85)
        return False

    def actuate(action, ticks):
        budget = min(ticks, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    def fail(reason):
        return {
            "status": "failed",
            "reason": reason,
            "steps_used": sdk.get_state()["steps_used"],
        }

    def nut_command(target, yaw, speed):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        desired = [0.0, 0.0, sin(yaw / 2.0), cos(yaw / 2.0)]
        rotation = rotation_error(desired, nut["quat"])
        error = [target[i] - nut["pos"][i] for i in range(3)]

        # Compensate the center motion induced by rotating a handle grasp.
        arm = [
            nut["pos"][i] - state["eef_pos"][i]
            for i in range(3)
        ]
        angular_step = [0.5 * clamp(2.0 * value, 0.45) for value in rotation]
        displacement = [
            angular_step[1] * arm[2] - angular_step[2] * arm[1],
            angular_step[2] * arm[0] - angular_step[0] * arm[2],
            angular_step[0] * arm[1] - angular_step[1] * arm[0],
        ]
        translation = [
            clamp(5.0 * error[i] - displacement[i] / 0.05, speed)
            for i in range(3)
        ]
        if remaining() > 0:
            sdk.step_raw(
                translation
                + [clamp(2.0 * value, 0.45) for value in rotation]
                + [1.0],
                steps=1,
            )

    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return fail("Missing nut or peg pose")

    resting_height = state["objects"]["nut"]["pos"][2]
    held = False

    for attempt in range(2):
        if remaining() < 175:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        grasp_yaw = state["eef_yaw"] + wrap(yaw - state["eef_yaw"])
        offset = 0.055
        handle = [
            nut["pos"][0] + offset * cos(yaw),
            nut["pos"][1] + offset * sin(yaw),
            nut["pos"][2] + 0.075,
        ]

        actuate("open", 5)
        move_eef(handle, grasp_yaw, -1.0, 42, 0.006)

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        before_lift = nut["pos"][2]
        grasp_yaw = state["eef_yaw"] + wrap(yaw - state["eef_yaw"])
        target = [
            nut["pos"][0] + offset * cos(yaw),
            nut["pos"][1] + offset * sin(yaw),
            nut["pos"][2] + (0.003 if attempt == 0 else -0.002),
        ]
        move_eef(target, grasp_yaw, -1.0, 32, 0.003)
        actuate("close", 12)

        state = sdk.get_state()
        if state["gripper_width"] > 0.012:
            target = list(state["eef_pos"])
            target[2] += 0.115
            move_eef(target, grasp_yaw, 1.0, 32, 0.006)
            state = sdk.get_state()
            held = (
                state["objects"]["nut"]["pos"][2] > before_lift + 0.050
                and state["gripper_width"] > 0.012
            )
            if held:
                break

        state = sdk.get_state()
        target = list(state["eef_pos"])
        target[2] = max(before_lift + 0.008, resting_height + 0.008)
        move_eef(target, grasp_yaw, 1.0, 16, 0.008)
        actuate("open", 7)

    if not held:
        return fail("Could not establish a lifting grasp")

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    aligned_yaw = nut["yaw"] - square_error(nut["yaw"])
    clearance = max(resting_height + 0.115, state["goal_pos"][2] + 0.085)
    centered = False

    for tick in range(min(80, max(0, remaining() - 110))):
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        lateral = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
        desired = [0.0, 0.0, sin(aligned_yaw / 2.0), cos(aligned_yaw / 2.0)]
        rotation = rotation_error(desired, nut["quat"])
        posture_error = sqrt(sum(value * value for value in rotation))

        if (
            lateral < 0.003
            and abs(nut["pos"][2] - clearance) < 0.007
            and posture_error < 0.045
        ):
            centered = True
            break

        nut_command([goal[0], goal[1], clearance], aligned_yaw, 0.85)

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    if hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) > 0.018:
        return fail("Could not center the nut above the peg")

    seated = False
    previous_height = nut["pos"][2]
    stalled_ticks = 0
    recovery_ticks = 0
    recovery_count = 0
    recovery_height = previous_height
    stable_ticks = 0

    while remaining() > 25:
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        position = nut["pos"]
        lateral = hypot(position[0] - goal[0], position[1] - goal[1])
        desired = [0.0, 0.0, sin(aligned_yaw / 2.0), cos(aligned_yaw / 2.0)]
        rotation = rotation_error(desired, nut["quat"])
        posture_error = sqrt(sum(value * value for value in rotation))

        if (
            position[2] <= resting_height + 0.010
            and lateral < 0.006
            and posture_error < 0.07
        ):
            stable_ticks += 1
            if stable_ticks >= 4:
                seated = True
                break
        else:
            stable_ticks = 0

        if state["gripper_width"] < 0.010:
            return fail("Grasp lost during insertion")

        target_x = goal[0]
        target_y = goal[1]

        if recovery_ticks > 0:
            target_height = recovery_height
            angle = recovery_count * pi / 2.0
            target_x += 0.0015 * cos(angle)
            target_y += 0.0015 * sin(angle)
            recovery_ticks -= 1
        elif lateral > 0.004 or posture_error > 0.065:
            target_height = position[2]
            stalled_ticks = 0
        else:
            target_height = max(resting_height + 0.002, position[2] - 0.030)
            if previous_height - position[2] < 0.00025:
                stalled_ticks += 1
            else:
                stalled_ticks = 0

            if stalled_ticks >= 18 and position[2] > resting_height + 0.012:
                recovery_height = position[2] + 0.009
                recovery_ticks = 9
                recovery_count += 1
                stalled_ticks = 0
                target_height = recovery_height

        previous_height = position[2]
        nut_command([target_x, target_y, target_height], aligned_yaw, 0.45)

    if not seated:
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        seated = (
            nut["pos"][2] <= resting_height + 0.010
            and hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.007
            and abs(square_error(nut["yaw"])) < 0.08
        )

    if not seated:
        return fail("Nut did not reach the seated height")

    actuate("open", 10)
    state = sdk.get_state()
    target = list(state["eef_pos"])
    target[2] += 0.080
    move_eef(target, state["eef_yaw"], -1.0, min(15, remaining()), 0.008)

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    center_error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    success = (
        center_error < 0.012
        and abs(nut["pos"][2] - resting_height) < 0.018
        and abs(square_error(nut["yaw"])) < 0.12
        and state["gripper_width"] > 0.04
    )
    return {
        "status": "done" if success else "failed",
        "reason": "Seated and released" if success else "Release verification failed",
        "center_error_m": center_error,
        "steps_used": state["steps_used"],
    }
