def clamp(value, low, high):
    return max(low, min(high, value))


def remaining(state):
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def distance(a, b):
    return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def quat_product(a, b):
    return [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]


def turn_quat(q, axis, angle):
    scale = sin(angle / 2.0)
    return quat_product(
        [axis[0] * scale, axis[1] * scale, axis[2] * scale,
         cos(angle / 2.0)],
        q,
    )


def orientation_action(target, current):
    error = quat_product(
        target, [-current[0], -current[1], -current[2], current[3]]
    )
    sign = 1.0 if error[3] >= 0.0 else -1.0
    return [clamp(5.0 * sign * error[i], -0.75, 0.75)
            for i in range(3)]


def rotate_xy(vector, angle):
    return [
        cos(angle) * vector[0] - sin(angle) * vector[1],
        sin(angle) * vector[0] + cos(angle) * vector[1],
        vector[2],
    ]


def unit_xy(vector):
    length = hypot(vector[0], vector[1])
    if length < 0.000001:
        return [0.0, 1.0, 0.0]
    return [vector[0] / length, vector[1] / length, 0.0]


def servo(sdk, state, target, orientation, grip):
    if remaining(state) == 0:
        return state
    translation = [
        clamp(30.0 * (target[i] - state["eef_pos"][i]), -0.9, 0.9)
        for i in range(3)
    ]
    sdk.step_raw(
        translation + orientation_action(orientation, state["eef_quat"])
        + [grip],
        steps=1,
    )
    return sdk.get_state()


def run_policy(sdk):
    state = sdk.get_state()
    best_hinge = state["hinge_qpos"]
    best_handle = state["handle_qpos"]
    attempts = 0

    # Endpoint logs show both missed grasps and partially opened doors.
    # Reserve most of the horizon for sustained, feedback-controlled opening.
    while remaining(state) > 20 and state["hinge_qpos"] <= 0.30:
        attempts += 1
        handle = list(state["objects"]["handle"]["pos"])
        yaw = state["objects"]["handle"]["yaw"]
        yaw += pi / 2.0 if attempts % 2 == 1 else 0.0

        sdk.move_to(
            [handle[0], handle[1], handle[2] + 0.075],
            yaw=yaw,
            gripper="open",
            max_steps=min(28, remaining(state)),
            tol=0.014,
        )
        state = sdk.get_state()
        if remaining(state) == 0:
            break

        handle = list(state["objects"]["handle"]["pos"])
        sdk.move_to(
            handle,
            yaw=yaw,
            gripper="open",
            max_steps=min(32, remaining(state)),
            tol=0.009,
        )
        state = sdk.get_state()
        orientation = list(state["eef_quat"])

        # Do not mistake an approach blocked above the lever for a grasp.
        for tick in range(18):
            if remaining(state) == 0:
                break
            handle = state["objects"]["handle"]["pos"]
            if distance(state["eef_pos"], handle) < 0.025:
                break
            target = list(handle)
            if tick < 9:
                target[0] += 0.014 if attempts % 2 == 1 else -0.014
            state = servo(sdk, state, target, orientation, -1.0)

        if remaining(state) < 8:
            break
        if distance(state["eef_pos"], state["objects"]["handle"]["pos"]) > 0.05:
            continue

        sdk.set_gripper("close", steps=min(9, remaining(state)))
        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        orientation = list(state["eef_quat"])
        offset = [
            clamp(state["eef_pos"][i] - handle[i], -0.025, 0.025)
            for i in range(3)
        ]

        axis = None
        latch_start_step = state["steps_used"]
        latch_best = state["handle_qpos"]
        latch_stagnant = 0
        directions = [
            [1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
        ]
        probe = 0
        probe_tick = 0
        probe_start = state["handle_qpos"]
        probe_orientation = list(orientation)
        best_orientation = list(orientation)

        while remaining(state) > 35:
            if state["hinge_qpos"] > 0.30:
                break
            elapsed = state["steps_used"] - latch_start_step
            if state["handle_qpos"] >= 0.65 or elapsed >= 75:
                break

            handle = state["objects"]["handle"]["pos"]
            target = [handle[i] + offset[i] for i in range(3)]
            target[2] -= 0.025

            # First follow the lever's downward arc. Probe wrist axes only
            # when translation alone stops producing latch motion.
            if axis is not None:
                if state["handle_qpos"] < 0.60:
                    orientation = turn_quat(orientation, axis, 0.025)
            elif elapsed >= 14 and latch_stagnant >= 6 and probe < 4:
                if probe_tick == 0:
                    probe_orientation = list(best_orientation)
                    probe_start = state["handle_qpos"]
                probe_tick += 1
                orientation = turn_quat(
                    probe_orientation, directions[probe], 0.05 * probe_tick
                )

            state = servo(sdk, state, target, orientation, 1.0)
            best_hinge = max(best_hinge, state["hinge_qpos"])
            best_handle = max(best_handle, state["handle_qpos"])

            if state["handle_qpos"] > latch_best + 0.008:
                latch_best = state["handle_qpos"]
                best_orientation = list(state["eef_quat"])
                latch_stagnant = 0
            else:
                latch_stagnant += 1

            if probe_tick >= 9:
                if state["handle_qpos"] > probe_start + 0.045:
                    axis = list(directions[probe])
                    orientation = list(state["eef_quat"])
                else:
                    orientation = list(best_orientation)
                    probe += 1
                probe_tick = 0

            if distance(state["eef_pos"], state["objects"]["handle"]["pos"]) > 0.085:
                break

        if state["hinge_qpos"] > 0.30:
            break

        # A small handle angle is insufficient evidence of a secure grasp.
        if state["handle_qpos"] < 0.28 and state["hinge_qpos"] < 0.025:
            if remaining(state) > 100:
                continue

        orientation = list(state["eef_quat"])
        door = state["objects"]["door"]["pos"]
        handle = state["objects"]["handle"]["pos"]
        radial = unit_xy([handle[i] - door[i] for i in range(3)])

        # The supplied goal is not a measured hinge tangent. Test forces
        # against actual hinge progress instead of committing to that goal.
        candidates = [
            [radial[1], -radial[0], 0.0],
            [-radial[1], radial[0], 0.0],
            radial,
            [-radial[0], -radial[1], 0.0],
        ]
        direction = list(candidates[0])
        candidate = 0
        probing = True
        probe_ticks = 0
        probe_hinge = state["hinge_qpos"]
        scores = []
        reference_hinge = state["hinge_qpos"]
        previous_hinge = state["hinge_qpos"]
        progress_hinge = state["hinge_qpos"]
        stagnant = 0
        opening_ticks = 0
        wrist_turn = 0.0

        while remaining(state) > 0 and state["hinge_qpos"] <= 0.30:
            opening_ticks += 1
            handle = state["objects"]["handle"]["pos"]
            lead = 0.035 if probing else 0.055
            target = [
                handle[i] + offset[i] + lead * direction[i]
                for i in range(3)
            ]
            target[2] -= 0.025 if state["handle_qpos"] < 0.65 else 0.008

            if axis is not None and state["handle_qpos"] < 0.50 and wrist_turn < 0.65:
                orientation = turn_quat(orientation, axis, 0.018)
                wrist_turn += 0.018

            state = servo(sdk, state, target, orientation, 1.0)
            change = clamp(state["hinge_qpos"] - previous_hinge, -0.08, 0.08)
            previous_hinge = state["hinge_qpos"]
            orientation = turn_quat(orientation, [0.0, 0.0, 1.0], change)
            offset = rotate_xy(offset, change)
            direction = rotate_xy(direction, change)
            if axis is not None:
                axis = rotate_xy(axis, change)

            best_hinge = max(best_hinge, state["hinge_qpos"])
            best_handle = max(best_handle, state["handle_qpos"])

            if state["hinge_qpos"] > progress_hinge + 0.002:
                progress_hinge = state["hinge_qpos"]
                stagnant = 0
            else:
                stagnant += 1

            if probing:
                probe_ticks += 1
                gain = state["hinge_qpos"] - probe_hinge
                if gain > 0.012:
                    probing = False
                    stagnant = 0
                elif probe_ticks >= 12:
                    scores.append(gain)
                    candidate += 1
                    if candidate < len(candidates):
                        direction = rotate_xy(
                            candidates[candidate],
                            state["hinge_qpos"] - reference_hinge,
                        )
                        probe_hinge = state["hinge_qpos"]
                        probe_ticks = 0
                    else:
                        selected = 0
                        for index in range(1, len(scores)):
                            if scores[index] > scores[selected]:
                                selected = index
                        candidate = selected
                        direction = rotate_xy(
                            candidates[selected],
                            state["hinge_qpos"] - reference_hinge,
                        )
                        probing = False
                        stagnant = 0
            elif stagnant >= 24:
                candidate = (candidate + 1) % len(candidates)
                direction = rotate_xy(
                    candidates[candidate],
                    state["hinge_qpos"] - reference_hinge,
                )
                probe_hinge = state["hinge_qpos"]
                probe_ticks = 0
                stagnant = 0

            separation = distance(
                state["eef_pos"], state["objects"]["handle"]["pos"]
            )
            lost = separation > 0.09
            empty = state["gripper_width"] < 0.012
            if remaining(state) > 100:
                if lost or (
                    opening_ticks > 48
                    and empty
                    and state["handle_qpos"] < 0.20
                    and state["hinge_qpos"] < 0.04
                ):
                    break

    state = sdk.get_state()
    return {
        "status": "done" if state["hinge_qpos"] > 0.30 else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "best_hinge_qpos": max(best_hinge, state["hinge_qpos"]),
        "best_handle_qpos": max(best_handle, state["handle_qpos"]),
        "steps_used": state["steps_used"],
    }
