def door_remaining(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def door_clip(value, limit):
    return max(-limit, min(limit, value))


def door_latch(state):
    return 0.0 if state["handle_qpos"] is None else state["handle_qpos"]


def door_distance(a, b):
    return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def door_move(sdk, position, yaw, grip, ticks):
    ticks = min(ticks, door_remaining(sdk))
    if ticks > 0:
        sdk.move_to(
            position, yaw=yaw, gripper=grip,
            max_steps=ticks, tol=0.005
        )


def door_grip(sdk, grip, ticks):
    ticks = min(ticks, door_remaining(sdk))
    if ticks > 0:
        sdk.set_gripper(grip, steps=ticks)


def door_action(sdk, translation, rotation, ticks):
    ticks = min(ticks, door_remaining(sdk))
    if ticks > 0:
        sdk.step_raw(
            [door_clip(translation[i], 1.0) for i in range(3)]
            + [door_clip(rotation[i], 1.0) for i in range(3)]
            + [1.0],
            steps=ticks
        )


def door_direction(state):
    handle = state["objects"]["handle"]["pos"]
    goal = state["goal_pos"]
    if goal is not None:
        dx = goal[0] - handle[0]
        dy = goal[1] - handle[1]
    else:
        yaw = state["objects"]["door"]["yaw"]
        dx = -sin(yaw)
        dy = -cos(yaw)
    length = max(hypot(dx, dy), 0.0001)
    return [dx / length, dy / length]


def run_policy(sdk):
    state = sdk.get_state()
    best_hinge = state["hinge_qpos"]
    best_latch = abs(door_latch(state))
    attempt = 0

    # The observed partial turns are not reliable evidence of release.
    # Establish a larger latch angle before applying substantial opening force.
    while door_remaining(sdk) > 18:
        state = sdk.get_state()
        if state["hinge_qpos"] > 0.30:
            break

        handle = list(state["objects"]["handle"]["pos"])
        yaw = state["objects"]["door"]["yaw"]
        yaw += pi / 2.0 if attempt % 2 == 0 else 0.0

        door_grip(sdk, "open", 4)
        state = sdk.get_state()
        door_move(
            sdk,
            [
                state["eef_pos"][0],
                state["eef_pos"][1],
                max(state["eef_pos"][2], handle[2] + 0.065)
            ],
            None, "open", 10
        )
        door_move(
            sdk, [handle[0], handle[1], handle[2] + 0.055],
            yaw, "open", 24
        )
        handle = list(sdk.get_state()["objects"]["handle"]["pos"])
        door_move(sdk, handle, yaw, "open", 22)
        door_grip(sdk, "close", 10)

        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        if door_distance(state["eef_pos"], handle) > 0.055:
            attempt += 1
            continue

        offset = [
            door_clip(state["eef_pos"][i] - handle[i], 0.018)
            for i in range(3)
        ]
        direction = door_direction(state)
        yaw = state["objects"]["door"]["yaw"]
        axes = [
            [cos(yaw), sin(yaw), 0.0],
            [-sin(yaw), cos(yaw), 0.0],
            [0.0, 0.0, 1.0]
        ]
        rotation_axis = list(axes[0])
        response_best = 0.0
        local_steps = 0
        peak = abs(door_latch(state))
        stagnant = 0
        pull_stagnant = 0
        previous_hinge = state["hinge_qpos"]
        previous_handle = list(handle)
        released = state["hinge_qpos"] > 0.045
        probe = 0
        lost = 0

        while door_remaining(sdk) > 0 and local_steps < 245:
            state = sdk.get_state()
            hinge = state["hinge_qpos"]
            latch = door_latch(state)
            magnitude = abs(latch)
            handle = list(state["objects"]["handle"]["pos"])
            eef = state["eef_pos"]
            best_hinge = max(best_hinge, hinge)
            best_latch = max(best_latch, magnitude)
            if hinge > 0.30:
                break

            distance = door_distance(eef, handle)
            if distance > 0.09:
                break
            if state["gripper_width"] < 0.009 and distance > 0.035:
                lost += 1
            else:
                lost = 0
            if lost >= 4:
                break

            progress = hinge - previous_hinge
            if progress > 0.002:
                dx = handle[0] - previous_handle[0]
                dy = handle[1] - previous_handle[1]
                travel = hypot(dx, dy)
                if travel > 0.001:
                    measured = [dx / travel, dy / travel]
                    mixed = [
                        0.65 * direction[i] + 0.35 * measured[i]
                        for i in range(2)
                    ]
                    length = max(hypot(mixed[0], mixed[1]), 0.0001)
                    direction = [mixed[0] / length, mixed[1] / length]
                pull_stagnant = 0
            else:
                pull_stagnant += 1
            previous_hinge = hinge
            previous_handle = list(handle)

            if magnitude > peak + 0.018:
                peak = magnitude
                stagnant = 0
            else:
                stagnant += 1

            if magnitude >= 0.48 or hinge > 0.045:
                released = True
            if hinge < 0.04 and magnitude < 0.34:
                released = False

            # Identify which wrist rotation actually turns the live handle.
            # Favor increasing the existing angle, including negative angles.
            probing = local_steps < 36 and magnitude < 0.46
            sign = -1.0 if latch < -0.06 else 1.0
            if probing:
                probe = local_steps // 6
                axis = axes[min(2, probe // 2)]
                polarity = 1.0 if probe % 2 == 0 else -1.0
                rotation = [0.5 * polarity * axis[i] for i in range(3)]
            else:
                strength = door_clip(1.6 * (0.62 - magnitude), 0.55)
                rotation = [
                    sign * strength * rotation_axis[i] for i in range(3)
                ]

            correction = [
                door_clip(12.0 * (handle[i] + offset[i] - eef[i]), 0.35)
                for i in range(3)
            ]
            pull = 0.04
            if released:
                pull = 0.48 if hinge < 0.06 else 0.7
            elif magnitude > 0.38:
                pull = 0.13

            translation = [
                correction[0] + pull * direction[0],
                correction[1] + pull * direction[1],
                correction[2]
            ]
            if not released:
                translation[2] -= 0.10 if magnitude < 0.40 else 0.04

            before = latch
            ticks = min(3, door_remaining(sdk))
            door_action(sdk, translation, rotation, ticks)
            local_steps += ticks
            after = sdk.get_state()
            change = door_latch(after) - before
            best_hinge = max(best_hinge, after["hinge_qpos"])
            best_latch = max(best_latch, abs(door_latch(after)))

            if probing:
                response = abs(change)
                if response > response_best and response > 0.008:
                    response_best = response
                    response_sign = 1.0 if change * polarity > 0.0 else -1.0
                    rotation_axis = [
                        response_sign * axis[i] for i in range(3)
                    ]

            if after["hinge_qpos"] > 0.30:
                break

            if not probing and not released and stagnant >= 14:
                # Test an alternative engagement without abandoning the grasp.
                axis_index = (1 + attempt + local_steps // 42) % 3
                rotation_axis = list(axes[axis_index])
                if (local_steps // 42) % 2:
                    rotation_axis = [-value for value in rotation_axis]
                stagnant = 0

            if released and pull_stagnant >= 18:
                # Probe neighboring opening directions while holding the latch.
                base = door_direction(after)
                angle = 0.65 if (local_steps // 54) % 2 == 0 else -0.65
                direction = [
                    cos(angle) * base[0] - sin(angle) * base[1],
                    sin(angle) * base[0] + cos(angle) * base[1]
                ]
                pull_stagnant = 0

            if local_steps >= 105 and peak < 0.33 and hinge < 0.025:
                break

        attempt += 1

    state = sdk.get_state()
    return {
        "status": "done" if state["hinge_qpos"] > 0.30 else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "best_hinge_qpos": max(best_hinge, state["hinge_qpos"]),
        "best_abs_handle_qpos": max(best_latch, abs(door_latch(state))),
        "steps_used": state["steps_used"]
    }
