def clamp(value, low, high):
    return max(low, min(high, value))


def quat_product(a, b):
    return [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]


def turn_quat(axis, angle, orientation):
    scale = sin(0.5 * angle)
    return quat_product(
        [axis[0] * scale, axis[1] * scale, axis[2] * scale,
         cos(0.5 * angle)],
        orientation,
    )


def remaining(state):
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def control(sdk, state, target, orientation, grip, speed=0.8):
    current = state["eef_quat"]
    error = quat_product(
        orientation, [-current[0], -current[1], -current[2], current[3]]
    )
    sign = 1.0 if error[3] >= 0.0 else -1.0
    action = [
        clamp(45.0 * (target[i] - state["eef_pos"][i]), -speed, speed)
        for i in range(3)
    ]
    action += [
        clamp(12.0 * sign * error[i], -1.0, 1.0)
        for i in range(3)
    ]
    sdk.step_raw(action + [grip], steps=1)
    return sdk.get_state()


def run_policy(sdk):
    state = sdk.get_state()
    attempt = 0

    while remaining(state) > 25 and state["hinge_qpos"] <= 0.30:
        handle = list(state["objects"]["handle"]["pos"])
        door = state["objects"]["door"]
        yaw = door["yaw"]
        normal = [-sin(yaw), cos(yaw), 0.0]
        if sum((handle[i] - door["pos"][i]) * normal[i]
               for i in range(3)) < 0.0:
            normal = [-value for value in normal]

        facing = atan2(normal[1], normal[0]) - pi / 2.0
        orientation = quat_product(
            [0.0, 0.0, sin(facing / 2.0), cos(facing / 2.0)],
            [sin(pi / 4.0), 0.0, 0.0, cos(pi / 4.0)],
        )
        if attempt % 2:
            orientation = turn_quat(normal, pi / 2.0, orientation)

        for distance, height, ticks in [
            (0.12, 0.065, 42),
            (0.075, 0.0, 22),
            (0.002, 0.0, 36),
        ]:
            for tick in range(min(ticks, remaining(state))):
                handle = state["objects"]["handle"]["pos"]
                target = [
                    handle[0] + distance * normal[0],
                    handle[1] + distance * normal[1],
                    handle[2] + height,
                ]
                state = control(sdk, state, target, orientation, -1.0)
                error = sum(
                    (target[i] - state["eef_pos"][i]) ** 2
                    for i in range(3)
                )
                if error < 0.000064:
                    break

        for tick in range(min(16, remaining(state))):
            handle = state["objects"]["handle"]["pos"]
            target = [handle[i] + 0.002 * normal[i] for i in range(3)]
            state = control(sdk, state, target, orientation, 1.0, 0.5)

        handle = list(state["objects"]["handle"]["pos"])
        separation = sqrt(sum(
            (state["eef_pos"][i] - handle[i]) ** 2 for i in range(3)
        ))
        if separation > 0.055:
            attempt += 1
            continue

        offset = [
            state["eef_pos"][i] - handle[i] for i in range(3)
        ]
        base_orientation = list(state["eef_quat"])
        base_hinge = state["hinge_qpos"]
        base_latch = state["handle_qpos"]
        twist = 0.0
        direction = 1.0 if attempt % 2 == 0 else -1.0
        best_latch = abs(base_latch)
        best_hinge = state["hinge_qpos"]
        stagnant = 0
        reversed_once = False
        opening_direction = [-normal[0], -normal[1], 0.0]
        previous_handle = list(handle)
        previous_hinge = state["hinge_qpos"]
        unlocked = state["hinge_qpos"] > 0.04

        for tick in range(min(245, remaining(state))):
            if state["hinge_qpos"] > 0.30:
                break

            handle = list(state["objects"]["handle"]["pos"])
            hinge = state["hinge_qpos"]
            latch = abs(state["handle_qpos"])
            if hinge > 0.04:
                unlocked = True

            if not unlocked:
                twist = clamp(twist + direction * 0.045, -2.1, 2.1)

            hinge_change = hinge - base_hinge
            c = cos(hinge_change)
            s = sin(hinge_change)
            live_normal = [
                c * normal[0] - s * normal[1],
                s * normal[0] + c * normal[1],
                0.0,
            ]
            desired = turn_quat(normal, twist, base_orientation)
            desired = turn_quat([0.0, 0.0, 1.0], hinge_change, desired)

            if not unlocked:
                opening_direction = [
                    -live_normal[0], -live_normal[1], 0.0
                ]

            lead = 0.07 if unlocked else (
                0.045 if latch > 0.22 else 0.014
            )
            target = [
                handle[0] + c * offset[0] - s * offset[1]
                + lead * opening_direction[0],
                handle[1] + s * offset[0] + c * offset[1]
                + lead * opening_direction[1],
                handle[2] + offset[2],
            ]
            state = control(sdk, state, target, desired, 1.0)

            new_handle = state["objects"]["handle"]["pos"]
            new_hinge = state["hinge_qpos"]
            new_latch = abs(state["handle_qpos"])
            dx = new_handle[0] - previous_handle[0]
            dy = new_handle[1] - previous_handle[1]
            distance = hypot(dx, dy)

            if new_hinge - previous_hinge > 0.001 and distance > 0.0005:
                mx = 0.75 * opening_direction[0] + 0.25 * dx / distance
                my = 0.75 * opening_direction[1] + 0.25 * dy / distance
                length = max(hypot(mx, my), 0.000001)
                opening_direction = [mx / length, my / length, 0.0]

            previous_handle = list(new_handle)
            previous_hinge = new_hinge

            if new_latch > best_latch + 0.008 or new_hinge > best_hinge + 0.002:
                best_latch = max(best_latch, new_latch)
                best_hinge = max(best_hinge, new_hinge)
                stagnant = 0
            else:
                stagnant += 1

            if not unlocked and stagnant >= 38 and not reversed_once:
                direction = -direction
                base_orientation = list(state["eef_quat"])
                base_hinge = new_hinge
                twist = 0.0
                reversed_once = True
                stagnant = 0

            separation = sqrt(sum(
                (state["eef_pos"][i] - new_handle[i]) ** 2
                for i in range(3)
            ))
            if separation > 0.12 or stagnant > 85:
                break

        attempt += 1

    return {
        "status": "done" if state["hinge_qpos"] > 0.30 else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "steps_used": state["steps_used"],
    }
