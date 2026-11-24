def clamp(value, low, high):
    return max(low, min(high, value))


def quat_multiply(a, b):
    return [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]


def axis_rotation(axis, angle):
    scale = sin(angle * 0.5)
    return [axis[0] * scale, axis[1] * scale,
            axis[2] * scale, cos(angle * 0.5)]


def rotate_vector(quat, vector):
    result = quat_multiply(
        quat_multiply(quat, [vector[0], vector[1], vector[2], 0.0]),
        [-quat[0], -quat[1], -quat[2], quat[3]],
    )
    return result[:3]


def servo(sdk, state, position, orientation):
    current = state["eef_quat"]
    error = quat_multiply(
        orientation, [-current[0], -current[1], -current[2], current[3]]
    )
    sign = 1.0 if error[3] >= 0.0 else -1.0
    action = [
        clamp((position[i] - state["eef_pos"][i]) / 0.05, -0.7, 0.7)
        for i in range(3)
    ]
    action += [clamp(4.0 * sign * error[i], -0.6, 0.6)
               for i in range(3)]
    sdk.step_raw(action + [1.0], steps=1)


def run_policy(sdk):
    state = sdk.get_state()
    limit = min(state["max_steps"], 400)

    for attempt in range(2):
        state = sdk.get_state()
        if state["hinge_qpos"] > 0.30:
            return {"status": "done", "hinge_qpos": state["hinge_qpos"]}
        if limit - state["steps_used"] < 65:
            break

        handle = state["objects"]["handle"]["pos"]
        door_quat = state["objects"]["door"]["quat"]
        normal = rotate_vector(door_quat, [0.0, 1.0, 0.0])
        yaw = atan2(normal[1], normal[0]) + attempt * pi * 0.5

        sdk.move_to(
            [handle[0], handle[1], handle[2] + 0.12],
            yaw=yaw, gripper="open",
            max_steps=min(35, limit - state["steps_used"]), tol=0.018,
        )
        state = sdk.get_state()
        handle = state["objects"]["handle"]["pos"]
        sdk.move_to(
            list(handle), yaw=yaw, gripper="open",
            max_steps=min(30, limit - state["steps_used"]), tol=0.007,
        )
        state = sdk.get_state()
        sdk.set_gripper("close", steps=min(12, limit - state["steps_used"]))
        state = sdk.get_state()

        offset = [
            clamp(state["eef_pos"][i] - state["objects"]["handle"]["pos"][i],
                  -0.015, 0.015)
            for i in range(3)
        ]
        orientation = list(state["eef_quat"])
        best_orientation = list(orientation)
        best_latch = abs(state["handle_qpos"])
        axes = [
            normal,
            rotate_vector(door_quat, [1.0, 0.0, 0.0]),
            rotate_vector(door_quat, [0.0, 0.0, 1.0]),
        ]

        # Identify a productive wrist rotation using measured latch motion.
        latch_budget = min(105, max(0, limit - state["steps_used"] - 85))
        latch_start = state["steps_used"]
        for axis in axes:
            if best_latch > 0.32:
                break
            for direction in [1.0, -1.0]:
                orientation = list(best_orientation)
                stale = 0
                for increment in range(9):
                    state = sdk.get_state()
                    if (state["steps_used"] - latch_start >= latch_budget
                            or abs(state["handle_qpos"]) > 0.32):
                        break
                    orientation = quat_multiply(
                        axis_rotation(axis, direction * 0.14), orientation
                    )
                    previous = abs(state["handle_qpos"])
                    for tick in range(5):
                        state = sdk.get_state()
                        if state["steps_used"] - latch_start >= latch_budget:
                            break
                        handle = state["objects"]["handle"]["pos"]
                        servo(sdk, state,
                              [handle[i] + offset[i] for i in range(3)],
                              orientation)
                    state = sdk.get_state()
                    value = abs(state["handle_qpos"])
                    if value > best_latch:
                        best_latch = value
                        best_orientation = list(state["eef_quat"])
                    stale = stale + 1 if value < previous + 0.012 else 0
                    if stale >= 2:
                        break
                if best_latch > 0.32:
                    break

        state = sdk.get_state()
        orientation = list(best_orientation)
        previous_hinge = state["hinge_qpos"]
        best_hinge = previous_hinge
        stalled = 0

        # Follow the live handle around the door's vertical hinge.
        for tick in range(175):
            state = sdk.get_state()
            hinge = state["hinge_qpos"]
            if hinge > 0.30:
                return {"status": "done", "hinge_qpos": hinge,
                        "steps_used": state["steps_used"]}
            if state["steps_used"] >= limit:
                break

            orientation = quat_multiply(
                axis_rotation([0.0, 0.0, 1.0], hinge - previous_hinge),
                orientation,
            )
            previous_hinge = hinge
            handle = state["objects"]["handle"]["pos"]
            door = state["objects"]["door"]["pos"]
            radius = [handle[0] - door[0], handle[1] - door[1]]
            length = hypot(radius[0], radius[1])
            if length > 0.04:
                tangent = [-radius[1] / length, radius[0] / length, 0.0]
            else:
                tangent = rotate_vector(
                    state["objects"]["door"]["quat"], [0.0, -1.0, 0.0]
                )

            lead = 0.025 if hinge < 0.06 else 0.04
            target = [handle[i] + offset[i] + lead * tangent[i]
                      for i in range(3)]

            # Maintain latch actuation until the door clears its stop.
            if hinge < 0.04 and abs(state["handle_qpos"]) < 0.28:
                target[2] -= 0.008

            servo(sdk, state, target, orientation)
            if hinge > best_hinge + 0.003:
                best_hinge = hinge
                stalled = 0
            else:
                stalled += 1
            if stalled > 45:
                break

    state = sdk.get_state()
    return {
        "status": "done" if state["hinge_qpos"] > 0.30 else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "steps_used": state["steps_used"],
    }
