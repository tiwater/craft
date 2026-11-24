def vector_add(a, b):
    return [a[i] + b[i] for i in range(3)]


def vector_sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def vector_scale(v, scale):
    return [scale * value for value in v]


def vector_norm(v):
    return sqrt(sum(value * value for value in v))


def unit_vector(v, fallback):
    length = vector_norm(v)
    if length < 0.00001:
        return list(fallback)
    return vector_scale(v, 1.0 / length)


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(angle):
    return atan2(sin(angle), cos(angle))


def steps_remaining(state):
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def door_open(state):
    hinge = state["hinge_qpos"]
    return hinge is not None and hinge > 0.30


def handle_angle(state):
    value = state["handle_qpos"]
    return 0.0 if value is None else value


def opening_tangent(state):
    handle = state["objects"]["handle"]["pos"]
    door = state["objects"]["door"]["pos"]
    radial = [handle[0] - door[0], handle[1] - door[1], 0.0]
    if vector_norm(radial) < 0.025:
        yaw = state["objects"]["door"]["yaw"]
        radial = [cos(yaw), sin(yaw), 0.0]
    return unit_vector([-radial[1], radial[0], 0.0], [1.0, 0.0, 0.0])


def bounded_move(sdk, target, yaw, grip, limit, tolerance):
    state = sdk.get_state()
    budget = min(limit, steps_remaining(state))
    if budget > 0 and not door_open(state):
        sdk.move_to(
            target,
            yaw=yaw,
            gripper=grip,
            max_steps=budget,
            tol=tolerance,
        )
    return sdk.get_state()


def position_servo(sdk, target, yaw, grip=1.0, ticks=2):
    state = sdk.get_state()
    budget = min(ticks, steps_remaining(state))
    if budget <= 0 or door_open(state):
        return state

    # Robosuite's standard OSC_POSE translation and rotation scales.
    error = vector_sub(target, state["eef_pos"])
    translation = vector_scale(error, 1.0 / 0.05)
    magnitude = vector_norm(translation)
    if magnitude > 0.7:
        translation = vector_scale(translation, 0.7 / magnitude)
    rotation = clamp(wrap_angle(yaw - state["eef_yaw"]) / 0.5, -0.35, 0.35)
    sdk.step_raw(
        [translation[0], translation[1], translation[2],
         0.0, 0.0, rotation, grip],
        steps=budget,
    )
    return sdk.get_state()


def grasp_handle(sdk, attempt):
    state = sdk.get_state()
    tangent = opening_tangent(state)
    radial = [tangent[1], -tangent[0], 0.0]
    yaw = atan2(radial[1], radial[0])
    if attempt % 2:
        yaw += pi / 2.0

    handle = state["objects"]["handle"]["pos"]
    approach = vector_add(handle, [0.0, 0.0, 0.085])
    state = bounded_move(sdk, approach, yaw, "open", 28, 0.015)
    if door_open(state) or steps_remaining(state) == 0:
        return state

    handle = state["objects"]["handle"]["pos"]
    state = bounded_move(sdk, handle, yaw, "open", 26, 0.007)
    budget = min(12, steps_remaining(state))
    if budget > 0 and not door_open(state):
        sdk.set_gripper("close", steps=budget)
    return sdk.get_state()


def run_policy(sdk):
    state = sdk.get_state()
    if door_open(state):
        return {"status": "done", "hinge_qpos": state["hinge_qpos"]}

    attempts = 0
    while steps_remaining(state) > 18 and attempts < 3:
        state = grasp_handle(sdk, attempts)
        attempts += 1
        if door_open(state):
            break
        if steps_remaining(state) == 0:
            break

        handle = state["objects"]["handle"]["pos"]
        anchor = vector_sub(state["eef_pos"], handle)
        if vector_norm(anchor) > 0.055:
            continue

        initial_yaw = state["eef_yaw"]
        initial_hinge = state["hinge_qpos"]
        tangent = opening_tangent(state)
        radial = [tangent[1], -tangent[0], 0.0]

        # Probe latch motion from observed joint response. Downward pressure
        # is the first candidate; alternatives accommodate grasp variation.
        candidates = [
            [0.0, 0.0, -1.0],
            [0.0, 0.0, 1.0],
            radial,
            vector_scale(radial, -1.0),
        ]
        latch_direction = candidates[0]
        best_score = -1.0
        best_state = state

        for direction in candidates:
            if steps_remaining(state) < 16 or door_open(state):
                break
            for probe in range(5):
                handle = state["objects"]["handle"]["pos"]
                offset = vector_scale(direction, 0.010 + 0.005 * probe)
                target = vector_add(vector_add(handle, anchor), offset)
                state = position_servo(sdk, target, initial_yaw, ticks=2)
                if door_open(state):
                    break

            score = abs(handle_angle(state))
            score += 3.0 * max(0.0, state["hinge_qpos"] - initial_hinge)
            if score > best_score:
                best_score = score
                latch_direction = list(direction)
                best_state = state
            if abs(handle_angle(state)) > 0.22 or door_open(state):
                break

        if door_open(state):
            break

        best_hinge = state["hinge_qpos"]
        stagnant = 0
        empty_ticks = 0
        direction_sign = 1.0
        previous_handle = list(state["objects"]["handle"]["pos"])
        previous_hinge = state["hinge_qpos"]
        learned_tangent = None

        for iteration in range(95):
            if steps_remaining(state) <= 0 or door_open(state):
                break

            hinge = state["hinge_qpos"]
            handle = state["objects"]["handle"]["pos"]
            hinge_change = hinge - previous_hinge
            displacement = vector_sub(handle, previous_handle)

            # Measured handle travel refines the hinge tangent without
            # requiring the reported door origin to be its hinge center.
            if abs(hinge_change) > 0.004:
                horizontal = [displacement[0], displacement[1], 0.0]
                if vector_norm(horizontal) > 0.001:
                    learned_tangent = unit_vector(
                        vector_scale(horizontal, 1.0 / hinge_change),
                        opening_tangent(state),
                    )
                    direction_sign = 1.0

            tangent = opening_tangent(state)
            if learned_tangent is not None:
                tangent = learned_tangent

            previous_handle = list(handle)
            previous_hinge = hinge

            # Keep depressing the latch while pulling along the door arc.
            latch_pressure = 0.030 if hinge < 0.10 else 0.012
            pull_distance = 0.025 if hinge < 0.05 else 0.040
            offset = vector_add(
                vector_scale(latch_direction, latch_pressure),
                vector_scale(tangent, direction_sign * pull_distance),
            )
            target = vector_add(vector_add(handle, anchor), offset)
            yaw = initial_yaw + hinge - initial_hinge
            state = position_servo(sdk, target, yaw, ticks=2)

            if state["hinge_qpos"] > best_hinge + 0.006:
                best_hinge = state["hinge_qpos"]
                stagnant = 0
            else:
                stagnant += 1

            if state["gripper_width"] < 0.010:
                empty_ticks += 1
            else:
                empty_ticks = 0

            if empty_ticks >= 8:
                break

            if stagnant == 15 and learned_tangent is None:
                direction_sign = -direction_sign

            if stagnant == 25:
                # Reapply the strongest measured latch direction.
                latch_direction = list(candidates[0]) if best_score < 0.04 else latch_direction

            if stagnant >= 35:
                break

        if door_open(state):
            break

        # Release before retreating and attempting a different grasp.
        budget = min(7, steps_remaining(state))
        if budget > 0:
            sdk.set_gripper("open", steps=budget)
        state = sdk.get_state()

    return {
        "status": "done" if door_open(state) else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "steps_used": state["steps_used"],
        "grasp_attempts": attempts,
    }
