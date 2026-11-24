def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def finished(state):
        return state["hinge_qpos"] is not None and state["hinge_qpos"] > 0.30

    def result():
        state = sdk.get_state()
        return {
            "status": "done" if finished(state) else "failed",
            "hinge_qpos": state["hinge_qpos"],
            "handle_qpos": state["handle_qpos"],
            "steps_used": state["steps_used"],
        }

    def servo(target, rotation, ticks):
        for tick in range(min(ticks, remaining())):
            state = sdk.get_state()
            if finished(state):
                return
            action = [
                clamp((target[i] - state["eef_pos"][i]) / 0.05, 0.55)
                for i in range(3)
            ]
            sdk.step_raw(action + list(rotation) + [1.0], steps=1)

    def press_target(state, offset, floor, depth):
        position = state["objects"]["handle"]["pos"]
        return [
            position[0] + offset[0],
            position[1] + offset[1],
            max(floor, position[2] + offset[2] - depth),
        ]

    state = sdk.get_state()
    if finished(state):
        return result()

    shifts = [0.0, 0.015, -0.015]
    twist = [0.0, 0.0, 0.0]

    for attempt in range(3):
        if remaining() < 35:
            break

        state = sdk.get_state()
        if finished(state):
            return result()

        if attempt > 0:
            sdk.set_gripper("open", steps=min(4, remaining()))
            sdk.move_delta(
                [0.0, 0.025, 0.035],
                gripper="open",
                steps=min(7, remaining()),
            )
        else:
            sdk.set_gripper("open", steps=min(6, remaining()))

        state = sdk.get_state()
        handle = state["objects"]["handle"]
        yaw = handle["yaw"]
        position = list(handle["pos"])
        sdk.move_to(
            [position[0] + shifts[attempt],
             position[1] + 0.015,
             position[2] + 0.08],
            yaw=yaw,
            gripper="open",
            max_steps=min(30 if attempt == 0 else 18, remaining()),
            tol=0.015,
        )

        state = sdk.get_state()
        position = list(state["objects"]["handle"]["pos"])
        sdk.move_to(
            [position[0] + shifts[attempt],
             position[1] + 0.005,
             position[2] + 0.005],
            yaw=yaw,
            gripper="open",
            max_steps=min(25 if attempt == 0 else 18, remaining()),
            tol=0.008,
        )
        if remaining() < 10:
            return result()

        sdk.set_gripper("close", steps=min(8, remaining()))
        state = sdk.get_state()
        offset = [
            clamp(state["eef_pos"][i] - state["objects"]["handle"]["pos"][i], 0.025)
            for i in range(3)
        ]
        offset[1] = clamp(offset[1], 0.012)
        offset[2] = clamp(offset[2], 0.012)
        floor = state["objects"]["handle"]["pos"][2] - 0.12

        # Keep the baseline's measured rotational assistance, but do not
        # mistake a partial latch movement for release.
        if attempt == 0:
            best_response = 0.0
            for axis in range(3):
                if remaining() < 40:
                    break
                state = sdk.get_state()
                if finished(state):
                    return result()
                if state["handle_qpos"] >= 1.15:
                    break

                target = [
                    state["objects"]["handle"]["pos"][i] + offset[i]
                    for i in range(3)
                ]
                before = state["handle_qpos"]
                rotation = [0.0, 0.0, 0.0]
                rotation[axis] = 0.16
                servo(target, rotation, 4)
                response = sdk.get_state()["handle_qpos"] - before
                if abs(response) > best_response:
                    best_response = abs(response)
                    twist = [0.0, 0.0, 0.0]
                    twist[axis] = 0.13 if response >= 0.0 else -0.13
                servo(target, [-value for value in rotation], 4)

        best_latch = sdk.get_state()["handle_qpos"]
        stalled = 0
        released = False

        # Ramp downward pressure to 10 cm, bounded by the engagement floor.
        # Re-read the moving handle every tick instead of pressing an old pose.
        for tick in range(min(44, remaining())):
            state = sdk.get_state()
            if finished(state):
                return result()
            if state["handle_qpos"] >= 1.15:
                released = True
                break

            depth = min(0.10, 0.02 + 0.004 * tick)
            target = press_target(state, offset, floor, depth)
            servo(target, twist, 1)

            state = sdk.get_state()
            if state["handle_qpos"] > best_latch + 0.015:
                best_latch = state["handle_qpos"]
                stalled = 0
            else:
                stalled += 1

            distance = sqrt(sum(
                (state["eef_pos"][i] - state["objects"]["handle"]["pos"][i]) ** 2
                for i in range(3)
            ))
            if tick >= 20 and stalled >= 16 and state["handle_qpos"] < 1.0:
                break
            if distance > 0.15:
                break

        state = sdk.get_state()
        if finished(state):
            return result()
        released = released or state["handle_qpos"] >= 1.15
        if not released:
            continue

        direction = [0.6, 0.8, 0.0]
        stagnant = 0
        lost = 0
        weak_latch = 0
        best_gain = -1000.0
        best_direction = list(direction)
        candidates = [
            [0.6, 0.8, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ]
        probe = 0

        for segment in range(40):
            if remaining() <= 0:
                return result()
            state = sdk.get_state()
            if finished(state):
                return result()

            before_angle = state["hinge_qpos"]
            before_handle = list(state["objects"]["handle"]["pos"])

            for tick in range(min(5, remaining())):
                state = sdk.get_state()
                if finished(state):
                    return result()

                latch = state["handle_qpos"]
                depth = 0.10 if latch < 1.15 else 0.075
                target = press_target(state, offset, floor, depth)

                # Restore latch pressure before adding further opening force.
                pull = 0.045
                if latch < 1.0 and state["hinge_qpos"] < 0.06:
                    pull = 0.0
                target[0] += pull * direction[0]
                target[1] += pull * direction[1]
                rotation = twist if latch < 1.15 else [0.0, 0.0, 0.0]
                servo(target, rotation, 1)

            state = sdk.get_state()
            if finished(state):
                return result()

            gain = state["hinge_qpos"] - before_angle
            displacement = [
                state["objects"]["handle"]["pos"][i] - before_handle[i]
                for i in range(3)
            ]
            planar_motion = hypot(displacement[0], displacement[1])

            if gain > best_gain:
                best_gain = gain
                best_direction = list(direction)

            if gain > 0.001:
                stagnant = 0
                if planar_motion > 0.001:
                    tangent = [
                        displacement[0] / planar_motion,
                        displacement[1] / planar_motion,
                        0.0,
                    ]
                    direction = [
                        0.4 * direction[i] + 0.6 * tangent[i]
                        for i in range(3)
                    ]
                    magnitude = hypot(direction[0], direction[1])
                    if magnitude > 0.001:
                        direction = [value / magnitude for value in direction]
            else:
                stagnant += 1
                if state["handle_qpos"] >= 1.0 and probe < len(candidates) - 1:
                    probe += 1
                    direction = list(candidates[probe])
                elif best_gain > 0.001:
                    direction = list(best_direction)

            distance = sqrt(sum(
                (state["eef_pos"][i] - state["objects"]["handle"]["pos"][i]) ** 2
                for i in range(3)
            ))
            lost = lost + 1 if distance > 0.13 else 0
            weak_latch = weak_latch + 1 if state["handle_qpos"] < 1.0 else 0

            if lost >= 2 or stagnant >= 5:
                break
            if weak_latch >= 3 and gain <= 0.001:
                break

    return result()
