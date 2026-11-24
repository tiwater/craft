def run_policy(sdk):
    def clamp(value, low, high):
        return max(low, min(high, value))

    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def success(state):
        return state["hinge_qpos"] is not None and state["hinge_qpos"] > 0.30

    def result(state):
        return {
            "status": "done" if success(state) else "failed",
            "hinge_qpos": state["hinge_qpos"],
            "handle_qpos": state["handle_qpos"],
            "steps_used": state["steps_used"],
        }

    def move(target, ticks, yaw, gripper="open", tolerance=0.006):
        budget = min(ticks, remaining())
        if budget > 0:
            return sdk.move_to(
                target,
                yaw=yaw,
                gripper=gripper,
                max_steps=budget,
                tol=tolerance,
            )
        return None

    def track(state, target, limit=0.8):
        if remaining() > 0:
            translation = [
                clamp((target[i] - state["eef_pos"][i]) / 0.05, -limit, limit)
                for i in range(3)
            ]
            sdk.step_raw(translation + [0.0, 0.0, 0.0, 1.0], steps=1)

    state = sdk.get_state()
    if success(state):
        return result(state)

    for attempt in range(3):
        if remaining() < 45:
            break

        state = sdk.get_state()
        if success(state):
            return result(state)

        lateral = [0.0, 0.015, -0.015][attempt]
        front = [0.008, 0.002, 0.014][attempt]
        yaw = state["objects"]["handle"]["yaw"] + pi / 2.0

        sdk.set_gripper("open", steps=min(8, remaining()))

        if attempt > 0:
            state = sdk.get_state()
            move(
                [
                    state["eef_pos"][0],
                    state["eef_pos"][1] + 0.035,
                    state["eef_pos"][2] + 0.040,
                ],
                12,
                yaw,
            )

        state = sdk.get_state()
        handle = state["objects"]["handle"]["pos"]
        yaw = state["objects"]["handle"]["yaw"] + pi / 2.0
        move(
            [handle[0] + lateral, handle[1] + 0.065, handle[2] + 0.025],
            28 if attempt == 0 else 20,
            yaw,
        )

        state = sdk.get_state()
        handle = state["objects"]["handle"]["pos"]
        move(
            [handle[0] + lateral, handle[1] + 0.045, handle[2] + 0.004],
            12,
            yaw,
        )

        state = sdk.get_state()
        handle = state["objects"]["handle"]["pos"]
        move(
            [handle[0] + lateral, handle[1] + front, handle[2] + 0.004],
            22 if attempt == 0 else 18,
            yaw,
            tolerance=0.004,
        )

        if remaining() <= 0:
            break
        sdk.set_gripper("close", steps=min(12, remaining()))

        state = sdk.get_state()
        stroke_start = state["eef_pos"][2]
        floor_z = stroke_start - 0.120
        commanded_z = stroke_start
        best_handle = state["handle_qpos"]
        stalled = 0
        released = False

        # Keep the successful live-handle approach and full bounded press.
        for tick in range(76):
            if remaining() <= 0:
                break
            state = sdk.get_state()
            if success(state):
                return result(state)
            if state["handle_qpos"] >= 1.15:
                released = True
                break

            if state["handle_qpos"] > best_handle + 0.008:
                best_handle = state["handle_qpos"]
                stalled = 0
            else:
                stalled += 1

            handle = state["objects"]["handle"]["pos"]
            commanded_z = max(floor_z, commanded_z - 0.003)
            track(
                state,
                [
                    handle[0] + lateral,
                    handle[1] + front,
                    max(floor_z, min(commanded_z, handle[2] - 0.014)),
                ],
            )

            if tick >= 32 and stalled >= 20:
                break

        state = sdk.get_state()
        if success(state):
            return result(state)
        released = released or state["handle_qpos"] >= 1.15
        if not released:
            continue

        handle = state["objects"]["handle"]["pos"]
        origin = list(state["eef_pos"])
        offset_x = clamp(origin[0] - handle[0], -0.025, 0.025)
        offset_y = clamp(origin[1] - handle[1], -0.015, 0.025)
        pressure = clamp(handle[2] - origin[2], 0.012, 0.035)
        pull_floor = origin[2] - 0.045
        best_hinge = state["hinge_qpos"]
        stagnant = 0
        slipping = 0
        travel_x = 0.0
        travel_y = 0.0

        # Accumulate an opening stroke instead of repeating a fixed small
        # target that can equilibrate with the hinge almost closed.
        for tick in range(125):
            if remaining() <= 0:
                break
            state = sdk.get_state()
            if success(state):
                return result(state)

            hinge = state["hinge_qpos"]
            handle_qpos = state["handle_qpos"]
            handle = state["objects"]["handle"]["pos"]

            if hinge > best_hinge + 0.002:
                best_hinge = hinge
                stagnant = 0
            else:
                stagnant += 1

            if handle_qpos < 1.0:
                slipping += 1
            else:
                slipping = 0

            if handle_qpos < 1.15:
                pressure = min(0.045, pressure + 0.001)
            else:
                pressure = max(0.014, pressure - 0.0003)

            # Briefly restore pressure at the live handle if the latch
            # starts returning before the door clears its frame.
            if hinge < 0.035 and handle_qpos < 1.05:
                track(
                    state,
                    [
                        handle[0] + offset_x,
                        handle[1] + offset_y + 0.008,
                        max(pull_floor, handle[2] - pressure),
                    ],
                    0.8,
                )
            else:
                travel_x = min(0.075, travel_x + 0.0012)
                travel_y = min(0.210, travel_y + 0.0035)
                lead_y = min(0.090, 0.030 + 0.0015 * stagnant)
                lead_x = min(0.035, 0.010 + 0.0005 * stagnant)

                target = [
                    min(
                        origin[0] + 0.080,
                        max(
                            origin[0] + travel_x,
                            handle[0] + offset_x + lead_x,
                        ),
                    ),
                    min(
                        origin[1] + 0.220,
                        max(
                            origin[1] + travel_y,
                            handle[1] + offset_y + lead_y,
                        ),
                    ),
                    max(pull_floor, handle[2] - pressure),
                ]
                track(state, target, 0.8)

            state = sdk.get_state()
            if success(state):
                return result(state)

            # Recover while there is still time to retract, shift by
            # 15 mm, and engage again using newly observed coordinates.
            if slipping >= 10:
                break
            if stagnant >= 32:
                break

    return result(sdk.get_state())
