def run_policy(sdk):
    def clip(value, limit):
        return max(-limit, min(limit, value))

    def remaining(state):
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def value(state, key):
        result = state[key]
        return 0.0 if result is None else result

    def success(state):
        return value(state, "hinge_qpos") > 0.30

    def move(position, yaw, grip, budget):
        state = sdk.get_state()
        ticks = min(budget, remaining(state))
        if ticks > 0 and not success(state):
            sdk.move_to(position, yaw=yaw, gripper=grip,
                        max_steps=ticks, tol=0.007)

    def finish(state):
        return {
            "status": "done" if success(state) else "failed",
            "hinge_qpos": state["hinge_qpos"],
            "handle_qpos": state["handle_qpos"],
            "steps_used": state["steps_used"]
        }

    state = sdk.get_state()
    initial_yaw = state["eef_yaw"]
    door = state["objects"]["door"]
    qx, qy, qz, qw = door["quat"]
    normal = [
        2.0 * (qx * qy - qz * qw),
        1.0 - 2.0 * (qx * qx + qz * qz)
    ]
    length = hypot(normal[0], normal[1])
    if length < 0.01:
        normal = [-sin(door["yaw"]), cos(door["yaw"])]
        length = 1.0
    normal = [normal[0] / length, normal[1] / length]

    # The door frame supplies the positive opening direction.
    # The task goal is not a reliable Cartesian pulling target.
    for attempt in range(3):
        state = sdk.get_state()
        if success(state):
            return finish(state)
        if remaining(state) < 20:
            break

        yaw = initial_yaw
        if attempt == 2:
            yaw += pi / 2.0

        handle = list(state["objects"]["handle"]["pos"])
        approach = [
            handle[0] + 0.035 * normal[0],
            handle[1] + 0.035 * normal[1],
            handle[2] + 0.075
        ]
        move(approach, yaw, "open", 24)

        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        move([handle[0], handle[1], handle[2] + 0.035],
             yaw, "open", 16)

        state = sdk.get_state()
        move(list(state["objects"]["handle"]["pos"]),
             yaw, "open", 20)

        state = sdk.get_state()
        if success(state):
            return finish(state)
        ticks = min(10, remaining(state))
        if ticks == 0:
            break
        sdk.set_gripper("close", steps=ticks)

        state = sdk.get_state()
        handle = state["objects"]["handle"]["pos"]
        offset = [
            clip(state["eef_pos"][i] - handle[i], 0.012)
            for i in range(3)
        ]
        start = state["steps_used"]
        best_hinge = value(state, "hinge_qpos")
        best_latch = value(state, "handle_qpos")
        last_hinge_progress = start
        last_latch_progress = start
        lost_ticks = 0

        while remaining(state) > 0:
            if success(state):
                return finish(state)

            hinge = value(state, "hinge_qpos")
            latch = value(state, "handle_qpos")
            tick = state["steps_used"]
            handle = list(state["objects"]["handle"]["pos"])

            if hinge > best_hinge + 0.003:
                best_hinge = hinge
                last_hinge_progress = tick
            if latch > best_latch + 0.025:
                best_latch = latch
                last_latch_progress = tick

            # Follow the rotating door, keeping force in its positive
            # opening direction rather than reversing a productive pull.
            theta = hinge
            tangent = [
                cos(theta) * normal[0] - sin(theta) * normal[1],
                sin(theta) * normal[0] + cos(theta) * normal[1]
            ]

            unlocked = latch > 0.38 or hinge > 0.04
            stalled = tick - last_hinge_progress
            lead = 0.0
            if unlocked:
                lead = 0.055
                if stalled > 18:
                    lead = 0.075
                if stalled > 35:
                    lead = 0.090

            # Never command upward motion to undo a depressed lever.
            depression = max(0.005, min(0.045, (0.95 - latch) * 0.07))
            if hinge > 0.10 and latch > 0.65:
                depression = 0.004

            target = [
                handle[0] + offset[0] + lead * tangent[0],
                handle[1] + offset[1] + lead * tangent[1],
                handle[2] + min(0.0, offset[2]) - depression
            ]
            action = [
                clip((target[i] - state["eef_pos"][i]) / 0.05, 0.85)
                for i in range(3)
            ]
            sdk.step_raw(action + [0.0, 0.0, 0.0, 1.0], steps=1)
            state = sdk.get_state()

            handle = state["objects"]["handle"]["pos"]
            separation = sqrt(sum(
                (state["eef_pos"][i] - handle[i]) ** 2
                for i in range(3)
            ))
            if separation > 0.105:
                lost_ticks += 1
            else:
                lost_ticks = 0

            if lost_ticks >= 8:
                break

            elapsed = state["steps_used"] - start
            if attempt < 2:
                if not unlocked and elapsed > 55:
                    if state["steps_used"] - last_latch_progress > 28:
                        break
                if unlocked and stalled > 65:
                    break

        if success(state):
            return finish(state)

        ticks = min(5, remaining(state))
        if ticks > 0:
            sdk.set_gripper("open", steps=ticks)

    return finish(sdk.get_state())
