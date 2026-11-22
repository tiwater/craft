def run_policy(sdk):
    def budget(state):
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def distance(a, b):
        return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    def result(state, status):
        return {
            "status": status,
            "hinge_qpos": state["hinge_qpos"],
            "handle_qpos": state["handle_qpos"],
            "steps_used": state["steps_used"],
        }

    def move(target, yaw, grip, ticks):
        state = sdk.get_state()
        available = budget(state)
        if available > 0:
            sdk.move_to(
                target,
                yaw=yaw,
                gripper=grip,
                max_steps=min(ticks, available),
                tol=0.006,
            )
        return sdk.get_state()

    state = sdk.get_state()
    if state["hinge_qpos"] > 0.30:
        return result(state, "done")

    initial_yaw = state["eef_yaw"]
    initial_hinge = state["hinge_qpos"]
    offsets = [0.0, -0.018, 0.018]

    for attempt in range(3):
        state = sdk.get_state()
        if state["hinge_qpos"] > 0.30:
            return result(state, "done")
        if budget(state) < 35:
            break

        handle = state["objects"]["handle"]["pos"]
        xoffset = offsets[attempt]
        yaw = initial_yaw + state["hinge_qpos"] - initial_hinge

        # Withdraw toward the accessible front before repositioning.
        if attempt > 0:
            sdk.set_gripper("open", steps=min(6, budget(state)))
            state = sdk.get_state()
            eef = state["eef_pos"]
            state = move(
                [eef[0], eef[1] - 0.065, eef[2] + 0.025],
                yaw, "open", 12,
            )

        handle = state["objects"]["handle"]["pos"]
        state = move(
            [handle[0] + xoffset, handle[1] - 0.085, handle[2] + 0.025],
            yaw, "open", 28,
        )
        handle = state["objects"]["handle"]["pos"]
        state = move(
            [handle[0] + xoffset, handle[1] - 0.012, handle[2] + 0.004],
            yaw, "open", 22,
        )

        if budget(state) <= 0:
            break
        sdk.set_gripper("close", steps=min(12, budget(state)))
        state = sdk.get_state()

        best_hinge = state["hinge_qpos"]
        best_latch = state["handle_qpos"]
        stalled = 0
        detached = 0

        for cycle in range(42):
            state = sdk.get_state()
            if state["hinge_qpos"] > 0.30:
                return result(state, "done")
            if budget(state) <= 0:
                return result(state, "failed")

            handle = state["objects"]["handle"]["pos"]
            eef = state["eef_pos"]
            hinge = state["hinge_qpos"]
            latch = state["handle_qpos"]

            # Keep downward preload while following the live swing arc.
            angle = max(0.0, min(1.2, hinge - initial_hinge))
            pull = 0.012 if latch < 0.15 else 0.023
            dx = -pull * sin(angle + 0.25)
            dy = -pull * cos(angle + 0.25)
            dz = -0.012 if latch < 0.20 else -0.004

            target = [
                eef[0] + dx,
                eef[1] + dy,
                max(handle[2] - 0.065, eef[2] + dz),
            ]

            # Bound accumulated position error so a missed grip cannot
            # carry the hand arbitrarily far away from the handle.
            target[0] = max(
                handle[0] - 0.075,
                min(handle[0] + 0.060, target[0]),
            )
            target[1] = max(
                handle[1] - 0.085,
                min(handle[1] + 0.015, target[1]),
            )
            yaw = initial_yaw + hinge - initial_hinge
            state = move(target, yaw, "close", 5)

            if state["hinge_qpos"] > 0.30:
                return result(state, "done")

            hinge_gain = state["hinge_qpos"] - best_hinge
            latch_gain = state["handle_qpos"] - best_latch
            if hinge_gain > 0.005 or latch_gain > 0.025:
                stalled = 0
            else:
                stalled += 1
            best_hinge = max(best_hinge, state["hinge_qpos"])
            best_latch = max(best_latch, state["handle_qpos"])

            separation = distance(
                state["eef_pos"], state["objects"]["handle"]["pos"]
            )
            if separation > 0.115:
                detached += 1
            else:
                detached = 0

            if detached >= 2 or stalled >= 11:
                break

    state = sdk.get_state()
    return result(state, "done" if state["hinge_qpos"] > 0.30 else "failed")
