def run_policy(sdk):
    state = sdk.get_state()
    if state["handle_qpos"] is None or state["hinge_qpos"] is None:
        return {"status": "failed", "reason": "Missing door joint observations"}

    limit = min(400, state["max_steps"])
    best_handle = state["handle_qpos"]
    best_hinge = state["hinge_qpos"]

    for attempt in range(3):
        state = sdk.get_state()
        if state["hinge_qpos"] > 0.30:
            break
        if limit - state["steps_used"] < 35:
            break

        handle = list(state["objects"]["handle"]["pos"])
        lateral = [0.0, 0.018, -0.018][attempt]

        sdk.move_to(
            [handle[0] + lateral, handle[1] - 0.11, handle[2] + 0.035],
            gripper="open",
            max_steps=min(25, limit - state["steps_used"]),
            tol=0.015,
        )

        state = sdk.get_state()
        remaining = limit - state["steps_used"]
        if remaining <= 0:
            break
        handle = list(state["objects"]["handle"]["pos"])
        sdk.move_to(
            [handle[0] + lateral, handle[1] + 0.008, handle[2] + 0.006],
            gripper="open",
            max_steps=min(30, remaining),
            tol=0.008,
        )

        state = sdk.get_state()
        remaining = limit - state["steps_used"]
        if remaining <= 0:
            break
        handle = state["objects"]["handle"]["pos"]
        separation = sqrt(sum(
            (state["eef_pos"][axis] - handle[axis]) ** 2
            for axis in range(3)
        ))
        if separation > 0.075:
            continue

        sdk.set_gripper("close", steps=min(9, remaining))
        state = sdk.get_state()
        contact = list(state["eef_pos"])
        target = list(contact)
        start_hinge = state["hinge_qpos"]
        progress_hinge = start_hinge
        progress_handle = state["handle_qpos"]
        unlocked = False
        pull_ticks = 0
        stalled = 0

        for tick in range(160):
            state = sdk.get_state()
            if state["steps_used"] >= limit or state["hinge_qpos"] > 0.30:
                break

            latch = state["handle_qpos"]
            hinge = state["hinge_qpos"]
            eef = state["eef_pos"]
            handle = state["objects"]["handle"]["pos"]
            best_handle = max(best_handle, latch)
            best_hinge = max(best_hinge, hinge)

            if latch > 0.25 or hinge > start_hinge + 0.015:
                if not unlocked:
                    unlocked = True
                    target = list(eef)
                    target[2] -= 0.008
                    stalled = 0

            if unlocked:
                pull_ticks += 1
                angle = max(0.0, hinge) + 0.12
                tangent = [-sin(angle), -cos(angle)]

                # Advance from the actual wrist position so the pull persists
                # even when the handle observation remains stationary.
                lead = min(0.085, 0.025 + 0.002 * pull_ticks)
                target[0] = eef[0] + tangent[0] * lead
                target[1] = eef[1] + tangent[1] * lead

                # Hold the release height rather than repeatedly chasing
                # the rotating handle downward.
                if latch < 0.18 and hinge < 0.04:
                    target[2] = max(contact[2] - 0.075, target[2] - 0.002)

                if hinge > progress_hinge + 0.006:
                    progress_hinge = hinge
                    stalled = 0
                else:
                    stalled += 1

                if stalled >= 48:
                    break
            else:
                target[0] = contact[0]
                target[1] = contact[1] - 0.008
                target[2] = contact[2] - min(0.065, 0.003 * (tick + 1))

                if latch > progress_handle + 0.02:
                    progress_handle = latch
                    stalled = 0
                else:
                    stalled += 1
                if stalled >= 30:
                    break

            translation = [
                max(-1.0, min(1.0, (target[axis] - eef[axis]) / 0.05))
                for axis in range(3)
            ]
            sdk.step_raw(translation + [0.0, 0.0, 0.0, 1.0], steps=1)

            state = sdk.get_state()
            handle = state["objects"]["handle"]["pos"]
            separation = sqrt(sum(
                (state["eef_pos"][axis] - handle[axis]) ** 2
                for axis in range(3)
            ))
            if separation > 0.12:
                break

        state = sdk.get_state()
        if state["hinge_qpos"] > 0.30:
            break
        remaining = limit - state["steps_used"]
        if remaining > 0:
            sdk.set_gripper("open", steps=min(5, remaining))

    state = sdk.get_state()
    return {
        "status": "done" if state["hinge_qpos"] > 0.30 else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "best_hinge_qpos": max(best_hinge, state["hinge_qpos"]),
        "best_handle_qpos": max(best_handle, state["handle_qpos"]),
        "steps_used": state["steps_used"],
    }
