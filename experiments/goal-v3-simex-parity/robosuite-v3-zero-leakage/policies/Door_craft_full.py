def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(target, steps, grip="close", tol=0.006):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.move_to(target, gripper=grip, max_steps=budget, tol=tol)

    def opened(state):
        return state["hinge_qpos"] is not None and state["hinge_qpos"] > 0.30

    def unit(x, y):
        length = hypot(x, y)
        if length < 0.00001:
            return [-sqrt(0.5), sqrt(0.5)]
        return [x / length, y / length]

    def latch_value(state):
        value = state["handle_qpos"]
        return value if value is not None else 0.0

    def hinge_value(state):
        value = state["hinge_qpos"]
        return value if value is not None else 0.0

    state = sdk.get_state()
    best_hinge = hinge_value(state)
    best_latch = latch_value(state)
    release_latch = None
    recoveries = 0

    if opened(state):
        return {"status": "done", "hinge_qpos": best_hinge}

    handle = list(state["objects"]["handle"]["pos"])
    door = state["objects"]["door"]["pos"]
    radial = unit(handle[0] - door[0], handle[1] - door[1])
    direction = unit(-abs(radial[1]), abs(radial[0]))

    move([handle[0], handle[1], handle[2] + 0.10], 40, "open")
    handle = list(sdk.get_state()["objects"]["handle"]["pos"])
    move(handle, 35, "open")
    if remaining() > 0:
        sdk.set_gripper("close", steps=min(12, remaining()))

    state = sdk.get_state()
    if state["gripper_width"] < 0.012 and remaining() > 60:
        sdk.set_gripper("open", steps=min(5, remaining()))
        handle = list(sdk.get_state()["objects"]["handle"]["pos"])
        move([handle[0], handle[1], handle[2] - 0.012], 18, "open")
        sdk.set_gripper("close", steps=min(10, remaining()))

    state = sdk.get_state()
    anchor = list(state["eef_pos"])
    floor_z = state["objects"]["handle"]["pos"][2] - 0.16
    hold_z = anchor[2]
    previous_latch = latch_value(state)

    # The measured 0.579-radian stroke did not release the catch.
    # Continue the observed downward actuation through its useful stroke.
    for stroke in range(10):
        if remaining() < 12:
            break
        state = sdk.get_state()
        if opened(state):
            return {"status": "done", "hinge_qpos": hinge_value(state)}
        current_latch = latch_value(state)
        best_latch = max(best_latch, current_latch)
        if current_latch >= 1.40:
            break
        hold_z = max(floor_z, hold_z - 0.015)
        move([anchor[0], anchor[1], hold_z], 7)
        after = sdk.get_state()
        best_latch = max(best_latch, latch_value(after))
        best_hinge = max(best_hinge, hinge_value(after))
        if hinge_value(after) > 0.04:
            release_latch = latch_value(after)
            break
        previous_latch = current_latch

    state = sdk.get_state()
    command = list(state["eef_pos"])
    command[2] = min(command[2], hold_z)
    hold_z = command[2]
    stalled = 0
    advancing = 0
    last_latch = latch_value(state)

    while remaining() > 0:
        before = sdk.get_state()
        if opened(before):
            return {
                "status": "done",
                "hinge_qpos": hinge_value(before),
                "steps_used": before["steps_used"],
            }

        hinge = hinge_value(before)
        latch = latch_value(before)
        handle = list(before["objects"]["handle"]["pos"])
        eef = before["eef_pos"]

        # Retain the depressed endpoint while pulling; do not reset the
        # vertical command to the moving handle's nominal height.
        required_latch = 1.40
        if release_latch is not None:
            required_latch = min(1.45, max(0.70, release_latch + 0.08))
        if latch < required_latch and hinge < 0.20:
            hold_z = max(floor_z, hold_z - 0.010)
        elif latch < last_latch - 0.12:
            hold_z = max(floor_z, hold_z - 0.008)

        stride = 0.018 if advancing < 2 else 0.027
        command[0] += stride * direction[0]
        command[1] += stride * direction[1]
        command[2] = hold_z

        # Bound commanded lead while allowing sustained force against the
        # catch, rather than repeatedly issuing the same short handle offset.
        lead_x = command[0] - eef[0]
        lead_y = command[1] - eef[1]
        lead = hypot(lead_x, lead_y)
        if lead > 0.13:
            command[0] = eef[0] + 0.13 * lead_x / lead
            command[1] = eef[1] + 0.13 * lead_y / lead

        move(command, 8)
        after = sdk.get_state()
        new_hinge = hinge_value(after)
        gain = new_hinge - hinge
        best_hinge = max(best_hinge, new_hinge)
        best_latch = max(best_latch, latch_value(after))

        if opened(after):
            return {
                "status": "done",
                "hinge_qpos": new_hinge,
                "steps_used": after["steps_used"],
            }

        if gain > 0.003:
            stalled = 0
            advancing += 1
            if new_hinge > 0.04 and release_latch is None:
                release_latch = min(latch, latch_value(after))

            angle = min(0.12, max(0.0, gain))
            predicted = [
                cos(angle) * direction[0] - sin(angle) * direction[1],
                sin(angle) * direction[0] + cos(angle) * direction[1],
            ]
            new_handle = after["objects"]["handle"]["pos"]
            dx = new_handle[0] - handle[0]
            dy = new_handle[1] - handle[1]
            if hypot(dx, dy) > 0.002 and abs(latch_value(after) - latch) < 0.08:
                tangent = unit(dx, dy)
                if tangent[0] * predicted[0] + tangent[1] * predicted[1] > 0.3:
                    predicted = unit(
                        0.4 * predicted[0] + 0.6 * tangent[0],
                        0.4 * predicted[1] + 0.6 * tangent[1],
                    )
            direction = unit(predicted[0], predicted[1])
        else:
            stalled += 1
            advancing = 0

        last_latch = latch_value(after)

        if stalled >= 4 and remaining() > 12:
            # Explore only the empirically supported pull quadrant before
            # hinge motion provides a reliable measured tangent.
            if new_hinge < 0.04:
                door = after["objects"]["door"]["pos"]
                live_handle = after["objects"]["handle"]["pos"]
                radial = unit(live_handle[0] - door[0], live_handle[1] - door[1])
                candidates = [
                    unit(-abs(radial[1]), abs(radial[0])),
                    unit(-0.70, 0.70),
                    unit(-0.95, 0.31),
                    unit(-0.15, 0.99),
                ]
                direction = candidates[(stalled // 4) % len(candidates)]
                command[0] = after["eef_pos"][0] + 0.055 * direction[0]
                command[1] = after["eef_pos"][1] + 0.055 * direction[1]

        if (
            after["gripper_width"] < 0.010
            and remaining() > 55
            and recoveries < 2
        ):
            recoveries += 1
            sdk.set_gripper("open", steps=min(5, remaining()))
            handle = list(sdk.get_state()["objects"]["handle"]["pos"])
            move([handle[0], handle[1], handle[2] + 0.025], 12, "open")
            move(handle, 12, "open")
            sdk.set_gripper("close", steps=min(10, remaining()))
            state = sdk.get_state()
            command = list(state["eef_pos"])
            hold_z = max(floor_z, command[2] - 0.035)
            command[2] = hold_z
            move(command, 8)
            stalled = 0

    state = sdk.get_state()
    return {
        "status": "done" if opened(state) else "failed",
        "hinge_qpos": hinge_value(state),
        "handle_qpos": latch_value(state),
        "best_hinge_qpos": best_hinge,
        "best_handle_qpos": best_latch,
        "steps_used": state["steps_used"],
    }
