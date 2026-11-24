def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(target, steps, grip="close", tol=0.008):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.move_to(
                target, gripper=grip, max_steps=budget, tol=tol
            )

    def unit(x, y):
        length = hypot(x, y)
        if length < 0.00001:
            return [1.0, 0.0]
        return [x / length, y / length]

    def opened(state):
        return state["hinge_qpos"] is not None and state["hinge_qpos"] > 0.30

    state = sdk.get_state()
    if opened(state):
        return {"status": "done", "hinge_qpos": state["hinge_qpos"]}

    handle = state["objects"]["handle"]["pos"]
    approach = unit(
        state["eef_pos"][0] - handle[0],
        state["eef_pos"][1] - handle[1],
    )

    # Approach vertically so the fingers can straddle the handle.
    move([handle[0], handle[1], handle[2] + 0.10], 40, "open")
    state = sdk.get_state()
    handle = state["objects"]["handle"]["pos"]
    move([handle[0], handle[1], handle[2]], 35, "open")
    if remaining() > 0:
        sdk.set_gripper("close", steps=min(12, remaining()))

    # An empty closure warrants one small, observation-relative correction.
    state = sdk.get_state()
    if state["gripper_width"] < 0.012 and remaining() > 50:
        sdk.set_gripper("open", steps=min(6, remaining()))
        handle = sdk.get_state()["objects"]["handle"]["pos"]
        move([handle[0], handle[1], handle[2] - 0.012], 18, "open")
        sdk.set_gripper("close", steps=min(12, remaining()))

    # Depress the lever. Retain the best observed turning direction.
    state = sdk.get_state()
    start_latch = state["handle_qpos"]
    if start_latch is None:
        start_latch = 0.0
    best_turn = 0.0
    latch_sign = -1.0

    for sign in [-1.0, 1.0]:
        if remaining() < 35:
            break
        anchor = list(sdk.get_state()["objects"]["handle"]["pos"])
        for distance in [0.025, 0.050, 0.075]:
            move(
                [anchor[0], anchor[1], anchor[2] + sign * distance],
                7,
            )
            state = sdk.get_state()
            if opened(state):
                return {"status": "done", "hinge_qpos": state["hinge_qpos"]}
            latch = state["handle_qpos"]
            turn = abs(latch - start_latch) if latch is not None else 0.0
            if turn > best_turn:
                best_turn = turn
                latch_sign = sign
            if turn > 0.30:
                break
        if best_turn > 0.30:
            break
        move(anchor, 8)

    # Probe horizontal pull directions, then follow the measured hinge arc.
    directions = [
        approach,
        [-approach[1], approach[0]],
        [approach[1], -approach[0]],
        [-approach[0], -approach[1]],
    ]
    direction = list(directions[0])
    probe = 0
    stalled = 0
    successes = 0
    best_hinge = sdk.get_state()["hinge_qpos"]
    if best_hinge is None:
        best_hinge = 0.0

    while remaining() > 0:
        before = sdk.get_state()
        if opened(before):
            return {
                "status": "done",
                "hinge_qpos": before["hinge_qpos"],
                "steps_used": before["steps_used"],
            }

        handle = list(before["objects"]["handle"]["pos"])
        hinge = before["hinge_qpos"]
        if hinge is None:
            hinge = 0.0

        # Once clear of the catch, avoid continuing to depress the lever.
        depression = 0.030 if hinge < 0.08 else 0.005
        distance = 0.035 if successes < 2 else 0.050
        target = [
            handle[0] + direction[0] * distance,
            handle[1] + direction[1] * distance,
            handle[2] + latch_sign * depression,
        ]
        move(target, 8)

        after = sdk.get_state()
        if opened(after):
            return {
                "status": "done",
                "hinge_qpos": after["hinge_qpos"],
                "steps_used": after["steps_used"],
            }

        new_hinge = after["hinge_qpos"]
        if new_hinge is None:
            new_hinge = hinge
        gain = new_hinge - hinge
        new_handle = after["objects"]["handle"]["pos"]

        if gain > 0.002:
            successes += 1
            stalled = 0
            dx = new_handle[0] - handle[0]
            dy = new_handle[1] - handle[1]
            if hypot(dx, dy) > 0.001:
                tangent = unit(dx, dy)
                # Extrapolate the measured chord toward the next arc segment.
                angle = min(0.15, max(0.0, gain))
                tangent = [
                    cos(angle) * tangent[0] - sin(angle) * tangent[1],
                    sin(angle) * tangent[0] + cos(angle) * tangent[1],
                ]
                direction = unit(
                    0.25 * direction[0] + 0.75 * tangent[0],
                    0.25 * direction[1] + 0.75 * tangent[1],
                )
        else:
            stalled += 1
            successes = 0

        best_hinge = max(best_hinge, new_hinge)

        if gain < -0.004:
            direction = [-direction[0], -direction[1]]
            stalled = 0
        elif stalled >= 2:
            probe += 1
            direction = list(directions[probe % len(directions)])
            stalled = 0

        # Recover contact if the fingers have visibly closed on empty space.
        if after["gripper_width"] < 0.010 and remaining() > 35:
            sdk.set_gripper("open", steps=min(5, remaining()))
            handle = sdk.get_state()["objects"]["handle"]["pos"]
            move([handle[0], handle[1], handle[2]], 12, "open")
            sdk.set_gripper("close", steps=min(8, remaining()))

    state = sdk.get_state()
    return {
        "status": "done" if opened(state) else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "best_hinge_qpos": best_hinge,
        "steps_used": state["steps_used"],
    }
