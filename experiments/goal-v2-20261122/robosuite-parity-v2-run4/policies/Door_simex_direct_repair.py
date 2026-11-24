def run_policy(sdk):
    state = sdk.get_state()
    best_hinge = state["hinge_qpos"]
    motion_limit = max(0, state["max_steps"] - 12)

    for attempt in range(2):
        state = sdk.get_state()
        if state["hinge_qpos"] > 0.30:
            break
        if motion_limit - state["steps_used"] < 35:
            break

        handle = list(state["objects"]["handle"]["pos"])
        door = state["objects"]["door"]["pos"]
        yaw = atan2(handle[1] - door[1], handle[0] - door[0])
        if attempt:
            yaw += pi / 2.0

        sdk.set_gripper("open", steps=5)
        state = sdk.get_state()
        clearance = max(handle[2] + 0.075, state["eef_pos"][2])
        targets = [
            [state["eef_pos"][0], state["eef_pos"][1], clearance],
            [handle[0], handle[1], clearance],
            handle
        ]
        for target in targets:
            state = sdk.get_state()
            ticks = min(20, motion_limit - state["steps_used"])
            if ticks > 0:
                sdk.move_to(
                    target, yaw=yaw, gripper="open",
                    max_steps=ticks, tol=0.006
                )

        state = sdk.get_state()
        ticks = min(10, motion_limit - state["steps_used"])
        if ticks <= 0:
            break
        sdk.set_gripper("close", steps=ticks)

        state = sdk.get_state()
        handle = state["objects"]["handle"]["pos"]
        anchor = [state["eef_pos"][i] - handle[i] for i in range(3)]
        if sqrt(sum(value * value for value in anchor)) > 0.065:
            continue

        press_z = state["eef_pos"][2]
        for iteration in range(12):
            state = sdk.get_state()
            best_hinge = max(best_hinge, state["hinge_qpos"])
            if state["hinge_qpos"] > 0.30:
                break
            if abs(state["handle_qpos"]) >= 0.65:
                break
            ticks = min(3, motion_limit - state["steps_used"])
            if ticks <= 0:
                break
            handle = state["objects"]["handle"]["pos"]
            press_z = max(handle[2] - 0.040, press_z - 0.006)
            sdk.move_to(
                [handle[0] + anchor[0], handle[1] + anchor[1], press_z],
                gripper="close", max_steps=ticks, tol=0.003
            )

        state = sdk.get_state()
        handle = state["objects"]["handle"]["pos"]
        door = state["objects"]["door"]["pos"]
        rx = handle[0] - door[0]
        ry = handle[1] - door[1]
        direction = (
            atan2(rx, -ry)
            if hypot(rx, ry) > 0.025
            else state["objects"]["door"]["yaw"] + pi / 2.0
        )
        previous_yaw = state["objects"]["door"]["yaw"]
        anchor = [
            state["eef_pos"][0] - handle[0],
            state["eef_pos"][1] - handle[1]
        ]
        vertical = state["eef_pos"][2] - handle[2]
        previous_hinge = state["hinge_qpos"]
        window_hinge = previous_hinge
        stall = 0
        reversals = 0
        productive = False

        for iteration in range(100):
            state = sdk.get_state()
            hinge = state["hinge_qpos"]
            best_hinge = max(best_hinge, hinge)
            if hinge > 0.30:
                break
            ticks = min(3, motion_limit - state["steps_used"])
            if ticks <= 0:
                break

            handle = state["objects"]["handle"]["pos"]
            leaf_yaw = state["objects"]["door"]["yaw"]
            rotation = atan2(
                sin(leaf_yaw - previous_yaw),
                cos(leaf_yaw - previous_yaw)
            )
            direction += rotation
            anchor = [
                cos(rotation) * anchor[0] - sin(rotation) * anchor[1],
                sin(rotation) * anchor[0] + cos(rotation) * anchor[1]
            ]
            previous_yaw = leaf_yaw

            if hinge - previous_hinge > 0.001:
                productive = True
            stall += 1
            if stall >= 10:
                gain = hinge - window_hinge
                if gain < -0.008 or (gain < 0.003 and not productive):
                    direction += pi
                    reversals += 1
                elif gain < 0.003:
                    anchor[0] *= 0.75
                    anchor[1] *= 0.75
                window_hinge = hinge
                stall = 0

            latch = abs(state["handle_qpos"])
            if hinge < 0.12:
                if latch < 0.60:
                    vertical -= 0.004
                elif latch > 0.90:
                    vertical += min(0.012, 0.040 * (latch - 0.85))
            else:
                vertical *= 0.70
                if latch > 1.05:
                    vertical = max(vertical, min(0.025, 0.04 * (latch - 1.0)))
            vertical = max(-0.035, min(0.035, vertical))

            pull = 0.055 if productive else 0.035
            if hinge < 0.04 and latch < 0.55:
                pull = 0.012
            previous_hinge = hinge
            sdk.move_to(
                [
                    handle[0] + anchor[0] + pull * cos(direction),
                    handle[1] + anchor[1] + pull * sin(direction),
                    handle[2] + vertical
                ],
                gripper="close", max_steps=ticks, tol=0.003
            )

            state = sdk.get_state()
            if state["hinge_qpos"] > 0.30:
                break
            separation = sqrt(sum(
                (state["eef_pos"][i] -
                 state["objects"]["handle"]["pos"][i]) ** 2
                for i in range(3)
            ))
            if separation > 0.095 or reversals >= 2:
                break

    state = sdk.get_state()
    if state["hinge_qpos"] > 0.30:
        hold = list(state["eef_pos"])
        ticks = min(6, state["max_steps"] - state["steps_used"])
        if ticks > 0:
            sdk.move_to(
                hold, gripper="close", max_steps=ticks, tol=0.002
            )
        state = sdk.get_state()

    return {
        "status": "done" if state["hinge_qpos"] > 0.30 else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "best_hinge_qpos": max(best_hinge, state["hinge_qpos"]),
        "steps_used": state["steps_used"]
    }
