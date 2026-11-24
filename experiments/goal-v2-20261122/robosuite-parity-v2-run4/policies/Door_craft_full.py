def door_budget(sdk, requested):
    state = sdk.get_state()
    return max(0, min(requested, state["max_steps"] - state["steps_used"]))


def door_move(sdk, target, yaw, grip, ticks, tolerance=0.008):
    ticks = door_budget(sdk, ticks)
    if ticks > 0:
        sdk.move_to(
            target, yaw=yaw, gripper=grip,
            max_steps=ticks, tol=tolerance
        )


def door_distance(a, b):
    return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def run_policy(sdk):
    state = sdk.get_state()
    best_hinge = state["hinge_qpos"]

    for attempt in range(3):
        state = sdk.get_state()
        if state["hinge_qpos"] > 0.30:
            return {
                "status": "done",
                "hinge_qpos": state["hinge_qpos"],
                "steps_used": state["steps_used"]
            }
        if door_budget(sdk, 1) == 0:
            break

        handle = list(state["objects"]["handle"]["pos"])
        door = state["objects"]["door"]["pos"]
        rx = handle[0] - door[0]
        ry = handle[1] - door[1]
        radius = hypot(rx, ry)
        if radius < 0.025:
            yaw = state["objects"]["door"]["yaw"]
        else:
            yaw = atan2(ry, rx)

        if attempt == 1:
            yaw += pi / 2.0
        elif attempt == 2:
            yaw -= pi / 2.0

        ticks = door_budget(sdk, 10)
        if ticks > 0:
            sdk.set_gripper("open", steps=ticks)

        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        clearance = max(handle[2] + 0.075, state["eef_pos"][2])
        door_move(
            sdk,
            [state["eef_pos"][0], state["eef_pos"][1], clearance],
            yaw, "open", 12
        )
        door_move(
            sdk,
            [handle[0], handle[1] + 0.065, handle[2] + 0.045],
            yaw, "open", 20
        )

        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        door_move(
            sdk, [handle[0], handle[1] + 0.045, handle[2] + 0.008],
            yaw, "open", 12
        )

        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        door_move(
            sdk, [handle[0], handle[1] + 0.004, handle[2] + 0.004],
            yaw, "open", 24, 0.005
        )

        ticks = door_budget(sdk, 12)
        if ticks > 0:
            sdk.set_gripper("close", steps=ticks)

        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        if door_distance(state["eef_pos"], handle) > 0.075:
            continue

        grasp_offset = [
            max(-0.018, min(0.018, state["eef_pos"][0] - handle[0])),
            max(-0.008, min(0.018, state["eef_pos"][1] - handle[1])),
            max(-0.012, min(0.012, state["eef_pos"][2] - handle[2]))
        ]
        depression = 0.012
        previous_hinge = state["hinge_qpos"]
        previous_latch = state["handle_qpos"]
        previous_handle = list(handle)
        best_latch = previous_latch if previous_latch is not None else 0.0
        latch_stagnant = 0
        pull_stagnant = 0
        released = False
        release_hinge = previous_hinge
        observed_tx = 0.6
        observed_ty = 0.8

        for iteration in range(38):
            state = sdk.get_state()
            hinge = state["hinge_qpos"]
            latch = state["handle_qpos"]
            if hinge > 0.30:
                return {
                    "status": "done",
                    "hinge_qpos": hinge,
                    "steps_used": state["steps_used"]
                }
            if door_budget(sdk, 1) == 0:
                break

            handle = list(state["objects"]["handle"]["pos"])
            progress = hinge - previous_hinge
            best_hinge = max(best_hinge, hinge)

            if latch is not None:
                if latch > best_latch + 0.015:
                    best_latch = latch
                    latch_stagnant = 0
                else:
                    latch_stagnant += 1

                if latch >= 1.45 and not released:
                    released = True
                    release_hinge = hinge

            if released and progress > 0.0015:
                dx = handle[0] - previous_handle[0]
                dy = handle[1] - previous_handle[1]
                travel = hypot(dx, dy)
                if travel > 0.001 and dx > -0.002 and dy > 0.0:
                    observed_tx = 0.7 * observed_tx + 0.3 * dx / travel
                    observed_ty = 0.7 * observed_ty + 0.3 * dy / travel
                    length = hypot(observed_tx, observed_ty)
                    observed_tx /= length
                    observed_ty /= length
                pull_stagnant = 0
            elif released:
                pull_stagnant += 1

            needs_press = not released
            if latch is not None and latch < 1.45 and hinge < 0.06:
                needs_press = True

            if needs_press:
                increment = 0.008
                if latch is not None and latch >= 1.20:
                    increment = 0.004
                depression = min(0.095, depression + increment)
                pull = 0.0
            else:
                # Retain downward load throughout the opening motion.
                if latch is not None and latch < 1.45:
                    depression = min(0.095, depression + 0.004)
                pull = min(0.032, 0.014 + max(0.0, hinge - release_hinge) * 0.10)

            angle = max(0.0, hinge - release_hinge)
            arc_tx = 0.6 * cos(angle) + 0.8 * sin(angle)
            arc_ty = 0.8 * cos(angle) - 0.6 * sin(angle)
            tx = 0.65 * arc_tx + 0.35 * observed_tx
            ty = 0.65 * arc_ty + 0.35 * observed_ty
            length = hypot(tx, ty)
            tx /= length
            ty /= length

            target = [
                handle[0] + grasp_offset[0] + tx * pull,
                handle[1] + grasp_offset[1] + ty * pull,
                handle[2] + grasp_offset[2] - depression
            ]

            previous_hinge = hinge
            previous_handle = list(handle)
            previous_latch = latch
            door_move(sdk, target, None, "close", 6, 0.004)

            state = sdk.get_state()
            if state["hinge_qpos"] > 0.30:
                return {
                    "status": "done",
                    "hinge_qpos": state["hinge_qpos"],
                    "steps_used": state["steps_used"]
                }

            separation = door_distance(
                state["eef_pos"], state["objects"]["handle"]["pos"]
            )
            current_latch = state["handle_qpos"]
            latch_progress = (
                current_latch is not None
                and previous_latch is not None
                and current_latch > previous_latch + 0.01
            )

            if separation > 0.145:
                break

            # Empty finger closure alone does not disprove downward contact.
            if needs_press and latch_stagnant >= 10 and not latch_progress:
                break

            if released and pull_stagnant >= 10:
                if current_latch is not None and current_latch < 1.20:
                    released = False
                    latch_stagnant = 0
                elif separation > 0.10:
                    break

    state = sdk.get_state()
    return {
        "status": "done" if state["hinge_qpos"] > 0.30 else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "steps_used": state["steps_used"]
    }
