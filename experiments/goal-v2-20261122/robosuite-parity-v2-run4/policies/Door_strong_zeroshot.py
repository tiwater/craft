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
    initial_eef = list(state["eef_pos"])
    best_hinge = state["hinge_qpos"]
    preferred_direction = 1.0

    for attempt in range(3):
        state = sdk.get_state()
        if state["hinge_qpos"] > 0.30:
            return {"status": "done", "hinge_qpos": state["hinge_qpos"]}
        if door_budget(sdk, 1) == 0:
            break

        handle = list(state["objects"]["handle"]["pos"])
        door = state["objects"]["door"]["pos"]
        rx = handle[0] - door[0]
        ry = handle[1] - door[1]
        radius = hypot(rx, ry)

        if radius < 0.025:
            leaf_yaw = state["objects"]["door"]["yaw"]
            rx, ry = cos(leaf_yaw), sin(leaf_yaw)
        else:
            rx, ry = rx / radius, ry / radius

        nx, ny = -ry, rx
        toward_robot = (
            nx * (initial_eef[0] - handle[0])
            + ny * (initial_eef[1] - handle[1])
        )
        if attempt == 0:
            preferred_direction = 1.0 if toward_robot >= 0.0 else -1.0

        yaw = atan2(ry, rx)
        if attempt == 1:
            yaw += pi / 2.0
        elif attempt == 2:
            yaw -= pi / 2.0

        ticks = door_budget(sdk, 7)
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
            sdk, [handle[0], handle[1], handle[2] + 0.075],
            yaw, "open", 20
        )

        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        door_move(sdk, handle, yaw, "open", 22)

        ticks = door_budget(sdk, 12)
        if ticks > 0:
            sdk.set_gripper("close", steps=ticks)

        state = sdk.get_state()
        handle = state["objects"]["handle"]["pos"]
        if door_distance(state["eef_pos"], handle) > 0.065:
            continue

        grasp_offset = [
            state["eef_pos"][i] - handle[i] for i in range(3)
        ]
        depression = 0.0
        direction = preferred_direction
        stagnant = 0
        reversals = 0
        previous_hinge = state["hinge_qpos"]
        previous_handle = list(handle)
        observed_tx, observed_ty = nx, ny

        for iteration in range(24):
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
            door = state["objects"]["door"]["pos"]
            radial_x = handle[0] - door[0]
            radial_y = handle[1] - door[1]
            radial_length = hypot(radial_x, radial_y)

            if radial_length > 0.025:
                tx = -radial_y / radial_length
                ty = radial_x / radial_length
            else:
                tx, ty = observed_tx, observed_ty

            progress = hinge - previous_hinge
            if progress > 0.0015:
                dx = handle[0] - previous_handle[0]
                dy = handle[1] - previous_handle[1]
                travel = hypot(dx, dy)
                if travel > 0.001:
                    observed_tx, observed_ty = dx / travel, dy / travel
                    direction = (
                        1.0 if tx * observed_tx + ty * observed_ty >= 0.0
                        else -1.0
                    )
                stagnant = 0
                preferred_direction = direction
            else:
                stagnant += 1

            best_hinge = max(best_hinge, hinge)

            if hinge < 0.035 and latch is not None and latch < 0.18:
                depression = min(0.055, depression + 0.009)
                pull = 0.009
            else:
                depression = max(0.012, depression - 0.004)
                pull = 0.035

            if stagnant >= 4:
                direction = -direction
                stagnant = 0
                reversals += 1
                if reversals >= 3:
                    break

            target = [
                handle[0] + grasp_offset[0] + direction * tx * pull,
                handle[1] + grasp_offset[1] + direction * ty * pull,
                handle[2] + grasp_offset[2] - depression
            ]

            previous_hinge = hinge
            previous_handle = list(handle)
            door_move(sdk, target, None, "close", 6, 0.005)

            state = sdk.get_state()
            separation = door_distance(
                state["eef_pos"], state["objects"]["handle"]["pos"]
            )
            if separation > 0.095:
                break

            if iteration >= 5 and state["gripper_width"] < 0.009:
                if state["hinge_qpos"] <= previous_hinge + 0.001:
                    break

    state = sdk.get_state()
    return {
        "status": "done" if state["hinge_qpos"] > 0.30 else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "steps_used": state["steps_used"]
    }
