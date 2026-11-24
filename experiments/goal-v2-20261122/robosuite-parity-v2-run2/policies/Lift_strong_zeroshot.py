def run_policy(sdk):
    state = sdk.get_state()
    if "cube" not in state["objects"]:
        return {"status": "failed", "reason": "Cube observation unavailable"}

    initial_cube_z = state["objects"]["cube"]["pos"][2]
    limit = min(400, state["max_steps"])

    def remaining():
        return max(0, limit - sdk.get_state()["steps_used"])

    def move(pos, yaw, gripper, budget, tol):
        steps = min(budget, remaining())
        if steps > 0:
            return sdk.move_to(
                pos, yaw=yaw, gripper=gripper,
                max_steps=steps, tol=tol
            )
        return {"reached": False}

    def actuate(action, budget):
        steps = min(budget, remaining())
        if steps > 0:
            sdk.set_gripper(action, steps=steps)

    for attempt in range(3):
        if remaining() < 65:
            break

        state = sdk.get_state()
        cube = state["objects"]["cube"]
        pos = cube["pos"]
        yaw = cube["yaw"]
        yaw += round((state["eef_yaw"] - yaw) / (pi / 2)) * (pi / 2)

        # Retreat vertically before moving sideways toward the cube.
        clearance = max(initial_cube_z + 0.12, pos[2] + 0.10)
        if state["eef_pos"][2] < clearance - 0.02:
            move(
                [state["eef_pos"][0], state["eef_pos"][1], clearance],
                None, None, 25, 0.012
            )

        actuate("open", 10)
        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        move([pos[0], pos[1], clearance], yaw, "open", 40, 0.008)

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.018:
            continue

        # The end-effector observation is the gripper grasp-center site.
        move(
            [pos[0], pos[1], pos[2] + 0.003],
            yaw, "open", 40, 0.005
        )

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.015
            or abs(eef[2] - pos[2] - 0.003) > 0.018
        ):
            continue

        actuate("close", 18)
        state = sdk.get_state()
        eef = state["eef_pos"]

        # Test attachment with a short vertical lift.
        move(
            [eef[0], eef[1], eef[2] + 0.055],
            yaw, "close", 25, 0.008
        )
        state = sdk.get_state()
        if state["objects"]["cube"]["pos"][2] < initial_cube_z + 0.025:
            continue

        eef = state["eef_pos"]
        cube_z = state["objects"]["cube"]["pos"][2]
        lift_distance = max(0.0, initial_cube_z + 0.16 - cube_z)
        move(
            [eef[0], eef[1], eef[2] + lift_distance],
            yaw, "close", 45, 0.010
        )

        if remaining() < 8:
            break
        actuate("close", 8)
        state = sdk.get_state()
        lifted = state["objects"]["cube"]["pos"][2] - initial_cube_z
        if lifted >= 0.10 and state["gripper_width"] > 0.012:
            return {
                "status": "done",
                "lift_height": lifted,
                "steps_used": state["steps_used"]
            }

    state = sdk.get_state()
    return {
        "status": "failed",
        "reason": "Could not verify a stable cube lift within the step budget",
        "lift_height": state["objects"]["cube"]["pos"][2] - initial_cube_z,
        "steps_used": state["steps_used"]
    }
