def run_policy(sdk):
    state = sdk.get_state()
    initial_cube_z = state["objects"]["cube"]["pos"][2]
    target_cube_z = initial_cube_z + 0.08

    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, gripper, budget, tol):
        steps = min(budget, remaining())
        if steps > 0:
            sdk.move_to(
                pos,
                yaw=yaw,
                gripper=gripper,
                max_steps=steps,
                tol=tol,
            )

    def grip(action, budget):
        steps = min(budget, remaining())
        if steps > 0:
            sdk.set_gripper(action, steps=steps)

    for attempt in range(3):
        if remaining() < 65:
            break

        state = sdk.get_state()
        cube = list(state["objects"]["cube"]["pos"])
        yaw = state["objects"]["cube"]["yaw"]
        current_yaw = state["eef_yaw"]
        yaw += round((current_yaw - yaw) / (pi / 2)) * (pi / 2)

        # Clear the cube before approaching from directly above.
        eef = state["eef_pos"]
        clearance_z = max(eef[2], cube[2] + 0.10)
        move(
            [eef[0], eef[1], clearance_z],
            current_yaw, "open", 20, 0.012,
        )
        move(
            [cube[0], cube[1], clearance_z],
            yaw, "open", 40, 0.008,
        )

        state = sdk.get_state()
        cube = list(state["objects"]["cube"]["pos"])
        eef = state["eef_pos"]
        lateral_error = hypot(eef[0] - cube[0], eef[1] - cube[1])
        if lateral_error > 0.018:
            continue

        move(
            [cube[0], cube[1], cube[2] + 0.002],
            yaw, "open", 40, 0.005,
        )

        state = sdk.get_state()
        cube = list(state["objects"]["cube"]["pos"])
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - cube[0], eef[1] - cube[1]) > 0.015
            or abs(eef[2] - cube[2]) > 0.018
        ):
            continue

        grip("close", 20)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        cube = state["objects"]["cube"]["pos"]

        # Preserve horizontal position while lifting with the fingers closed.
        lift_z = eef[2] + max(0.12, target_cube_z - cube[2] + 0.035)
        move([eef[0], eef[1], lift_z], yaw, "close", 55, 0.008)
        grip("close", 10)

        state = sdk.get_state()
        cube = state["objects"]["cube"]["pos"]
        if cube[2] >= target_cube_z:
            return {
                "status": "done",
                "cube_z": cube[2],
                "lift_height": cube[2] - initial_cube_z,
                "steps_used": state["steps_used"],
            }

        # If still holding the cube, allow additional vertical travel.
        if 0.015 <= state["gripper_width"] <= 0.065 and remaining() >= 20:
            eef = state["eef_pos"]
            move(
                [eef[0], eef[1], eef[2] + 0.06],
                yaw, "close", 30, 0.008,
            )
            grip("close", 8)
            state = sdk.get_state()
            cube = state["objects"]["cube"]["pos"]
            if cube[2] >= target_cube_z:
                return {
                    "status": "done",
                    "cube_z": cube[2],
                    "lift_height": cube[2] - initial_cube_z,
                    "steps_used": state["steps_used"],
                }

    state = sdk.get_state()
    cube_z = state["objects"]["cube"]["pos"][2]
    return {
        "status": "done" if cube_z >= target_cube_z else "failed",
        "cube_z": cube_z,
        "lift_height": cube_z - initial_cube_z,
        "steps_used": state["steps_used"],
    }
