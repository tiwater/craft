def run_policy(sdk):
    state = sdk.get_state()
    if "cube" not in state["objects"]:
        return {"status": "failed", "reason": "Cube observation missing"}

    initial_z = state["objects"]["cube"]["pos"][2]
    attempts = 0

    def remaining():
        observation = sdk.get_state()
        return max(0, observation["max_steps"] - observation["steps_used"])

    def move(target, yaw, gripper, limit, tolerance):
        budget = min(limit, remaining())
        if budget <= 0:
            return False
        result = sdk.move_to(
            target,
            yaw=yaw,
            gripper=gripper,
            max_steps=budget,
            tol=tolerance,
        )
        return result["reached"]

    while attempts < 3 and remaining() >= 85:
        attempts += 1
        state = sdk.get_state()
        cube = state["objects"]["cube"]
        position = cube["pos"]

        yaw = cube["yaw"]
        yaw += round((state["eef_yaw"] - yaw) / (pi / 2)) * (pi / 2)

        # Open and retreat vertically before moving across the table.
        clearance = max(state["eef_pos"][2], position[2] + 0.13)
        move(
            [state["eef_pos"][0], state["eef_pos"][1], clearance],
            yaw,
            "open",
            30,
            0.012,
        )

        state = sdk.get_state()
        position = state["objects"]["cube"]["pos"]
        reached = move(
            [position[0], position[1], clearance],
            yaw,
            "open",
            45,
            0.008,
        )
        if not reached:
            continue

        # Refresh the target immediately before descending.
        state = sdk.get_state()
        position = state["objects"]["cube"]["pos"]
        target = [position[0], position[1], position[2] + 0.003]
        move(target, yaw, "open", 45, 0.005)

        state = sdk.get_state()
        position = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        horizontal_error = hypot(eef[0] - position[0], eef[1] - position[1])
        vertical_error = abs(eef[2] - (position[2] + 0.003))

        if horizontal_error > 0.012 or vertical_error > 0.018:
            if remaining() < 65:
                break
            move(
                [position[0], position[1], position[2] + 0.003],
                yaw,
                "open",
                25,
                0.005,
            )
            state = sdk.get_state()
            position = state["objects"]["cube"]["pos"]
            eef = state["eef_pos"]
            if (
                hypot(eef[0] - position[0], eef[1] - position[1]) > 0.012
                or abs(eef[2] - position[2] - 0.003) > 0.018
            ):
                continue

        if remaining() < 40:
            break

        sdk.set_gripper("close", steps=min(22, remaining()))
        state = sdk.get_state()
        eef = state["eef_pos"]

        # Judge the grasp by actual cube motion, rather than finger width alone.
        lift_target = [
            eef[0],
            eef[1],
            max(eef[2] + 0.17, initial_z + 0.18),
        ]
        move(lift_target, yaw, "close", 55, 0.01)

        state = sdk.get_state()
        height = state["objects"]["cube"]["pos"][2] - initial_z
        if height >= 0.10:
            if remaining() < 8:
                break
            move(lift_target, yaw, "close", 15, 0.008)
            sdk.set_gripper("close", steps=min(10, remaining()))
            state = sdk.get_state()
            height = state["objects"]["cube"]["pos"][2] - initial_z
            if height >= 0.10:
                return {
                    "status": "done",
                    "lift_height": height,
                    "attempts": attempts,
                    "steps_used": state["steps_used"],
                }

    state = sdk.get_state()
    return {
        "status": "failed",
        "reason": "Could not verify a sustained cube lift within the budget",
        "lift_height": state["objects"]["cube"]["pos"][2] - initial_z,
        "attempts": attempts,
        "steps_used": state["steps_used"],
    }
