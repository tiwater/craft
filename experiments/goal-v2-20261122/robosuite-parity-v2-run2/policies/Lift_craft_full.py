def run_policy(sdk):
    state = sdk.get_state()
    if "cube" not in state["objects"]:
        return {"status": "failed", "reason": "Cube observation unavailable"}

    initial_height = state["objects"]["cube"]["pos"][2]
    limit = min(400, state["max_steps"])

    def remaining():
        return max(0, limit - sdk.get_state()["steps_used"])

    def move(pos, yaw, gripper, steps, tol=0.008):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.move_to(
                pos,
                yaw=yaw,
                gripper=gripper,
                max_steps=budget,
                tol=tol,
            )

    def grip(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    for attempt in range(3):
        if remaining() < 85:
            break

        state = sdk.get_state()
        cube = list(state["objects"]["cube"]["pos"])
        yaw = state["objects"]["cube"]["yaw"]
        yaw += round((state["eef_yaw"] - yaw) / (pi / 2)) * (pi / 2)

        grip("open", 10)

        # Clear the cube before moving laterally over its live position.
        state = sdk.get_state()
        eef = state["eef_pos"]
        clearance = cube[2] + 0.11
        if eef[2] < clearance - 0.015:
            move([eef[0], eef[1], clearance], yaw, "open", 25)

        state = sdk.get_state()
        cube = list(state["objects"]["cube"]["pos"])
        move([cube[0], cube[1], cube[2] + 0.11], yaw, "open", 45)

        state = sdk.get_state()
        cube = list(state["objects"]["cube"]["pos"])
        eef = state["eef_pos"]
        if hypot(eef[0] - cube[0], eef[1] - cube[1]) > 0.018:
            continue

        grasp_height = cube[2] + (0.004 if attempt == 0 else 0.0)
        move([cube[0], cube[1], grasp_height], yaw, "open", 40, 0.005)

        # Correct for any cube motion during the approach.
        state = sdk.get_state()
        cube = list(state["objects"]["cube"]["pos"])
        eef = state["eef_pos"]
        if hypot(eef[0] - cube[0], eef[1] - cube[1]) > 0.008:
            move([cube[0], cube[1], cube[2] + 0.002],
                 yaw, "open", 15, 0.005)

        state = sdk.get_state()
        cube = list(state["objects"]["cube"]["pos"])
        eef = state["eef_pos"]
        if (hypot(eef[0] - cube[0], eef[1] - cube[1]) > 0.018
                or abs(eef[2] - cube[2]) > 0.025):
            continue

        grasp_cube_height = cube[2]
        grip("close", 20)

        state = sdk.get_state()
        eef = list(state["eef_pos"])
        move([eef[0], eef[1], eef[2] + 0.055], yaw, "close", 30)

        state = sdk.get_state()
        cube = state["objects"]["cube"]["pos"]
        if cube[2] < grasp_cube_height + 0.025:
            continue

        # Lift based on observed cube height, preserving the grasp offset.
        eef = list(state["eef_pos"])
        rise = max(0.06, initial_height + 0.16 - cube[2])
        move([eef[0], eef[1], eef[2] + rise], yaw, "close", 45)

        if remaining() < 8:
            break
        grip("close", 8)

        state = sdk.get_state()
        cube = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        height_gain = cube[2] - initial_height
        if (height_gain >= 0.10
                and hypot(eef[0] - cube[0], eef[1] - cube[1]) < 0.05
                and abs(eef[2] - cube[2]) < 0.08):
            return {
                "status": "done",
                "cube_height_gain": height_gain,
                "steps_used": state["steps_used"],
            }

    state = sdk.get_state()
    return {
        "status": "failed",
        "reason": "Could not verify a stable lift within the step budget",
        "cube_height_gain": state["objects"]["cube"]["pos"][2] - initial_height,
        "steps_used": state["steps_used"],
    }
