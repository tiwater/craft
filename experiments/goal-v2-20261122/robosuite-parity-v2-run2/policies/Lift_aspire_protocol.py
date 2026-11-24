def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, grip, limit, tol):
        budget = min(limit, remaining())
        if budget > 0:
            sdk.move_to(
                pos,
                yaw=yaw,
                gripper=grip,
                max_steps=budget,
                tol=tol,
            )

    def actuate(action, limit):
        budget = min(limit, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)

    state = sdk.get_state()
    if "cube" not in state["objects"]:
        return {"status": "failed", "reason": "Cube observation unavailable"}

    initial_z = state["objects"]["cube"]["pos"][2]
    required_rise = 0.10

    for attempt in range(3):
        if remaining() < 65:
            break

        state = sdk.get_state()
        cube = state["objects"]["cube"]
        pos = cube["pos"]
        yaw = cube["yaw"]
        yaw += round((state["eef_yaw"] - yaw) / (pi / 2)) * (pi / 2)

        # Clear the object vertically before moving sideways or opening.
        eef = state["eef_pos"]
        clearance = max(eef[2], pos[2] + 0.11)
        move([eef[0], eef[1], clearance], None, None, 25, 0.01)
        actuate("open", 12)

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        move([pos[0], pos[1], pos[2] + 0.10], yaw, "open", 40, 0.007)

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.018:
            continue

        # Small depth variations allow recovery from an imperfect first grasp.
        depth = [0.005, 0.0, 0.010][attempt]
        move([pos[0], pos[1], pos[2] + depth], yaw, "open", 40, 0.005)

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.015
            or abs(eef[2] - (pos[2] + depth)) > 0.018
        ):
            continue

        grasp_z = pos[2]
        actuate("close", 20)

        state = sdk.get_state()
        eef = state["eef_pos"]
        move([eef[0], eef[1], eef[2] + 0.055], yaw, "close", 30, 0.007)

        state = sdk.get_state()
        cube_z = state["objects"]["cube"]["pos"][2]

        # Verify that the cube followed the hand, rather than trusting closure.
        if cube_z - grasp_z < 0.025:
            continue

        eef = state["eef_pos"]
        lift = max(0.06, initial_z + required_rise + 0.04 - cube_z)
        move([eef[0], eef[1], eef[2] + lift], yaw, "close", 45, 0.008)

        if remaining() < 8:
            break
        actuate("close", 8)

        state = sdk.get_state()
        rise = state["objects"]["cube"]["pos"][2] - initial_z
        if rise >= required_rise:
            return {
                "status": "done",
                "cube_lift_m": rise,
                "steps_used": state["steps_used"],
            }

    state = sdk.get_state()
    return {
        "status": "failed",
        "reason": "Could not verify a sustained lift within the step budget",
        "cube_lift_m": state["objects"]["cube"]["pos"][2] - initial_z,
        "steps_used": state["steps_used"],
    }
