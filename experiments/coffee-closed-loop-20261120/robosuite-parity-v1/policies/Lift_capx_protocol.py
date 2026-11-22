def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def move_gripper(sdk, position, yaw, action, steps=50, tolerance=0.006):
    budget = min(steps, remaining_steps(sdk))
    if budget > 0:
        return sdk.move_to(
            position,
            yaw=yaw,
            gripper=action,
            max_steps=budget,
            tol=tolerance,
        )
    return {"reached": False}


def actuate_gripper(sdk, action, steps=15):
    budget = min(steps, remaining_steps(sdk))
    if budget > 0:
        sdk.set_gripper(action, steps=budget)


def nearest_cube_yaw(cube_yaw, reference_yaw):
    period = pi / 2.0
    return cube_yaw + round((reference_yaw - cube_yaw) / period) * period


def distance_xy(first, second):
    return hypot(first[0] - second[0], first[1] - second[1])


def verify_lift(sdk, target_height):
    for check in range(3):
        state = sdk.get_state()
        if state["objects"]["cube"]["pos"][2] < target_height:
            return False
        if check < 2:
            if remaining_steps(sdk) < 6:
                return False
            actuate_gripper(sdk, "close", steps=6)
    return True


def run_policy(sdk):
    state = sdk.get_state()
    initial_cube_z = state["objects"]["cube"]["pos"][2]
    required_height = initial_cube_z + 0.08

    for attempt in range(3):
        if remaining_steps(sdk) < 65:
            break

        state = sdk.get_state()
        cube = state["objects"]["cube"]
        cube_pos = list(cube["pos"])
        yaw = nearest_cube_yaw(cube["yaw"], state["eef_yaw"])

        # Retreat vertically before opening or moving laterally.
        clearance_z = max(
            state["eef_pos"][2],
            cube_pos[2] + 0.12,
            initial_cube_z + 0.12,
        )
        move_gripper(
            sdk,
            [state["eef_pos"][0], state["eef_pos"][1], clearance_z],
            state["eef_yaw"],
            "close" if attempt > 0 else "open",
            steps=35,
        )
        actuate_gripper(sdk, "open", steps=12)

        state = sdk.get_state()
        cube = state["objects"]["cube"]
        cube_pos = list(cube["pos"])
        yaw = nearest_cube_yaw(cube["yaw"], state["eef_yaw"])
        move_gripper(
            sdk,
            [cube_pos[0], cube_pos[1], clearance_z],
            yaw,
            "open",
            steps=50,
        )

        state = sdk.get_state()
        cube_pos = list(state["objects"]["cube"]["pos"])
        if distance_xy(state["eef_pos"], cube_pos) > 0.012:
            continue

        # The end-effector site is the grasp center between the fingers.
        grasp_z = cube_pos[2] + (0.003 if attempt == 0 else 0.0)
        move_gripper(
            sdk,
            [cube_pos[0], cube_pos[1], grasp_z],
            yaw,
            "open",
            steps=45,
            tolerance=0.004,
        )

        state = sdk.get_state()
        cube_pos = list(state["objects"]["cube"]["pos"])
        if (
            distance_xy(state["eef_pos"], cube_pos) > 0.012
            or abs(state["eef_pos"][2] - grasp_z) > 0.012
        ):
            continue

        actuate_gripper(sdk, "close", steps=20)

        # Use the actual grasp position to keep the lift vertical.
        state = sdk.get_state()
        grasp_position = list(state["eef_pos"])
        move_gripper(
            sdk,
            [
                grasp_position[0],
                grasp_position[1],
                grasp_position[2] + 0.045,
            ],
            yaw,
            "close",
            steps=30,
            tolerance=0.005,
        )

        state = sdk.get_state()
        cube_height = state["objects"]["cube"]["pos"][2]
        if cube_height < cube_pos[2] + 0.018:
            continue

        lift_z = max(
            grasp_position[2] + 0.13,
            state["eef_pos"][2] + required_height - cube_height + 0.035,
        )
        move_gripper(
            sdk,
            [grasp_position[0], grasp_position[1], lift_z],
            yaw,
            "close",
            steps=45,
            tolerance=0.006,
        )

        if verify_lift(sdk, required_height):
            state = sdk.get_state()
            return {
                "status": "done",
                "lift_m": state["objects"]["cube"]["pos"][2] - initial_cube_z,
                "steps_used": state["steps_used"],
            }

    state = sdk.get_state()
    return {
        "status": "failed",
        "reason": "Could not verify a sustained lift within the motion budget.",
        "lift_m": state["objects"]["cube"]["pos"][2] - initial_cube_z,
        "steps_used": state["steps_used"],
    }
