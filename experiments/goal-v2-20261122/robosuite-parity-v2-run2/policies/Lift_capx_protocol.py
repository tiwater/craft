def remaining_steps(sdk):
    state = sdk.get_state()
    return max(0, state["max_steps"] - state["steps_used"])


def move_gripper_to(sdk, position, yaw, gripper, limit=50, tolerance=0.006):
    budget = min(limit, remaining_steps(sdk))
    if budget <= 0:
        return False
    result = sdk.move_to(
        position,
        yaw=yaw,
        gripper=gripper,
        max_steps=budget,
        tol=tolerance,
    )
    return result["reached"]


def actuate_gripper(sdk, action, ticks=15):
    budget = min(ticks, remaining_steps(sdk))
    if budget > 0:
        sdk.set_gripper(action, steps=budget)


def nearest_cube_yaw(cube_yaw, current_yaw):
    period = pi / 2.0
    return cube_yaw + round((current_yaw - cube_yaw) / period) * period


def horizontal_distance(first, second):
    return hypot(first[0] - second[0], first[1] - second[1])


def cube_is_lifted(state, initial_height, minimum_rise=0.10):
    cube = state["objects"]["cube"]["pos"]
    hand = state["eef_pos"]
    separation = sqrt(sum((cube[i] - hand[i]) ** 2 for i in range(3)))
    return (
        cube[2] >= initial_height + minimum_rise
        and horizontal_distance(cube, hand) < 0.045
        and separation < 0.09
    )


def run_policy(sdk):
    state = sdk.get_state()
    initial_height = state["objects"]["cube"]["pos"][2]
    grasp_offsets = [0.003, -0.004, 0.009]

    for attempt in range(len(grasp_offsets)):
        if remaining_steps(sdk) < 75:
            break

        state = sdk.get_state()
        cube = state["objects"]["cube"]["pos"]
        yaw = nearest_cube_yaw(
            state["objects"]["cube"]["yaw"], state["eef_yaw"]
        )

        # Gain clearance before opening or moving sideways.
        clearance = max(
            state["eef_pos"][2], cube[2] + 0.12, initial_height + 0.12
        )
        move_gripper_to(
            sdk,
            [state["eef_pos"][0], state["eef_pos"][1], clearance],
            state["eef_yaw"],
            "close" if attempt > 0 else "open",
            limit=30,
        )
        actuate_gripper(sdk, "open", ticks=12)

        state = sdk.get_state()
        cube = state["objects"]["cube"]["pos"]
        yaw = nearest_cube_yaw(
            state["objects"]["cube"]["yaw"], state["eef_yaw"]
        )
        move_gripper_to(
            sdk, [cube[0], cube[1], cube[2] + 0.10],
            yaw, "open", limit=40
        )

        state = sdk.get_state()
        cube = state["objects"]["cube"]["pos"]
        if horizontal_distance(state["eef_pos"], cube) > 0.018:
            continue

        # Refresh the cube position before each part of the descent.
        move_gripper_to(
            sdk, [cube[0], cube[1], cube[2] + 0.04],
            yaw, "open", limit=25
        )
        state = sdk.get_state()
        cube = state["objects"]["cube"]["pos"]
        target = [cube[0], cube[1], cube[2] + grasp_offsets[attempt]]
        move_gripper_to(sdk, target, yaw, "open", limit=30, tolerance=0.004)

        state = sdk.get_state()
        cube = state["objects"]["cube"]["pos"]
        if (
            horizontal_distance(state["eef_pos"], cube) > 0.015
            or abs(state["eef_pos"][2] - cube[2]) > 0.022
        ):
            continue
        if remaining_steps(sdk) < 40:
            break

        actuate_gripper(sdk, "close", ticks=20)
        state = sdk.get_state()
        hand = state["eef_pos"]
        cube_before = list(state["objects"]["cube"]["pos"])

        # A short test lift verifies actual object motion.
        move_gripper_to(
            sdk, [hand[0], hand[1], hand[2] + 0.055],
            yaw, "close", limit=25
        )
        state = sdk.get_state()
        cube = state["objects"]["cube"]["pos"]
        if (
            cube[2] - cube_before[2] < 0.022
            or horizontal_distance(cube, state["eef_pos"]) > 0.045
        ):
            continue

        hand = state["eef_pos"]
        lift_height = hand[2] + max(0.06, initial_height + 0.17 - cube[2])
        move_gripper_to(
            sdk, [hand[0], hand[1], lift_height],
            yaw, "close", limit=45
        )
        if remaining_steps(sdk) < 8:
            break

        actuate_gripper(sdk, "close", ticks=8)
        state = sdk.get_state()
        if cube_is_lifted(state, initial_height):
            return {
                "status": "done",
                "cube_rise_m": state["objects"]["cube"]["pos"][2] - initial_height,
                "steps_used": state["steps_used"],
            }

    state = sdk.get_state()
    return {
        "status": "failed",
        "reason": "Could not confirm a stable lift within the control budget.",
        "cube_rise_m": state["objects"]["cube"]["pos"][2] - initial_height,
        "steps_used": state["steps_used"],
    }
