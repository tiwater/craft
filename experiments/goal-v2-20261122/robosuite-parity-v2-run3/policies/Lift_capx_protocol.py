def distance_xy(a, b):
    return hypot(a[0] - b[0], a[1] - b[1])


def remaining_steps(state):
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def aligned_grasp_yaw(cube_yaw, eef_yaw):
    period = pi / 2.0
    return cube_yaw + round((eef_yaw - cube_yaw) / period) * period


def move_budgeted(sdk, position, yaw, gripper, limit, tolerance):
    state = sdk.get_state()
    budget = min(limit, remaining_steps(state))
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


def actuate_budgeted(sdk, action, ticks):
    state = sdk.get_state()
    budget = min(ticks, remaining_steps(state))
    if budget <= 0:
        return False
    sdk.set_gripper(action, steps=budget)
    return True


def lift_is_stable(sdk, resting_height, minimum_lift):
    state = sdk.get_state()
    cube = state["objects"]["cube"]["pos"]
    if cube[2] - resting_height < minimum_lift:
        return False

    offset = [
        cube[0] - state["eef_pos"][0],
        cube[1] - state["eef_pos"][1],
        cube[2] - state["eef_pos"][2],
    ]

    for check in range(3):
        if remaining_steps(sdk.get_state()) < 5:
            return False
        actuate_budgeted(sdk, "close", 5)
        state = sdk.get_state()
        cube = state["objects"]["cube"]["pos"]
        eef = state["eef_pos"]
        drift = sqrt(sum(
            (cube[axis] - eef[axis] - offset[axis]) ** 2
            for axis in range(3)
        ))
        if cube[2] - resting_height < minimum_lift:
            return False
        if drift > 0.025 or distance_xy(cube, eef) > 0.055:
            return False

    return True


def run_policy(sdk):
    state = sdk.get_state()
    resting_height = state["objects"]["cube"]["pos"][2]
    attempts = 0

    for attempt in range(3):
        state = sdk.get_state()
        if remaining_steps(state) < 95:
            break
        attempts += 1

        cube = state["objects"]["cube"]
        yaw = aligned_grasp_yaw(cube["yaw"], state["eef_yaw"])

        # Release and retreat vertically before moving across the table.
        if not actuate_budgeted(sdk, "open", 12):
            break
        state = sdk.get_state()
        cube_pos = state["objects"]["cube"]["pos"]
        clearance = max(resting_height + 0.11, cube_pos[2] + 0.10)
        eef = state["eef_pos"]

        if eef[2] < clearance - 0.015:
            if not move_budgeted(
                sdk, [eef[0], eef[1], clearance],
                yaw, "open", 30, 0.012
            ):
                continue

        state = sdk.get_state()
        cube = state["objects"]["cube"]
        yaw = aligned_grasp_yaw(cube["yaw"], state["eef_yaw"])
        position = cube["pos"]

        if not move_budgeted(
            sdk, [position[0], position[1], clearance],
            yaw, "open", 40, 0.008
        ):
            continue

        # Refresh the target after every motion in case contact moved the cube.
        state = sdk.get_state()
        position = state["objects"]["cube"]["pos"]
        grasp_height = position[2] + (0.004 if attempt == 0 else 0.0)
        reached = move_budgeted(
            sdk, [position[0], position[1], grasp_height],
            yaw, "open", 40, 0.006
        )
        state = sdk.get_state()
        position = state["objects"]["cube"]["pos"]

        if (
            not reached
            or distance_xy(state["eef_pos"], position) > 0.012
            or abs(state["eef_pos"][2] - grasp_height) > 0.012
        ):
            continue

        if not actuate_budgeted(sdk, "close", 20):
            break

        state = sdk.get_state()
        before_probe = list(state["objects"]["cube"]["pos"])
        eef = state["eef_pos"]
        move_budgeted(
            sdk, [eef[0], eef[1], eef[2] + 0.05],
            yaw, "close", 30, 0.008
        )

        state = sdk.get_state()
        position = state["objects"]["cube"]["pos"]
        if (
            position[2] - before_probe[2] < 0.025
            or distance_xy(position, state["eef_pos"]) > 0.05
        ):
            continue

        # Preserve the measured grasp offset while raising the cube.
        eef = state["eef_pos"]
        rise = max(0.0, resting_height + 0.15 - position[2])
        move_budgeted(
            sdk, [eef[0], eef[1], eef[2] + rise],
            yaw, "close", 45, 0.01
        )

        if lift_is_stable(sdk, resting_height, 0.10):
            state = sdk.get_state()
            return {
                "status": "done",
                "attempts": attempts,
                "lift_height_m": (
                    state["objects"]["cube"]["pos"][2] - resting_height
                ),
                "steps_used": state["steps_used"],
            }

    state = sdk.get_state()
    return {
        "status": "failed",
        "reason": "Could not verify a stable grasp and lift within the step budget.",
        "attempts": attempts,
        "lift_height_m": state["objects"]["cube"]["pos"][2] - resting_height,
        "steps_used": state["steps_used"],
    }
