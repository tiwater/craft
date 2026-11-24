def policy_budget(sdk, requested):
    state = sdk.get_state()
    return max(0, min(requested, state["max_steps"] - state["steps_used"]))


def policy_move(sdk, pos, yaw, gripper, steps, tol):
    budget = policy_budget(sdk, steps)
    if budget > 0:
        return sdk.move_to(
            pos, yaw=yaw, gripper=gripper, max_steps=budget, tol=tol
        )
    return {"reached": False}


def policy_grip(sdk, action, steps):
    budget = policy_budget(sdk, steps)
    if budget > 0:
        sdk.set_gripper(action, steps=budget)


def run_policy(sdk):
    state = sdk.get_state()
    initial_cube_z = state["objects"]["cube"]["pos"][2]
    lift_height = 0.14

    for attempt in range(3):
        if policy_budget(sdk, 1) == 0:
            break

        state = sdk.get_state()
        cube = state["objects"]["cube"]
        pos = cube["pos"]
        yaw = cube["yaw"]
        yaw += round((state["eef_yaw"] - yaw) / (pi / 2)) * (pi / 2)

        # Rise before opening or moving laterally.
        eef = state["eef_pos"]
        clearance = max(eef[2], pos[2] + 0.11)
        policy_move(
            sdk, [eef[0], eef[1], clearance],
            yaw, None, 25, 0.012
        )
        policy_grip(sdk, "open", 12)

        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        approach = policy_move(
            sdk, [pos[0], pos[1], pos[2] + 0.10],
            yaw, "open", 45, 0.008
        )
        if not approach["reached"]:
            continue

        # Refresh the cube position immediately before descending.
        state = sdk.get_state()
        pos = state["objects"]["cube"]["pos"]
        grasp_offset = [0.002, -0.004, 0.008][attempt]
        descent = policy_move(
            sdk, [pos[0], pos[1], pos[2] + grasp_offset],
            yaw, "open", 40, 0.005
        )
        if not descent["reached"]:
            continue

        policy_grip(sdk, "close", 20)
        state = sdk.get_state()
        eef = state["eef_pos"]
        cube_before = state["objects"]["cube"]["pos"]
        offset = eef[2] - cube_before[2]

        # A short lift tests physical attachment using object motion.
        policy_move(
            sdk, [eef[0], eef[1], eef[2] + 0.055],
            yaw, "close", 30, 0.007
        )
        state = sdk.get_state()
        cube_after = state["objects"]["cube"]["pos"]
        if cube_after[2] - cube_before[2] < 0.022:
            continue

        eef = state["eef_pos"]
        target_z = max(
            eef[2], initial_cube_z + lift_height + offset
        )
        policy_move(
            sdk, [eef[0], eef[1], target_z],
            yaw, "close", 45, 0.008
        )

        if policy_budget(sdk, 10) < 10:
            break
        policy_grip(sdk, "close", 10)
        state = sdk.get_state()
        height_gain = state["objects"]["cube"]["pos"][2] - initial_cube_z

        if height_gain >= 0.10:
            return {
                "status": "done",
                "cube_height_gain": height_gain,
                "steps_used": state["steps_used"],
            }

    state = sdk.get_state()
    return {
        "status": "failed",
        "reason": "Could not verify a sustained lift within the step budget",
        "cube_height_gain": (
            state["objects"]["cube"]["pos"][2] - initial_cube_z
        ),
        "steps_used": state["steps_used"],
    }
