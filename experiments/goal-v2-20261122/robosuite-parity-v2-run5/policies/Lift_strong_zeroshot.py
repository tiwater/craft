def policy_budget(sdk, requested):
    state = sdk.get_state()
    return max(0, min(requested, state["max_steps"] - state["steps_used"]))


def policy_move(sdk, position, yaw, gripper, steps, tolerance):
    budget = policy_budget(sdk, steps)
    if budget == 0:
        return False
    result = sdk.move_to(
        position,
        yaw=yaw,
        gripper=gripper,
        max_steps=budget,
        tol=tolerance,
    )
    return result["reached"]


def run_policy(sdk):
    state = sdk.get_state()
    initial_height = state["objects"]["cube"]["pos"][2]
    attempts = 0

    while attempts < 3 and policy_budget(sdk, 400) >= 90:
        attempts += 1
        state = sdk.get_state()
        cube = state["objects"]["cube"]
        position = cube["pos"]
        grasp_height = position[2]

        yaw = cube["yaw"]
        yaw += round((state["eef_yaw"] - yaw) / (pi / 2)) * (pi / 2)

        # Open above the cube, then descend using its updated position.
        hover_height = max(
            position[2] + 0.10,
            state["eef_pos"][2],
        )
        reached = policy_move(
            sdk,
            [position[0], position[1], hover_height],
            yaw,
            "open",
            45,
            0.008,
        )
        if not reached:
            continue

        state = sdk.get_state()
        position = state["objects"]["cube"]["pos"]
        grasp_height = position[2]
        reached = policy_move(
            sdk,
            [position[0], position[1], position[2]],
            yaw,
            "open",
            40,
            0.005,
        )
        if not reached:
            continue

        budget = policy_budget(sdk, 20)
        if budget == 0:
            break
        sdk.set_gripper("close", steps=budget)

        # A short lift tests whether the cube follows the hand.
        state = sdk.get_state()
        hand = state["eef_pos"]
        policy_move(
            sdk,
            [hand[0], hand[1], hand[2] + 0.055],
            yaw,
            "close",
            30,
            0.006,
        )
        state = sdk.get_state()
        cube_height = state["objects"]["cube"]["pos"][2]

        if cube_height < grasp_height + 0.022:
            # Keep the hand elevated; the next approach opens the fingers.
            continue

        # Preserve the measured hand-to-object offset while lifting.
        hand = state["eef_pos"]
        target_height = hand[2] + max(
            0.08, initial_height + 0.17 - cube_height
        )
        policy_move(
            sdk,
            [hand[0], hand[1], target_height],
            yaw,
            "close",
            55,
            0.008,
        )

        budget = policy_budget(sdk, 12)
        if budget > 0:
            sdk.set_gripper("close", steps=budget)

        state = sdk.get_state()
        lift = state["objects"]["cube"]["pos"][2] - initial_height
        if lift >= 0.10:
            return {
                "status": "done",
                "lift_m": lift,
                "attempts": attempts,
                "steps_used": state["steps_used"],
            }

    state = sdk.get_state()
    lift = state["objects"]["cube"]["pos"][2] - initial_height
    return {
        "status": "done" if lift >= 0.10 else "failed",
        "lift_m": lift,
        "attempts": attempts,
        "steps_used": state["steps_used"],
    }
