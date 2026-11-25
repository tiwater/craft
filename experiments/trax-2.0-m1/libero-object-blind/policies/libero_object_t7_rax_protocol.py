def policy_remaining(sdk):
    state = sdk.get_state()
    return max(0, min(450, state["max_steps"]) - state["steps_used"])


def policy_move(sdk, position, yaw, gripper, steps, tolerance=0.012):
    available = policy_remaining(sdk)
    if available <= 0:
        return False
    result = sdk.move_to(
        position,
        yaw=yaw,
        gripper=gripper,
        max_steps=min(steps, available),
        tol=tolerance,
    )
    return result["reached"]


def policy_gripper(sdk, action, steps):
    available = policy_remaining(sdk)
    if available > 0:
        return sdk.set_gripper(action, steps=min(steps, available))
    state = sdk.get_state()
    return {
        "gripper_width": state["gripper_width"],
        "grasped_hint": False,
    }


def policy_yaw(target, current):
    while target - current > pi / 2:
        target -= pi
    while target - current < -pi / 2:
        target += pi
    return target


def policy_destination(state, basket):
    site = basket + "_contain_region"
    if site in state["sites"]:
        return list(state["sites"][site])
    if basket in state["objects"]:
        position = state["objects"][basket]["pos"]
    else:
        position = state["fixtures"][basket]["pos"]
    return [position[0], position[1], position[2] + 0.07]


def policy_near(state, target, horizontal=0.02, vertical=0.025):
    eef = state["eef_pos"]
    return (
        hypot(eef[0] - target[0], eef[1] - target[1]) <= horizontal
        and abs(eef[2] - target[2]) <= vertical
    )


def policy_offset(state, milk):
    position = state["objects"][milk]["pos"]
    eef = state["eef_pos"]
    return [position[i] - eef[i] for i in range(3)]


def policy_offset_error(first, second):
    return sqrt(sum((first[i] - second[i]) ** 2 for i in range(3)))


def run_policy(sdk):
    state = sdk.get_state()
    milk = None
    basket = None
    for name in state["objects"]:
        if "milk" in name:
            milk = name
        if "basket" in name:
            basket = name
    if basket is None:
        for name in state["fixtures"]:
            if "basket" in name:
                basket = name

    if milk is None or basket is None:
        return {"status": "failed", "reason": "Required object missing"}

    grasped = False
    carry_yaw = state["eef_yaw"]
    last_failure = "Insufficient grasp budget"

    # Each reported rollout used 3 * (8 + 25) steps: all attempts
    # stopped at the initial clearance gate, before reaching the milk.
    # Raise with the current yaw, then rotate above the object. Use
    # measured positional clearance instead of requiring exact convergence.
    for attempt in range(3):
        if policy_remaining(sdk) < 210:
            break

        state = sdk.get_state()
        position = list(state["objects"][milk]["pos"])
        original_z = position[2]
        yaw = state["objects"][milk]["yaw"]
        if attempt == 1:
            yaw += pi / 2
        yaw = policy_yaw(yaw, state["eef_yaw"])

        policy_gripper(sdk, "open", 8)
        state = sdk.get_state()
        eef = state["eef_pos"]
        clearance = max(eef[2], position[2] + 0.18)
        raised = [eef[0], eef[1], clearance]

        policy_move(
            sdk, raised, state["eef_yaw"], "open", 40, 0.02
        )
        state = sdk.get_state()
        if state["eef_pos"][2] < position[2] + 0.13:
            last_failure = "Could not establish approach clearance"
            continue

        clearance = max(state["eef_pos"][2], position[2] + 0.16)
        approach = [position[0], position[1], clearance]
        policy_move(sdk, approach, yaw, "open", 60, 0.015)
        state = sdk.get_state()

        if not policy_near(state, approach, 0.018, 0.035):
            last_failure = "Could not align above milk"
            continue
        if abs(policy_yaw(yaw, state["eef_yaw"]) - state["eef_yaw"]) > 0.25:
            last_failure = "Could not align grasp yaw"
            continue

        position = list(state["objects"][milk]["pos"])
        grasp_height = position[2] + 0.015
        if attempt == 2:
            grasp_height = position[2] + 0.035
        grasp_position = [position[0], position[1], grasp_height]

        policy_move(sdk, grasp_position, yaw, "open", 50, 0.008)
        state = sdk.get_state()
        if not policy_near(state, grasp_position, 0.012, 0.018):
            last_failure = "Could not reach grasp position"
            continue

        policy_gripper(sdk, "close", 14)
        before_lift = sdk.get_state()
        offset_before = policy_offset(before_lift, milk)
        eef = before_lift["eef_pos"]

        policy_move(
            sdk,
            [eef[0], eef[1], eef[2] + 0.14],
            yaw,
            "close",
            35,
        )

        state = sdk.get_state()
        lifted = state["objects"][milk]["pos"]
        offset_after = policy_offset(state, milk)
        if (
            lifted[2] - original_z > 0.065
            and policy_offset_error(offset_after, offset_before) < 0.045
        ):
            grasped = True
            carry_yaw = yaw
            break
        last_failure = "Milk did not follow the gripper during lift"

    if not grasped:
        return {"status": "failed", "reason": last_failure}

    state = sdk.get_state()
    eef = state["eef_pos"]
    offset = policy_offset(state, milk)
    destination = policy_destination(state, basket)
    transit_z = max(eef[2], destination[2] + 0.24 - offset[2])
    raised = [eef[0], eef[1], transit_z]

    policy_move(sdk, raised, carry_yaw, "close", 30, 0.015)
    state = sdk.get_state()
    if state["eef_pos"][2] < transit_z - 0.025:
        return {"status": "failed", "reason": "Could not reach clearance"}

    destination = policy_destination(state, basket)
    transit = [
        destination[0] - offset[0],
        destination[1] - offset[1],
        max(transit_z, state["eef_pos"][2]),
    ]
    policy_move(sdk, transit, carry_yaw, "close", 55)
    state = sdk.get_state()
    if not policy_near(state, transit, 0.025, 0.035):
        return {"status": "failed", "reason": "Could not reach basket"}

    current_offset = policy_offset(state, milk)
    if policy_offset_error(current_offset, offset) > 0.05:
        return {"status": "failed", "reason": "Milk slipped during transport"}

    destination = policy_destination(state, basket)
    release_position = [
        destination[0] - current_offset[0],
        destination[1] - current_offset[1],
        destination[2] + 0.045 - current_offset[2],
    ]

    if policy_remaining(sdk) < 70:
        return {"status": "failed", "reason": "Insufficient placement budget"}

    policy_move(
        sdk, release_position, carry_yaw, "close", 35, 0.01
    )
    state = sdk.get_state()
    if not policy_near(state, release_position, 0.025, 0.025):
        return {"status": "failed", "reason": "Could not lower milk into basket"}

    position = state["objects"][milk]["pos"]
    destination = policy_destination(state, basket)
    if hypot(
        position[0] - destination[0], position[1] - destination[1]
    ) > 0.045:
        return {"status": "failed", "reason": "Milk is not centered over basket"}

    policy_gripper(sdk, "open", 12)
    eef = sdk.get_state()["eef_pos"]
    policy_move(
        sdk,
        [eef[0], eef[1], eef[2] + 0.16],
        carry_yaw,
        "open",
        25,
    )

    available = policy_remaining(sdk)
    if available > 0:
        sdk.step_raw(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
            steps=min(18, available),
        )

    state = sdk.get_state()
    position = state["objects"][milk]["pos"]
    destination = policy_destination(state, basket)
    horizontal_error = hypot(
        position[0] - destination[0], position[1] - destination[1]
    )
    vertical_error = position[2] - destination[2]
    released = (
        state["eef_pos"][2] - position[2] > 0.10
        and state["gripper_width"] > 0.05
    )

    if horizontal_error < 0.075 and -0.06 < vertical_error < 0.10 and released:
        return {"status": "done", "verification": "Milk released inside basket"}
    return {"status": "failed", "reason": "Final placement could not be verified"}
