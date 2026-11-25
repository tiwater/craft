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
        sdk.set_gripper(action, steps=min(steps, available))


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
    position = state["objects"][basket]["pos"]
    return [position[0], position[1], position[2] + 0.07]


def run_policy(sdk):
    state = sdk.get_state()
    milk = None
    basket = None
    for name in state["objects"]:
        if "milk" in name:
            milk = name
        if "basket" in name:
            basket = name

    if milk is None or basket is None:
        return {"status": "failed", "reason": "Required object missing"}

    grasped = False
    carry_yaw = state["eef_yaw"]

    for attempt in range(3):
        if policy_remaining(sdk) < 175:
            break

        state = sdk.get_state()
        position = list(state["objects"][milk]["pos"])
        original_z = position[2]
        destination = policy_destination(state, basket)
        yaw = state["objects"][milk]["yaw"]
        if attempt == 1:
            yaw += pi / 2
        yaw = policy_yaw(yaw, state["eef_yaw"])
        clearance = max(
            state["eef_pos"][2],
            position[2] + 0.19,
            destination[2] + 0.22,
        )

        policy_gripper(sdk, "open", 8)
        eef = sdk.get_state()["eef_pos"]
        if not policy_move(
            sdk, [eef[0], eef[1], clearance], yaw, "open", 25
        ):
            continue

        if not policy_move(
            sdk, [position[0], position[1], clearance], yaw, "open", 40
        ):
            continue

        state = sdk.get_state()
        position = list(state["objects"][milk]["pos"])
        grasp_height = position[2] + 0.015
        if attempt == 2:
            grasp_height = position[2] + 0.035

        if not policy_move(
            sdk,
            [position[0], position[1], grasp_height],
            yaw,
            "open",
            40,
            0.008,
        ):
            continue

        policy_gripper(sdk, "close", 14)
        before_lift = sdk.get_state()
        eef = before_lift["eef_pos"]
        object_before = before_lift["objects"][milk]["pos"]
        offset_before = [
            object_before[i] - eef[i] for i in range(3)
        ]

        policy_move(
            sdk,
            [eef[0], eef[1], eef[2] + 0.14],
            yaw,
            "close",
            35,
        )

        state = sdk.get_state()
        lifted = state["objects"][milk]["pos"]
        eef = state["eef_pos"]
        offset_after = [lifted[i] - eef[i] for i in range(3)]
        offset_error = sqrt(sum(
            (offset_after[i] - offset_before[i]) ** 2
            for i in range(3)
        ))

        if lifted[2] - original_z > 0.065 and offset_error < 0.045:
            grasped = True
            carry_yaw = yaw
            break

    if not grasped:
        return {"status": "failed", "reason": "Could not verify a grasp"}

    state = sdk.get_state()
    position = state["objects"][milk]["pos"]
    eef = state["eef_pos"]
    offset = [position[i] - eef[i] for i in range(3)]
    destination = policy_destination(state, basket)
    transit_z = max(eef[2], destination[2] + 0.24 - offset[2])

    if not policy_move(
        sdk, [eef[0], eef[1], transit_z], carry_yaw, "close", 30
    ):
        return {"status": "failed", "reason": "Could not reach clearance"}

    if not policy_move(
        sdk,
        [
            destination[0] - offset[0],
            destination[1] - offset[1],
            transit_z,
        ],
        carry_yaw,
        "close",
        55,
    ):
        return {"status": "failed", "reason": "Could not reach basket"}

    state = sdk.get_state()
    position = state["objects"][milk]["pos"]
    eef = state["eef_pos"]
    current_offset = [position[i] - eef[i] for i in range(3)]
    if sqrt(sum(
        (current_offset[i] - offset[i]) ** 2 for i in range(3)
    )) > 0.05:
        return {"status": "failed", "reason": "Milk slipped during transport"}

    destination = policy_destination(state, basket)
    release_center_z = destination[2] + 0.045
    release_position = [
        destination[0] - current_offset[0],
        destination[1] - current_offset[1],
        release_center_z - current_offset[2],
    ]

    if policy_remaining(sdk) < 45:
        return {"status": "failed", "reason": "Insufficient placement budget"}

    if not policy_move(
        sdk, release_position, carry_yaw, "close", 35, 0.01
    ):
        return {"status": "failed", "reason": "Could not lower milk into basket"}

    state = sdk.get_state()
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
    released = state["eef_pos"][2] - position[2] > 0.10

    if horizontal_error < 0.075 and -0.06 < vertical_error < 0.10 and released:
        return {"status": "done", "verification": "Milk released inside basket"}
    return {"status": "failed", "reason": "Final placement could not be verified"}
