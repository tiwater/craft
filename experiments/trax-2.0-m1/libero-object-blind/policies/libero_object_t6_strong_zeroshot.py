def policy_move(sdk, pos, yaw, grip, limit=55, tol=0.012):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    result = sdk.move_to(
        pos,
        yaw=yaw,
        gripper=grip,
        max_steps=min(limit, remaining),
        tol=tol,
    )
    return result["reached"]


def policy_grip(sdk, action, steps=12):
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining <= 0:
        return False
    sdk.set_gripper(action, steps=min(steps, remaining))
    return True


def policy_target(state, basket):
    sites = state["sites"]
    preferred = basket + "_contain_region"
    if preferred in sites:
        return list(sites[preferred])
    for name in sites:
        if basket in name and "contain" in name:
            return list(sites[name])
    pos = state["objects"][basket]["pos"]
    return [pos[0], pos[1], pos[2] + 0.075]


def run_policy(sdk):
    state = sdk.get_state()
    butter = None
    basket = None
    for name in state["objects"]:
        if "butter" in name:
            butter = name
        if "basket" in name:
            basket = name
    if butter is None or basket is None:
        return {"status": "failed", "reason": "Required objects unavailable"}

    held = False
    offset = [0.0, 0.0, 0.0]
    grasp_yaw = state["eef_yaw"]

    for attempt in range(3):
        state = sdk.get_state()
        remaining = state["max_steps"] - state["steps_used"]
        if remaining < 155:
            break

        obj = state["objects"][butter]
        pos = list(obj["pos"])
        yaw = obj["yaw"] + (pi / 2 if attempt == 1 else 0.0)
        current_yaw = state["eef_yaw"]
        yaw = current_yaw + atan2(
            sin(2.0 * (yaw - current_yaw)),
            cos(2.0 * (yaw - current_yaw)),
        ) / 2.0
        grasp_yaw = yaw

        policy_grip(sdk, "open", 8)
        clearance = max(state["eef_pos"][2], pos[2] + 0.15)
        if state["eef_pos"][2] < pos[2] + 0.10:
            policy_move(
                sdk,
                [state["eef_pos"][0], state["eef_pos"][1], clearance],
                yaw,
                "open",
                25,
            )

        if not policy_move(
            sdk, [pos[0], pos[1], clearance], yaw, "open", 45
        ):
            continue

        state = sdk.get_state()
        pos = list(state["objects"][butter]["pos"])
        grasp_height = pos[2] + (0.008 if attempt == 2 else 0.018)
        if not policy_move(
            sdk,
            [pos[0], pos[1], grasp_height],
            yaw,
            "open",
            40,
            0.006,
        ):
            continue

        before = list(sdk.get_state()["objects"][butter]["pos"])
        policy_grip(sdk, "close", 15)
        state = sdk.get_state()
        eef = state["eef_pos"]
        policy_move(
            sdk,
            [eef[0], eef[1], eef[2] + 0.13],
            yaw,
            "close",
            35,
            0.008,
        )
        state = sdk.get_state()
        lifted = state["objects"][butter]["pos"]
        eef = state["eef_pos"]
        if (
            lifted[2] - before[2] > 0.065
            and hypot(lifted[0] - eef[0], lifted[1] - eef[1]) < 0.065
        ):
            held = True
            offset = [lifted[i] - eef[i] for i in range(3)]
            break

    if not held:
        return {"status": "failed", "reason": "Could not verify a grasp"}

    state = sdk.get_state()
    target = policy_target(state, basket)
    clearance = max(
        state["eef_pos"][2],
        target[2] + 0.14 - offset[2],
        max(obj["pos"][2] for obj in state["objects"].values()) + 0.15,
    )
    eef = state["eef_pos"]
    if not policy_move(
        sdk, [eef[0], eef[1], clearance], grasp_yaw, "close", 35
    ):
        return {"status": "failed", "reason": "Could not reach transport height"}

    above = [target[0] - offset[0], target[1] - offset[1], clearance]
    if not policy_move(sdk, above, grasp_yaw, "close", 65, 0.009):
        return {"status": "failed", "reason": "Could not reach basket"}

    state = sdk.get_state()
    obj = state["objects"][butter]["pos"]
    eef = state["eef_pos"]
    if any(abs(obj[i] - eef[i] - offset[i]) > 0.045 for i in range(3)):
        return {"status": "failed", "reason": "Object slipped during transport"}

    target = policy_target(state, basket)
    release = [
        target[0] - offset[0],
        target[1] - offset[1],
        target[2] + 0.025 - offset[2],
    ]
    if not policy_move(sdk, release, grasp_yaw, "close", 45, 0.008):
        return {"status": "failed", "reason": "Could not reach release position"}

    if not policy_grip(sdk, "open", 15):
        return {"status": "failed", "reason": "Step budget exhausted before release"}

    state = sdk.get_state()
    eef = state["eef_pos"]
    policy_move(
        sdk,
        [eef[0], eef[1], eef[2] + 0.14],
        grasp_yaw,
        "open",
        30,
    )
    state = sdk.get_state()
    remaining = state["max_steps"] - state["steps_used"]
    if remaining > 0:
        sdk.step_raw(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
            steps=min(20, remaining),
        )

    state = sdk.get_state()
    target = policy_target(state, basket)
    pos = state["objects"][butter]["pos"]
    base = state["objects"][basket]["pos"]
    placed = (
        hypot(pos[0] - target[0], pos[1] - target[1]) < 0.055
        and base[2] - 0.01 <= pos[2] <= target[2] + 0.065
        and state["eef_pos"][2] - pos[2] > 0.07
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Placement checked from live poses"
        if placed
        else "Final placement could not be verified",
        "steps_used": state["steps_used"],
    }
