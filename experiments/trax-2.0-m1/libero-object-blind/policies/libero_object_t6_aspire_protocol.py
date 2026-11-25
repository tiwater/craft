def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, state["max_steps"] - state["steps_used"])

    def move(pos, yaw, action, steps=30, tol=0.012):
        budget = remaining()
        if budget <= 0:
            return False
        sdk.move_to(
            pos, yaw=yaw, gripper=action,
            max_steps=min(steps, budget), tol=tol
        )
        state = sdk.get_state()
        return sqrt(sum(
            (state["eef_pos"][i] - pos[i]) ** 2 for i in range(3)
        )) <= max(0.018, tol * 1.8)

    def grip(action, steps=12):
        budget = remaining()
        if budget > 0:
            return sdk.set_gripper(action, steps=min(steps, budget))
        return {"gripper_width": 0.0, "grasped_hint": False}

    def basket_pose(state):
        if basket in state["objects"]:
            return state["objects"][basket]
        return state["fixtures"][basket]

    def destination(state):
        preferred = basket + "_contain_region"
        if preferred in state["sites"]:
            return list(state["sites"][preferred])
        for name in state["sites"]:
            if basket in name and "contain" in name:
                return list(state["sites"][name])
        pos = basket_pose(state)["pos"]
        return [pos[0], pos[1], pos[2] + 0.075]

    state = sdk.get_state()
    butter = None
    basket = None
    for name in state["objects"]:
        if "butter" in name:
            butter = name
        if "basket" in name:
            basket = name
    if basket is None:
        for name in state["fixtures"]:
            if "basket" in name:
                basket = name
    if butter is None or basket is None:
        return {"status": "failed", "reason": "Required poses unavailable"}

    held = False
    offset = [0.0, 0.0, 0.0]
    grasp_yaw = state["eef_yaw"]

    # Try the alternative jaw axis first. A small transient lift alone
    # does not establish a secure grasp.
    for attempt in range(3):
        if remaining() < 175:
            break

        state = sdk.get_state()
        pos = list(state["objects"][butter]["pos"])
        eef = list(state["eef_pos"])
        current_yaw = state["eef_yaw"]
        desired_yaw = state["objects"][butter]["yaw"]
        if attempt == 1:
            desired_yaw += pi / 2.0
        grasp_yaw = current_yaw + 0.5 * atan2(
            sin(2.0 * (desired_yaw - current_yaw)),
            cos(2.0 * (desired_yaw - current_yaw))
        )

        grip("open", 8)
        hover_z = max(eef[2], pos[2] + 0.105)
        if eef[2] < pos[2] + 0.065:
            move(
                [eef[0], eef[1], hover_z],
                current_yaw, "open", 18
            )

        move([pos[0], pos[1], hover_z], grasp_yaw, "open", 30)
        state = sdk.get_state()
        pos = list(state["objects"][butter]["pos"])
        eef = list(state["eef_pos"])
        if hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.022:
            continue

        depth = -0.006 if attempt < 2 else -0.014
        move(
            [pos[0], pos[1], pos[2] + depth],
            grasp_yaw, "open", 28, 0.007
        )
        state = sdk.get_state()
        pos = list(state["objects"][butter]["pos"])
        eef = list(state["eef_pos"])

        if hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.012:
            move(
                [pos[0], pos[1], pos[2] + depth],
                grasp_yaw, "open", 10, 0.007
            )
            state = sdk.get_state()
            pos = list(state["objects"][butter]["pos"])
            eef = list(state["eef_pos"])

        if (hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.022
                or abs(eef[2] - pos[2] - depth) > 0.030):
            continue

        before = list(pos)
        grip("close", 18)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        start_offset = [
            state["objects"][butter]["pos"][i] - eef[i]
            for i in range(3)
        ]

        # Seat the fingers with a short lift before committing to transport.
        move(
            [eef[0], eef[1], eef[2] + 0.030],
            grasp_yaw, "close", 12, 0.006
        )
        state = sdk.get_state()
        pos = list(state["objects"][butter]["pos"])
        eef = list(state["eef_pos"])
        followed = (
            pos[2] - before[2] > 0.010
            and max(
                abs(pos[i] - eef[i] - start_offset[i])
                for i in range(3)
            ) < 0.025
        )
        if not followed:
            grip("open", 6)
            continue

        grip("close", 6)
        move(
            [eef[0], eef[1], eef[2] + 0.075],
            grasp_yaw, "close", 22, 0.010
        )
        state = sdk.get_state()
        lifted = list(state["objects"][butter]["pos"])
        eef = list(state["eef_pos"])
        if (lifted[2] - before[2] > 0.055
                and hypot(lifted[0] - eef[0], lifted[1] - eef[1]) < 0.055
                and max(
                    abs(lifted[i] - eef[i] - start_offset[i])
                    for i in range(3)
                ) < 0.030):
            held = True
            offset = [lifted[i] - eef[i] for i in range(3)]
            break
        grip("open", 6)

    if not held:
        return {
            "status": "failed",
            "reason": "No sustained lift",
            "steps_used": sdk.get_state()["steps_used"]
        }

    state = sdk.get_state()
    target = destination(state)
    eef = list(state["eef_pos"])
    obstacle_z = max(
        state["objects"][name]["pos"][2] for name in state["objects"]
    )
    clearance = max(
        eef[2],
        target[2] + 0.13 - offset[2],
        obstacle_z + 0.13 - min(offset[2], 0.0)
    )
    if not move(
        [eef[0], eef[1], clearance], grasp_yaw, "close", 28
    ):
        return {"status": "failed", "reason": "Clearance unreachable"}

    state = sdk.get_state()
    target = destination(state)
    if not move(
        [target[0] - offset[0], target[1] - offset[1], clearance],
        grasp_yaw, "close", 55, 0.010
    ):
        return {"status": "failed", "reason": "Basket approach unreachable"}

    state = sdk.get_state()
    pos = list(state["objects"][butter]["pos"])
    eef = list(state["eef_pos"])
    if max(
        abs(pos[i] - eef[i] - offset[i]) for i in range(3)
    ) > 0.035:
        return {"status": "failed", "reason": "Grasp lost during transport"}

    offset = [pos[i] - eef[i] for i in range(3)]
    target = destination(state)
    release = [
        target[0] - offset[0],
        target[1] - offset[1],
        target[2] + 0.045 - offset[2]
    ]
    if remaining() < 42:
        return {"status": "failed", "reason": "Insufficient placement budget"}
    move(
        release, grasp_yaw, "close",
        min(32, remaining() - 30), 0.010
    )

    state = sdk.get_state()
    target = destination(state)
    pos = list(state["objects"][butter]["pos"])
    if (hypot(pos[0] - target[0], pos[1] - target[1]) > 0.035
            or abs(pos[2] - target[2] - 0.045) > 0.045):
        return {"status": "failed", "reason": "Release alignment unverified"}

    grip("open", 12)
    state = sdk.get_state()
    eef = list(state["eef_pos"])
    move(
        [eef[0], eef[1], eef[2] + 0.12],
        grasp_yaw, "open", 24
    )
    if remaining() > 0:
        sdk.step_raw(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
            steps=min(18, remaining())
        )

    state = sdk.get_state()
    target = destination(state)
    pos = state["objects"][butter]["pos"]
    base = basket_pose(state)["pos"]
    placed = (
        hypot(pos[0] - target[0], pos[1] - target[1]) < 0.055
        and base[2] - 0.015 <= pos[2] <= target[2] + 0.065
        and state["eef_pos"][2] - pos[2] > 0.065
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Placement verified" if placed else "Placement unverified",
        "steps_used": state["steps_used"]
    }
