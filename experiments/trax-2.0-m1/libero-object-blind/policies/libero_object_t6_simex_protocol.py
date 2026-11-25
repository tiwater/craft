def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return state["max_steps"] - state["steps_used"]

    def move(pos, yaw, action, limit=50, tol=0.009):
        budget = remaining()
        if budget <= 0:
            return False
        result = sdk.move_to(
            pos, yaw=yaw, gripper=action,
            max_steps=min(limit, budget), tol=tol
        )
        return result["reached"]

    def grip(action, steps=12):
        budget = remaining()
        if budget <= 0:
            return False
        sdk.set_gripper(action, steps=min(steps, budget))
        return True

    def resolve(mapping, stem):
        if stem + "_1" in mapping:
            return stem + "_1"
        if stem in mapping:
            return stem
        matches = []
        for name in mapping:
            if name.startswith(stem + "_"):
                suffix = name[len(stem) + 1:]
                if suffix.isdigit():
                    matches.append(name)
        if len(matches) == 1:
            return matches[0]
        return None

    def basket_target(state, basket, group):
        sites = state["sites"]
        for name in (
            basket + "_contain_region",
            basket + "_contain",
            basket + "_inside_region",
        ):
            if name in sites:
                return list(sites[name]), name
        matches = []
        for name in sites:
            if name.startswith(basket + "_") and "contain" in name:
                matches.append(name)
        if len(matches) == 1:
            return list(sites[matches[0]]), matches[0]
        pos = state[group][basket]["pos"]
        return [pos[0], pos[1], pos[2] + 0.075], None

    state = sdk.get_state()
    butter = resolve(state["objects"], "butter")
    basket = resolve(state["objects"], "basket")
    basket_group = "objects"
    if basket is None:
        basket = resolve(state["fixtures"], "basket")
        basket_group = "fixtures"
    if butter is None or basket is None:
        return {"status": "failed", "reason": "Ambiguous butter or basket"}

    held = False
    offset = [0.0, 0.0, 0.0]
    grasp_yaw = state["eef_yaw"]
    diagnostics = []

    for attempt in range(3):
        if remaining() < 185:
            break

        state = sdk.get_state()
        pos = list(state["objects"][butter]["pos"])
        current_yaw = state["eef_yaw"]
        desired_yaw = state["objects"][butter]["yaw"]
        if attempt == 1:
            desired_yaw += pi / 2.0
        grasp_yaw = current_yaw + 0.5 * atan2(
            sin(2.0 * (desired_yaw - current_yaw)),
            cos(2.0 * (desired_yaw - current_yaw)),
        )

        grip("open", 6)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        clearance = pos[2] + 0.15
        if eef[2] < clearance - 0.015:
            move(
                [eef[0], eef[1], clearance],
                current_yaw, "open", 25, 0.012
            )

        state = sdk.get_state()
        pos = list(state["objects"][butter]["pos"])
        approach = [pos[0], pos[1], clearance]
        move(approach, current_yaw, "open", 65, 0.012)

        state = sdk.get_state()
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - approach[0], eef[1] - approach[1]) > 0.018
            or abs(eef[2] - approach[2]) > 0.025
        ):
            move(approach, current_yaw, "open", 30, 0.012)

        move(approach, grasp_yaw, "open", 25, 0.012)
        state = sdk.get_state()
        pos = list(state["objects"][butter]["pos"])
        eef = state["eef_pos"]
        if hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.022:
            diagnostics.append("Pregrasp alignment failed")
            continue

        move(
            [pos[0], pos[1], pos[2] + 0.055],
            grasp_yaw, "open", 25, 0.008
        )
        state = sdk.get_state()
        pos = list(state["objects"][butter]["pos"])
        height = pos[2] + (0.006 if attempt != 2 else 0.0)
        move(
            [pos[0], pos[1], height],
            grasp_yaw, "open", 35, 0.005
        )

        state = sdk.get_state()
        pos = list(state["objects"][butter]["pos"])
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.010
            or abs(eef[2] - height) > 0.012
        ):
            move(
                [pos[0], pos[1], height],
                grasp_yaw, "open", 15, 0.005
            )

        state = sdk.get_state()
        pos = list(state["objects"][butter]["pos"])
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - pos[0], eef[1] - pos[1]) > 0.014
            or abs(eef[2] - height) > 0.018
        ):
            diagnostics.append("Contact alignment failed")
            continue

        before = list(pos)
        grip("close", 20)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        move(
            [eef[0], eef[1], eef[2] + 0.045],
            grasp_yaw, "close", 20, 0.007
        )

        state = sdk.get_state()
        pos = state["objects"][butter]["pos"]
        eef = state["eef_pos"]
        if (
            pos[2] - before[2] < 0.020
            or hypot(pos[0] - eef[0], pos[1] - eef[1]) > 0.055
            or state["gripper_width"] < 0.008
        ):
            diagnostics.append("Test lift failed")
            continue

        move(
            [eef[0], eef[1], before[2] + 0.13],
            grasp_yaw, "close", 25, 0.009
        )
        state = sdk.get_state()
        pos = state["objects"][butter]["pos"]
        eef = state["eef_pos"]
        if (
            pos[2] - before[2] > 0.065
            and hypot(pos[0] - eef[0], pos[1] - eef[1]) < 0.055
            and state["gripper_width"] > 0.008
        ):
            held = True
            offset = [pos[i] - eef[i] for i in range(3)]
            break
        diagnostics.append("Full lift failed")

    if not held:
        return {
            "status": "failed",
            "reason": "Could not verify butter grasp",
            "diagnostics": diagnostics,
            "steps_used": sdk.get_state()["steps_used"],
        }

    state = sdk.get_state()
    target, target_site = basket_target(state, basket, basket_group)
    clearance = max(
        state["eef_pos"][2],
        target[2] + 0.14 - offset[2],
        max(obj["pos"][2] for obj in state["objects"].values()) + 0.14,
    )
    eef = state["eef_pos"]
    if not move([eef[0], eef[1], clearance], grasp_yaw, "close", 30):
        return {"status": "failed", "reason": "Transport clearance unreachable"}

    if not move(
        [target[0] - offset[0], target[1] - offset[1], clearance],
        grasp_yaw, "close", 65, 0.010
    ):
        return {"status": "failed", "reason": "Basket approach unreachable"}

    state = sdk.get_state()
    pos = state["objects"][butter]["pos"]
    eef = state["eef_pos"]
    if any(abs(pos[i] - eef[i] - offset[i]) > 0.035 for i in range(3)):
        return {"status": "failed", "reason": "Butter slipped during transport"}

    offset = [pos[i] - eef[i] for i in range(3)]
    target, target_site = basket_target(state, basket, basket_group)
    release = [
        target[0] - offset[0],
        target[1] - offset[1],
        target[2] + 0.035 - offset[2],
    ]
    if remaining() < 40:
        return {"status": "failed", "reason": "Insufficient placement budget"}
    if not move(
        release, grasp_yaw, "close",
        min(40, remaining() - 25), 0.009
    ):
        return {"status": "failed", "reason": "Release position unreachable"}
    grip("open", 12)

    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.13],
        grasp_yaw, "open", 25, 0.012
    )
    budget = remaining()
    if budget > 0:
        sdk.step_raw(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
            steps=min(18, budget),
        )

    state = sdk.get_state()
    target, target_site = basket_target(state, basket, basket_group)
    pos = state["objects"][butter]["pos"]
    base = state[basket_group][basket]["pos"]
    placed = (
        hypot(pos[0] - target[0], pos[1] - target[1]) < 0.055
        and base[2] - 0.01 <= pos[2] <= target[2] + 0.065
        and state["eef_pos"][2] - pos[2] > 0.07
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Live placement verified" if placed else "Placement unverified",
        "butter": butter,
        "basket": basket,
        "target_site": target_site,
        "target": target,
        "butter_pos": list(pos),
        "steps_used": state["steps_used"],
    }
