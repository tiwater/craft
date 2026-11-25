def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def distance(a, b):
        return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    def move(position, yaw, grip, budget, tolerance=0.015):
        budget = min(budget, remaining())
        while budget > 0:
            state = sdk.get_state()
            if distance(state["eef_pos"], position) <= tolerance:
                return True
            before = state["steps_used"]
            sdk.move_to(
                position,
                yaw=yaw,
                gripper=grip,
                max_steps=min(25, budget),
                tol=tolerance,
            )
            state = sdk.get_state()
            used = state["steps_used"] - before
            if distance(state["eef_pos"], position) <= tolerance:
                return True
            if used <= 0:
                break
            budget -= used
        return False

    def grip(action, steps):
        if remaining() > 0:
            sdk.set_gripper(action, steps=min(steps, remaining()))

    def nearby_yaw(target, current):
        while target - current > pi / 2:
            target -= pi
        while target - current < -pi / 2:
            target += pi
        return target

    def destination(state, basket):
        for name in (basket + "_contain_region", basket + "_inside_region"):
            if name in state["sites"]:
                return list(state["sites"][name])
        if basket in state["objects"]:
            position = state["objects"][basket]["pos"]
        else:
            position = state["fixtures"][basket]["pos"]
        return [position[0], position[1], position[2] + 0.07]

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
        return {"status": "failed", "reason": "Required scene entity missing"}

    grasped = False
    offset = [0.0, 0.0, 0.0]
    carry_yaw = state["eef_yaw"]

    for attempt in range(2):
        if remaining() < 210:
            break

        grip("open", 8)
        state = sdk.get_state()
        position = list(state["objects"][milk]["pos"])
        eef = list(state["eef_pos"])
        travel_yaw = state["eef_yaw"]
        clearance = max(eef[2], position[2] + 0.18)

        # Translate with the existing wrist orientation. Large simultaneous
        # yaw changes can stall an otherwise reachable overhead approach.
        if not move(
            [eef[0], eef[1], clearance],
            travel_yaw, "open", 65, 0.02
        ):
            return {"status": "failed", "reason": "Clearance motion stalled"}

        position = list(sdk.get_state()["objects"][milk]["pos"])
        approach = [position[0], position[1], clearance]
        reached = move(approach, travel_yaw, "open", 65, 0.015)

        if not reached and remaining() > 245:
            state = sdk.get_state()
            eef = list(state["eef_pos"])
            travel_yaw = state["eef_yaw"]
            clearance = max(clearance, eef[2]) + 0.045
            move(
                [eef[0], eef[1], clearance],
                travel_yaw, "open", 20, 0.02
            )
            position = list(sdk.get_state()["objects"][milk]["pos"])
            reached = move(
                [position[0], position[1], clearance],
                travel_yaw, "open", 35, 0.018
            )

        if not reached:
            return {"status": "failed", "reason": "Milk approach stalled"}

        state = sdk.get_state()
        current_yaw = state["eef_yaw"]
        object_yaw = state["objects"][milk]["yaw"]
        yaw_a = nearby_yaw(object_yaw, current_yaw)
        yaw_b = nearby_yaw(object_yaw + pi / 2, current_yaw)
        if abs(yaw_a - current_yaw) <= abs(yaw_b - current_yaw):
            yaw = yaw_a if attempt == 0 else yaw_b
        else:
            yaw = yaw_b if attempt == 0 else yaw_a

        # Align separately above the object, selecting the nearer grasp axis.
        if remaining() > 185:
            sdk.move_to(
                list(state["eef_pos"]),
                yaw=yaw,
                gripper="open",
                max_steps=min(18, remaining()),
                tol=0.01,
            )

        state = sdk.get_state()
        yaw = state["eef_yaw"]
        position = list(state["objects"][milk]["pos"])
        original_z = position[2]
        grasp_height = position[2] + 0.015 + 0.01 * attempt
        descended = move(
            [position[0], position[1], grasp_height],
            yaw, "open", 50, 0.01
        )
        if not descended:
            state = sdk.get_state()
            position = state["objects"][milk]["pos"]
            eef = state["eef_pos"]
            if (
                hypot(eef[0] - position[0], eef[1] - position[1]) > 0.018
                or abs(eef[2] - grasp_height) > 0.025
            ):
                continue

        grip("close", 16)
        state = sdk.get_state()
        eef = list(state["eef_pos"])
        position = list(state["objects"][milk]["pos"])
        before_offset = [position[i] - eef[i] for i in range(3)]
        move(
            [eef[0], eef[1], eef[2] + 0.13],
            yaw, "close", 40, 0.015
        )
        state = sdk.get_state()
        position = state["objects"][milk]["pos"]
        eef = state["eef_pos"]
        offset = [position[i] - eef[i] for i in range(3)]
        if (
            position[2] > original_z + 0.06
            and distance(offset, before_offset) < 0.04
        ):
            grasped = True
            carry_yaw = state["eef_yaw"]
            break

    if not grasped:
        return {"status": "failed", "reason": "Milk did not follow the lift"}

    state = sdk.get_state()
    target = destination(state, basket)
    eef = list(state["eef_pos"])
    transit_z = max(eef[2], target[2] + 0.18 - offset[2])

    if not move(
        [eef[0], eef[1], transit_z],
        carry_yaw, "close", min(40, max(0, remaining() - 110)), 0.02
    ):
        return {"status": "failed", "reason": "Transport clearance stalled"}

    state = sdk.get_state()
    target = destination(state, basket)
    if not move(
        [target[0] - offset[0], target[1] - offset[1], transit_z],
        carry_yaw, "close", min(70, max(0, remaining() - 65)), 0.012
    ):
        return {"status": "failed", "reason": "Basket approach stalled"}

    state = sdk.get_state()
    position = state["objects"][milk]["pos"]
    eef = state["eef_pos"]
    current_offset = [position[i] - eef[i] for i in range(3)]
    if distance(current_offset, offset) > 0.05:
        return {"status": "failed", "reason": "Milk slipped during transport"}

    if remaining() < 45:
        return {"status": "failed", "reason": "Insufficient release budget"}

    target = destination(state, basket)
    move(
        [
            target[0] - current_offset[0],
            target[1] - current_offset[1],
            target[2] + 0.055 - current_offset[2],
        ],
        carry_yaw, "close", min(50, remaining() - 35), 0.012
    )

    state = sdk.get_state()
    position = state["objects"][milk]["pos"]
    target = destination(state, basket)
    if (
        hypot(position[0] - target[0], position[1] - target[1]) > 0.04
        or position[2] > target[2] + 0.11
    ):
        return {"status": "failed", "reason": "Unsafe release alignment"}

    grip("open", 12)
    state = sdk.get_state()
    eef = list(state["eef_pos"])
    move(
        [eef[0], eef[1], eef[2] + 0.16],
        carry_yaw, "open", min(30, max(0, remaining() - 10)), 0.02
    )
    if remaining() > 0:
        sdk.step_raw(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
            steps=min(20, remaining()),
        )

    state = sdk.get_state()
    position = state["objects"][milk]["pos"]
    target = destination(state, basket)
    if (
        hypot(position[0] - target[0], position[1] - target[1]) < 0.075
        and -0.06 < position[2] - target[2] < 0.10
        and state["gripper_width"] > 0.04
        and state["eef_pos"][2] - position[2] > 0.10
    ):
        return {"status": "done"}
    return {"status": "failed", "reason": "Final placement not verified"}
