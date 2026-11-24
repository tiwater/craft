def clip(value, low, high):
    return max(low, min(high, value))


def budget(state):
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def opened(state):
    return state["hinge_qpos"] is not None and state["hinge_qpos"] > 0.30


def distance(a, b):
    return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def control(sdk, offset, displacement, rotation, ticks):
    state = sdk.get_state()
    for tick in range(min(ticks, budget(state))):
        if opened(state):
            break
        handle = state["objects"]["handle"]["pos"]
        eef = state["eef_pos"]
        action = [
            clip(
                20.0 * (handle[i] + offset[i] + displacement[i] - eef[i]),
                -1.0,
                1.0,
            )
            for i in range(3)
        ]
        action.extend(rotation)
        action.append(1.0)
        sdk.step_raw(action, steps=1)
        state = sdk.get_state()
    return state


def run_policy(sdk):
    state = sdk.get_state()
    if "handle" not in state["objects"] or "door" not in state["objects"]:
        return {"status": "failed", "reason": "Missing door observations"}

    peak = state["hinge_qpos"]
    starting_yaw = state["eef_yaw"]
    best_axis = None

    for attempt in range(3):
        state = sdk.get_state()
        if opened(state) or budget(state) < 40:
            break

        handle = list(state["objects"]["handle"]["pos"])
        door_yaw = state["objects"]["door"]["yaw"]
        grasp_yaw = starting_yaw if attempt != 1 else starting_yaw + pi / 2.0

        sdk.set_gripper("open", steps=min(8, budget(state)))
        state = sdk.get_state()
        sdk.move_to(
            [handle[0], handle[1], handle[2] + 0.075],
            yaw=grasp_yaw,
            gripper="open",
            max_steps=min(30, budget(state)),
            tol=0.012,
        )
        state = sdk.get_state()
        if opened(state) or budget(state) < 15:
            break

        handle = list(state["objects"]["handle"]["pos"])
        sdk.move_to(
            [handle[0], handle[1], handle[2] - 0.006],
            yaw=grasp_yaw,
            gripper="open",
            max_steps=min(32, budget(state) - 10),
            tol=0.008,
        )
        state = sdk.get_state()
        if opened(state):
            break
        if distance(state["eef_pos"], state["objects"]["handle"]["pos"]) > 0.045:
            continue

        sdk.set_gripper("close", steps=min(10, budget(state)))
        state = sdk.get_state()
        offset = [
            clip(
                state["eef_pos"][i] - state["objects"]["handle"]["pos"][i],
                -0.02,
                0.02,
            )
            for i in range(3)
        ]
        normal = [-sin(door_yaw), cos(door_yaw), 0.0]
        along = [cos(door_yaw), sin(door_yaw), 0.0]
        axes = [
            normal,
            [-v for v in normal],
            along,
            [-v for v in along],
        ]

        if best_axis is None:
            best_gain = 0.0
            for axis in axes:
                if opened(state) or budget(state) < 12:
                    break
                before = abs(state["handle_qpos"])
                state = control(
                    sdk, offset, [0.0, 0.0, -0.008],
                    [0.45 * v for v in axis], 7
                )
                gain = abs(state["handle_qpos"]) - before
                if gain > best_gain:
                    best_gain = gain
                    best_axis = list(axis)
                if gain > 0.035:
                    break
                state = control(
                    sdk, offset, [0.0, 0.0, 0.0],
                    [-0.35 * v for v in axis], 4
                )

        if best_axis is None:
            best_axis = list(normal)

        latch_ticks = 0
        while budget(state) > 0 and latch_ticks < 32:
            if opened(state) or abs(state["handle_qpos"]) > 0.65:
                break
            state = control(
                sdk, offset, [0.0, 0.0, -0.008],
                [0.45 * v for v in best_axis], 4
            )
            latch_ticks += 4

        handle = state["objects"]["handle"]["pos"]
        goal = state["goal_pos"]
        if goal is not None:
            dx = goal[0] - handle[0]
            dy = goal[1] - handle[1]
            length = max(0.001, hypot(dx, dy))
            preferred = [dx / length, dy / length, 0.0]
        else:
            preferred = [-normal[0], -normal[1], 0.0]

        directions = [
            preferred,
            normal,
            [-v for v in normal],
            [-v for v in along],
            along,
            [-v for v in preferred],
        ]
        direction_index = 0
        winning_direction = None
        stalled = 0
        contact_ticks = 0

        while budget(state) > 0 and contact_ticks < 210:
            if opened(state):
                break
            peak = max(peak, state["hinge_qpos"])
            gap = distance(state["eef_pos"], state["objects"]["handle"]["pos"])
            if gap > 0.115:
                break
            if state["gripper_width"] < 0.009 and gap > 0.045:
                break

            direction = (
                winning_direction
                if winning_direction is not None
                else directions[direction_index % len(directions)]
            )
            before_hinge = state["hinge_qpos"]
            before_handle = list(state["objects"]["handle"]["pos"])
            latch = abs(state["handle_qpos"])

            if before_hinge < 0.07:
                torque = 0.42 if latch < 0.65 else 0.14
            else:
                torque = 0.0

            displacement = [
                0.09 * direction[0],
                0.09 * direction[1],
                -0.004,
            ]
            if latch < 0.32 and before_hinge < 0.025:
                displacement = [0.025 * direction[0], 0.025 * direction[1], -0.008]

            state = control(
                sdk, offset, displacement,
                [torque * v for v in best_axis], 10
            )
            contact_ticks += 10
            gain = state["hinge_qpos"] - before_hinge

            if gain > 0.002:
                after_handle = state["objects"]["handle"]["pos"]
                dx = after_handle[0] - before_handle[0]
                dy = after_handle[1] - before_handle[1]
                length = hypot(dx, dy)
                if length > 0.003 and gain > 0.008:
                    winning_direction = [dx / length, dy / length, 0.0]
                else:
                    winning_direction = list(direction)
                stalled = 0
            else:
                winning_direction = None
                direction_index += 1
                stalled += 1
                state = control(
                    sdk, offset, [0.0, 0.0, -0.004],
                    [0.22 * v for v in best_axis], 3
                )
                contact_ticks += 3

            if stalled >= 7:
                break

    state = sdk.get_state()
    return {
        "status": "done" if opened(state) else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "peak_hinge_qpos": max(peak, state["hinge_qpos"]),
        "steps_used": state["steps_used"],
    }
