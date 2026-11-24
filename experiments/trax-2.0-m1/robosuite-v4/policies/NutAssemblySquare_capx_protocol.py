def wrap_angle(angle):
    return atan2(sin(angle), cos(angle))


def square_error(yaw):
    return 0.25 * atan2(sin(-4.0 * yaw), cos(-4.0 * yaw))


def budget(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def move(sdk, position, yaw, grip, steps, tolerance):
    available = budget(sdk)
    if available > 0:
        return sdk.move_to(
            position,
            yaw=wrap_angle(yaw),
            gripper=grip,
            max_steps=min(steps, available),
            tol=tolerance,
        )
    return {"reached": False}


def grip(sdk, action, steps):
    available = budget(sdk)
    if available > 0:
        sdk.set_gripper(action, steps=min(steps, available))


def position_nut(sdk, target, steps, tolerance, turn_limit):
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    offset = [nut["pos"][i] - state["eef_pos"][i] for i in range(3)]
    turn = max(-turn_limit, min(turn_limit, square_error(nut["yaw"])))
    rotated = [
        cos(turn) * offset[0] - sin(turn) * offset[1],
        sin(turn) * offset[0] + cos(turn) * offset[1],
        offset[2],
    ]
    wrist = [target[i] - rotated[i] for i in range(3)]
    move(sdk, wrist, state["eef_yaw"] + turn, "close", steps, tolerance)


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or peg pose"}

    grip(sdk, "open", 12)
    state = sdk.get_state()
    resting_height = state["objects"]["nut"]["pos"][2]
    grasped = False

    for attempt in range(2):
        if budget(sdk) < 205:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        radius = 0.050 if attempt == 0 else 0.043
        yaw = nut["yaw"]
        grasp_yaw = state["eef_yaw"] + 0.5 * atan2(
            sin(2.0 * (yaw - state["eef_yaw"])),
            cos(2.0 * (yaw - state["eef_yaw"])),
        )
        handle = [
            nut["pos"][0] + radius * cos(yaw),
            nut["pos"][1] + radius * sin(yaw),
            nut["pos"][2] + 0.002,
        ]

        move(
            sdk,
            [handle[0], handle[1], handle[2] + 0.090],
            grasp_yaw,
            "open",
            42,
            0.006,
        )

        # Finish the descent before closing; a timed-out approach is not a grasp.
        ready = False
        for correction in range(3):
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            handle = [
                nut["pos"][0] + radius * cos(nut["yaw"]),
                nut["pos"][1] + radius * sin(nut["yaw"]),
                nut["pos"][2] + 0.002,
            ]
            move(
                sdk, handle, grasp_yaw, "open",
                42 if correction == 0 else 14, 0.004,
            )
            state = sdk.get_state()
            nut = state["objects"]["nut"]
            actual_handle = [
                nut["pos"][0] + radius * cos(nut["yaw"]),
                nut["pos"][1] + radius * sin(nut["yaw"]),
                nut["pos"][2] + 0.002,
            ]
            error = sqrt(sum(
                (state["eef_pos"][i] - actual_handle[i]) ** 2
                for i in range(3)
            ))
            if error < 0.008:
                ready = True
                break
            if budget(sdk) < 180:
                break

        if not ready:
            return {
                "status": "failed",
                "reason": "Gripper did not reach the live handle pose",
                "steps_used": state["steps_used"],
            }

        before_height = state["objects"]["nut"]["pos"][2]
        resting_height = min(resting_height, before_height)
        grip(sdk, "close", 16)
        state = sdk.get_state()
        lift = list(state["eef_pos"])
        lift[2] += 0.070
        move(sdk, lift, state["eef_yaw"], "close", 30, 0.006)

        state = sdk.get_state()
        if (
            state["objects"]["nut"]["pos"][2] - before_height > 0.035
            and state["gripper_width"] > 0.012
        ):
            grasped = True
            break

        grip(sdk, "open", 12)

    if not grasped:
        return {"status": "failed", "reason": "No verified lifted grasp"}

    state = sdk.get_state()
    goal = state["goal_pos"]
    clearance = max(resting_height + 0.125, goal[2] + 0.105)
    nut = state["objects"]["nut"]
    position_nut(
        sdk, [nut["pos"][0], nut["pos"][1], clearance],
        28, 0.006, 0.80,
    )

    aligned = False
    for iteration in range(4):
        if budget(sdk) < 65:
            break
        state = sdk.get_state()
        goal = state["goal_pos"]
        position_nut(
            sdk, [goal[0], goal[1], clearance],
            48 if iteration == 0 else 16, 0.003, 0.80,
        )
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        if (
            hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) < 0.005
            and abs(square_error(nut["yaw"])) < 0.055
            and abs(nut["pos"][2] - clearance) < 0.012
        ):
            aligned = True
            break

    if not aligned:
        return {"status": "failed", "reason": "Could not align above the peg"}

    seated_height = resting_height + 0.002
    stalled = 0
    for iteration in range(12):
        if budget(sdk) <= 32:
            break
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        goal = state["goal_pos"]
        lateral = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
        height = nut["pos"][2]

        if height < seated_height + 0.009 and lateral < 0.008:
            break

        target_height = max(seated_height, height - 0.025)
        if lateral > 0.006 or abs(square_error(nut["yaw"])) > 0.07:
            target_height = height

        position_nut(
            sdk, [goal[0], goal[1], target_height],
            min(14, budget(sdk) - 32), 0.003, 0.10,
        )
        after_height = sdk.get_state()["objects"]["nut"]["pos"][2]
        if target_height < height - 0.005 and height - after_height < 0.002:
            stalled += 1
        else:
            stalled = 0

        if stalled >= 2 and budget(sdk) > 60:
            state = sdk.get_state()
            goal = state["goal_pos"]
            position_nut(
                sdk, [goal[0], goal[1], after_height + 0.012],
                12, 0.003, 0.12,
            )
            stalled = 0

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    if (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) > 0.010
        or nut["pos"][2] > seated_height + 0.018
    ):
        return {
            "status": "failed",
            "reason": "Nut did not reach seated height",
            "steps_used": state["steps_used"],
        }

    grip(sdk, "open", 14)
    state = sdk.get_state()
    retreat = list(state["eef_pos"])
    retreat[2] += 0.085
    move(sdk, retreat, state["eef_yaw"], "open", 18, 0.010)

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    goal = state["goal_pos"]
    lateral = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    seated = (
        lateral < 0.012
        and abs(nut["pos"][2] - resting_height) < 0.020
        and abs(square_error(nut["yaw"])) < 0.12
    )
    return {
        "status": "done" if seated else "failed",
        "lateral_error_m": lateral,
        "steps_used": state["steps_used"],
    }
