def angle_wrap(value):
    return atan2(sin(value), cos(value))


def square_error(target, current):
    return atan2(sin(4.0 * (target - current)),
                 cos(4.0 * (target - current))) / 4.0


def budget(sdk):
    state = sdk.get_state()
    return max(0, min(400, state["max_steps"]) - state["steps_used"])


def actuate(sdk, action, steps):
    count = min(steps, budget(sdk))
    if count > 0:
        sdk.set_gripper(action, steps=count)


def servo(sdk, target, yaw, grip, steps=36, tolerance=0.004):
    # Correct persistent tracking error against the actual requested position.
    bias = [0.0, 0.0, 0.0]
    used = 0
    while used < steps and budget(sdk) > 0:
        count = min(12, steps - used, budget(sdk))
        command = [target[i] + bias[i] for i in range(3)]
        sdk.move_to(command, yaw=yaw, gripper=grip,
                    max_steps=count, tol=0.002)
        used += count
        state = sdk.get_state()
        error = [target[i] - state["eef_pos"][i] for i in range(3)]
        distance = sqrt(sum(value * value for value in error))
        if distance < tolerance:
            return True
        # Apply compensation only near the target, after the gross motion.
        if distance < 0.045:
            for i in range(3):
                bias[i] = max(-0.025, min(0.025, bias[i] + 0.8 * error[i]))
    return False


def peg_angle(state):
    for name in ("square_peg", "peg_square", "peg1", "peg"):
        if name in state["objects"]:
            return state["objects"][name]["yaw"]
    return 0.0


def position_hole(sdk, target, peg_yaw, steps=36):
    used = 0
    while used < steps and budget(sdk) > 0:
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        hole = nut["pos"]
        eef = state["eef_pos"]
        rotation = square_error(peg_yaw, nut["yaw"])
        dx = eef[0] - hole[0]
        dy = eef[1] - hole[1]
        destination = [
            target[0] + cos(rotation) * dx - sin(rotation) * dy,
            target[1] + sin(rotation) * dx + cos(rotation) * dy,
            target[2] + eef[2] - hole[2],
        ]
        count = min(18, steps - used, budget(sdk))
        servo(sdk, destination, angle_wrap(state["eef_yaw"] + rotation),
              "close", count, 0.003)
        used += count
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        if (
            hypot(nut["pos"][0] - target[0],
                  nut["pos"][1] - target[1]) < 0.0035
            and abs(nut["pos"][2] - target[2]) < 0.005
            and abs(square_error(peg_yaw, nut["yaw"])) < 0.04
        ):
            return True
    return False


def run_policy(sdk):
    state = sdk.get_state()
    if "nut" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing nut or goal"}

    goal = list(state["goal_pos"])
    rest_z = state["objects"]["nut"]["pos"][2]
    peg_yaw = peg_angle(state)
    clearance = max(rest_z + 0.115, goal[2] + 0.085)
    grasped = False

    for attempt in range(2):
        if attempt > 0 and budget(sdk) < 235:
            break

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        yaw = nut["yaw"]
        handle = [
            nut["pos"][0] + 0.055 * cos(yaw),
            nut["pos"][1] + 0.055 * sin(yaw),
            nut["pos"][2] - 0.006,
        ]
        actuate(sdk, "open", 8)
        servo(sdk, [handle[0], handle[1], clearance],
              yaw, "open", 30, 0.006)

        state = sdk.get_state()
        nut = state["objects"]["nut"]
        original_z = nut["pos"][2]
        yaw = nut["yaw"]
        handle = [
            nut["pos"][0] + 0.055 * cos(yaw),
            nut["pos"][1] + 0.055 * sin(yaw),
            nut["pos"][2] - 0.006,
        ]
        servo(sdk, handle, yaw, "open", 48, 0.003)

        # The previous attempt closed roughly 19 mm above the requested
        # grasp. Require measured depth and lateral alignment before closing.
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        live_handle = [
            nut["pos"][0] + 0.055 * cos(nut["yaw"]),
            nut["pos"][1] + 0.055 * sin(nut["yaw"]),
            nut["pos"][2] - 0.006,
        ]
        if (
            hypot(state["eef_pos"][0] - live_handle[0],
                  state["eef_pos"][1] - live_handle[1]) > 0.006
            or abs(state["eef_pos"][2] - live_handle[2]) > 0.005
        ):
            servo(sdk, live_handle, nut["yaw"], "open", 18, 0.003)

        state = sdk.get_state()
        if abs(state["eef_pos"][2] - live_handle[2]) > 0.008:
            continue

        actuate(sdk, "close", 14)
        state = sdk.get_state()
        if not 0.012 <= state["gripper_width"] <= 0.070:
            continue

        eef = state["eef_pos"]
        servo(sdk, [eef[0], eef[1], eef[2] + 0.065],
              state["eef_yaw"], "close", 30, 0.005)
        state = sdk.get_state()
        grasped = (
            state["objects"]["nut"]["pos"][2] > original_z + 0.035
            and 0.012 <= state["gripper_width"] <= 0.070
        )
        if grasped:
            break

    if not grasped:
        actuate(sdk, "open", 8)
        return {"status": "failed", "reason": "Handle grasp failed lift verification"}

    state = sdk.get_state()
    hole = state["objects"]["nut"]["pos"]
    position_hole(sdk, [hole[0], hole[1], clearance], peg_yaw, 36)
    state = sdk.get_state()
    if state["objects"]["nut"]["pos"][2] < clearance - 0.012:
        return {"status": "failed", "reason": "Insufficient clearance"}

    position_hole(sdk, [goal[0], goal[1], clearance], peg_yaw, 54)
    state = sdk.get_state()
    nut = state["objects"]["nut"]
    if (
        hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) > 0.005
        or abs(square_error(peg_yaw, nut["yaw"])) > 0.06
        or state["gripper_width"] < 0.012
    ):
        return {"status": "failed", "reason": "Pre-insertion alignment failed"}

    for height in (rest_z + 0.060, rest_z + 0.030, rest_z + 0.003):
        if budget(sdk) < 35:
            return {"status": "failed", "reason": "Insufficient seating budget"}
        position_hole(sdk, [goal[0], goal[1], height],
                      peg_yaw, min(30, budget(sdk) - 23))
        state = sdk.get_state()
        nut = state["objects"]["nut"]
        if (
            hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1]) > 0.010
            or state["gripper_width"] < 0.010
        ):
            return {"status": "failed", "reason": "Lost insertion alignment"}

    state = sdk.get_state()
    if state["objects"]["nut"]["pos"][2] > rest_z + 0.018:
        if budget(sdk) >= 41:
            position_hole(sdk, [goal[0], goal[1], rest_z + 0.002],
                          peg_yaw, 18)
        state = sdk.get_state()
        if state["objects"]["nut"]["pos"][2] > rest_z + 0.018:
            return {"status": "failed", "reason": "Nut remains above seat"}

    actuate(sdk, "open", 11)
    state = sdk.get_state()
    eef = state["eef_pos"]
    servo(sdk, [eef[0], eef[1], eef[2] + 0.080],
          state["eef_yaw"], "open", min(18, budget(sdk)), 0.008)

    state = sdk.get_state()
    nut = state["objects"]["nut"]
    error = hypot(nut["pos"][0] - goal[0], nut["pos"][1] - goal[1])
    seated = (
        error < 0.012
        and abs(nut["pos"][2] - rest_z) < 0.020
        and abs(square_error(peg_yaw, nut["yaw"])) < 0.10
        and state["gripper_width"] > 0.045
    )
    return {
        "status": "done" if seated else "failed",
        "reason": "Released and seated" if seated else "Seating verification failed",
        "hole_xy_error_m": error,
        "steps_used": state["steps_used"],
    }
