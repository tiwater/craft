def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def move(target, grip, steps=30, tol=0.008):
        budget = remaining()
        if budget <= 0:
            return False
        result = sdk.move_to(
            target, gripper=grip, max_steps=min(steps, budget), tol=tol
        )
        return result["reached"]

    def actuate(action, steps):
        if remaining() > 0:
            sdk.set_gripper(action, steps=min(steps, remaining()))

    def distance(a, b):
        return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    def point_down(yaw):
        target = [cos(yaw / 2.0), sin(yaw / 2.0), 0.0, 0.0]
        for tick in range(16):
            if remaining() <= 0:
                break
            q = sdk.get_state()["eef_quat"]
            x, y, z, w = q
            a, b, c, d = target
            error = [
                -d * x + a * w - b * z + c * y,
                -d * y + a * z + b * w - c * x,
                -d * z - a * y + b * x + c * w,
                d * w + a * x + b * y + c * z,
            ]
            if error[3] < 0.0:
                error = [-value for value in error]
            length = sqrt(sum(error[i] ** 2 for i in range(3)))
            angle = 2.0 * atan2(length, max(0.0, error[3]))
            if angle < 0.035:
                break
            scale = min(0.6, 1.5 * angle) / max(length, 0.000001)
            sdk.step_raw(
                [
                    0.0, 0.0, 0.0,
                    error[0] * scale,
                    error[1] * scale,
                    error[2] * scale,
                    -1.0,
                ],
                steps=1,
            )

    state = sdk.get_state()
    if "can" not in state["objects"] or state["goal_pos"] is None:
        return {"status": "failed", "reason": "Missing can or target"}

    # The unchanged can positions show that the previous high grasps
    # closed above the can. Use the grasp site near the can's center.
    attached = False
    offset = [0.0, 0.0, 0.0]
    yaw = state["eef_yaw"]

    for attempt, grasp_height in enumerate([0.005, -0.012, 0.02]):
        if remaining() < 165:
            break

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = list(state["eef_pos"])
        goal = state["goal_pos"]
        clearance = max(can[2] + 0.19, goal[2] + 0.19, eef[2])

        move([eef[0], eef[1], clearance], None, 22)
        actuate("open", 7)
        point_down(yaw + (pi / 2.0 if attempt == 1 else 0.0))

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        move([can[0], can[1], clearance], "open", 30)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if hypot(eef[0] - can[0], eef[1] - can[1]) > 0.022:
            continue

        move([can[0], can[1], can[2] + 0.085], "open", 18)
        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        move([can[0], can[1], can[2] + grasp_height], "open", 28, 0.005)

        state = sdk.get_state()
        can = list(state["objects"]["can"]["pos"])
        eef = state["eef_pos"]
        if (
            hypot(eef[0] - can[0], eef[1] - can[1]) > 0.025
            or eef[2] - can[2] > 0.045
            or eef[2] - can[2] < -0.035
        ):
            continue

        actuate("close", 12)
        state = sdk.get_state()
        before_can = list(state["objects"]["can"]["pos"])
        before_eef = list(state["eef_pos"])
        move(
            [before_eef[0], before_eef[1], before_eef[2] + 0.13],
            "close", 25,
        )

        state = sdk.get_state()
        can = state["objects"]["can"]["pos"]
        eef = state["eef_pos"]
        can_rise = can[2] - before_can[2]
        eef_rise = eef[2] - before_eef[2]
        attached = (
            can_rise > 0.045
            and abs(can_rise - eef_rise) < 0.035
            and hypot(can[0] - eef[0], can[1] - eef[1]) < 0.055
            and 0.012 < state["gripper_width"] < 0.075
        )
        if attached:
            offset = [can[i] - eef[i] for i in range(3)]
            break

    if not attached:
        return {"status": "failed", "reason": "No verified grasp"}

    state = sdk.get_state()
    goal = list(state["goal_pos"])
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    transit_height = max(can[2], goal[2] + 0.21) - offset[2]

    move([eef[0], eef[1], transit_height], "close", 25)
    state = sdk.get_state()
    if state["objects"]["can"]["pos"][2] < goal[2] + 0.16:
        return {"status": "failed", "reason": "Insufficient wall clearance"}

    move(
        [goal[0] - offset[0], goal[1] - offset[1], transit_height],
        "close", min(65, max(1, remaining() - 65)),
    )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    if distance(can, [eef[i] + offset[i] for i in range(3)]) > 0.045:
        return {"status": "failed", "reason": "Grasp lost during transport"}

    if hypot(can[0] - goal[0], can[1] - goal[1]) > 0.012:
        move(
            [eef[0] + goal[0] - can[0],
             eef[1] + goal[1] - can[1], eef[2]],
            "close", min(18, max(1, remaining() - 47)), 0.006,
        )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    eef = state["eef_pos"]
    goal = list(state["goal_pos"])
    if hypot(can[0] - goal[0], can[1] - goal[1]) > 0.035:
        return {"status": "failed", "reason": "Target alignment failed"}

    offset = [can[i] - eef[i] for i in range(3)]
    move(
        [goal[0] - offset[0],
         goal[1] - offset[1],
         goal[2] + 0.045 - offset[2]],
        "close", min(30, max(1, remaining() - 27)), 0.008,
    )

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    if (
        hypot(can[0] - goal[0], can[1] - goal[1]) > 0.04
        or can[2] > goal[2] + 0.11
        or remaining() < 20
    ):
        return {"status": "failed", "reason": "Unsafe release position"}

    actuate("open", 9)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.17],
        "open", min(22, max(1, remaining() - 5)),
    )
    actuate("open", 5)

    state = sdk.get_state()
    can = state["objects"]["can"]["pos"]
    goal = state["goal_pos"]
    horizontal_error = hypot(can[0] - goal[0], can[1] - goal[1])
    placed = (
        horizontal_error < 0.055
        and goal[2] - 0.065 < can[2] < goal[2] + 0.085
        and state["gripper_width"] > 0.055
        and state["eef_pos"][2] - can[2] > 0.12
    )
    return {
        "status": "done" if placed else "failed",
        "reason": "Can released in target bin" if placed else "Placement verification failed",
        "horizontal_error_m": horizontal_error,
        "steps_used": state["steps_used"],
    }
