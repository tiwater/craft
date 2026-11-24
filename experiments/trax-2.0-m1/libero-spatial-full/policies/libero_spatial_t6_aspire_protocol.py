def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def move(pos, yaw, action, steps, tol=0.009):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.move_to(
                pos, yaw=yaw, gripper=action,
                max_steps=budget, tol=tol
            )
        return sdk.get_state()

    def grip(action, steps):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.set_gripper(action, steps=budget)
        return sdk.get_state()

    def distance(a, b):
        return hypot(a[0] - b[0], a[1] - b[1])

    def offset(state, name):
        return [
            state["objects"][name]["pos"][i] - state["eef_pos"][i]
            for i in range(3)
        ]

    def change(a, b):
        return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    state = sdk.get_state()
    objects = state["objects"]
    bowls = [
        name for name in objects
        if "black" in name.lower() and "bowl" in name.lower()
    ]
    cookies = [name for name in objects if "cookie" in name.lower()]
    plates = [name for name in objects if "plate" in name.lower()]
    if not bowls or not cookies or not plates:
        return {"status": "failed", "reason": "Required objects missing"}

    target = bowls[0]
    cookie_name = cookies[0]
    nearest = float("inf")
    for name in bowls:
        for cookie in cookies:
            gap = distance(objects[name]["pos"], objects[cookie]["pos"])
            if gap < nearest:
                nearest = gap
                target = name
                cookie_name = cookie

    plate = plates[0]
    initial_z = objects[target]["pos"][2]
    held = False
    grasp_offset = [0.0, 0.0, 0.0]
    yaw = state["eef_yaw"]

    # Pinch one wall of the bowl; a centered grasp can close inside it.
    for attempt in range(3):
        if remaining() < 150:
            break

        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        cookie = state["objects"][cookie_name]["pos"]
        vx = bowl[0] - cookie[0]
        vy = bowl[1] - cookie[1]
        norm = hypot(vx, vy)
        if norm < 0.001:
            vx = 1.0
            vy = 0.0
            norm = 1.0
        vx = vx / norm
        vy = vy / norm

        angle = atan2(vy, vx)
        radius = 0.043
        depth = 0.012
        yaw = angle - pi / 2
        if attempt == 1:
            yaw = angle
            depth = 0.006
        if attempt == 2:
            angle = angle + pi / 2
            vx = cos(angle)
            vy = sin(angle)
            yaw = angle - pi / 2
            radius = 0.047
            depth = 0.018

        # Lift closed fingers clear before reopening for another approach.
        eef = list(state["eef_pos"])
        clearance = max(eef[2], bowl[2] + 0.13)
        state = move(
            [eef[0], eef[1], clearance],
            state["eef_yaw"], "close" if attempt > 0 else "open", 18
        )
        state = grip("open", 7)
        bowl = list(state["objects"][target]["pos"])
        state = move(
            [bowl[0] + radius * vx, bowl[1] + radius * vy, clearance],
            yaw, "open", 28
        )
        bowl = list(state["objects"][target]["pos"])
        aim = [
            bowl[0] + radius * vx,
            bowl[1] + radius * vy,
            bowl[2] + depth,
        ]
        if distance(state["eef_pos"], aim) > 0.02:
            continue

        state = move(aim, yaw, "open", 28, 0.006)
        bowl = list(state["objects"][target]["pos"])
        eef = list(state["eef_pos"])
        if distance(eef, aim) > 0.022 or abs(eef[2] - aim[2]) > 0.025:
            continue

        before = list(bowl)
        state = grip("close", 12)
        eef = list(state["eef_pos"])
        state = move(
            [eef[0], eef[1], eef[2] + 0.09],
            yaw, "close", 23
        )
        bowl = list(state["objects"][target]["pos"])
        if bowl[2] < before[2] + 0.035:
            continue

        first_bowl = list(bowl)
        first_eef = list(state["eef_pos"])
        first_offset = offset(state, target)
        state = move(
            [first_eef[0], first_eef[1], first_eef[2] + 0.035],
            yaw, "close", 14, 0.006
        )
        bowl = state["objects"][target]["pos"]
        grasp_offset = offset(state, target)
        if (
            bowl[2] > before[2] + 0.05
            and bowl[2] - first_bowl[2] > 0.012
            and state["eef_pos"][2] - first_eef[2] > 0.015
            and change(first_offset, grasp_offset) < 0.025
        ):
            held = True
            break

    if not held:
        return {"status": "failed", "reason": "No verified rim grasp"}

    state = sdk.get_state()
    destination = list(state["objects"][plate]["pos"])
    eef = list(state["eef_pos"])
    transit_z = max(
        eef[2],
        max(initial_z, destination[2]) + 0.15 - grasp_offset[2]
    )
    state = move([eef[0], eef[1], transit_z], yaw, "close", 18)
    destination = list(state["objects"][plate]["pos"])
    state = move(
        [
            destination[0] - grasp_offset[0],
            destination[1] - grasp_offset[1],
            transit_z,
        ],
        yaw, "close", min(48, max(0, remaining() - 55))
    )

    if change(grasp_offset, offset(state, target)) > 0.045:
        return {"status": "failed", "reason": "Bowl slipped during transport"}

    for correction in range(2):
        bowl = list(state["objects"][target]["pos"])
        destination = list(state["objects"][plate]["pos"])
        if distance(bowl, destination) < 0.012:
            break
        eef = list(state["eef_pos"])
        state = move(
            [
                eef[0] + destination[0] - bowl[0],
                eef[1] + destination[1] - bowl[1],
                eef[2],
            ],
            yaw, "close", min(16, max(0, remaining() - 43)), 0.006
        )

    bowl = list(state["objects"][target]["pos"])
    destination = list(state["objects"][plate]["pos"])
    if distance(bowl, destination) > 0.03 or remaining() < 32:
        return {"status": "failed", "reason": "Insufficient placement alignment"}

    grasp_offset = offset(state, target)
    state = move(
        [
            destination[0] - grasp_offset[0],
            destination[1] - grasp_offset[1],
            destination[2] + 0.04 - grasp_offset[2],
        ],
        yaw, "close", min(30, max(0, remaining() - 22)), 0.007
    )
    bowl = state["objects"][target]["pos"]
    destination = state["objects"][plate]["pos"]
    if distance(bowl, destination) > 0.035 or bowl[2] > destination[2] + 0.075:
        return {"status": "failed", "reason": "Placement descent incomplete"}

    state = grip("open", 9)
    eef = list(state["eef_pos"])
    state = move(
        [eef[0], eef[1], eef[2] + 0.11],
        yaw, "open", min(18, max(0, remaining() - 5)), 0.015
    )
    state = grip("open", min(6, remaining()))
    bowl = state["objects"][target]["pos"]
    destination = state["objects"][plate]["pos"]
    error = distance(bowl, destination)
    height = bowl[2] - destination[2]
    return {
        "status": "done" if error < 0.04 and -0.015 < height < 0.065 else "failed",
        "target": target,
        "horizontal_error_m": error,
        "height_above_plate_m": height,
    }
