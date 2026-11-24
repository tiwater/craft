def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def distance_xy(a, b):
        return hypot(a[0] - b[0], a[1] - b[1])

    def distance_xyz(a, b):
        return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    def wrap(angle):
        return atan2(sin(angle), cos(angle))

    def move(pos, yaw, grip, budget=35, tol=0.01):
        available = remaining()
        if available > 0:
            sdk.move_to(
                pos,
                yaw=yaw,
                gripper=grip,
                max_steps=min(budget, available),
                tol=tol,
            )
        return distance_xyz(sdk.get_state()["eef_pos"], pos)

    def actuate(action, steps=10):
        if remaining() > 0:
            sdk.set_gripper(action, steps=min(steps, remaining()))

    def choose_angle(state, bowl_name, attempted):
        bowl = state["objects"][bowl_name]["pos"]
        obstacles = []
        for group in ("objects", "fixtures"):
            for name in state[group]:
                if group != "objects" or name != bowl_name:
                    obstacles.append(state[group][name]["pos"])

        best_angle = None
        best_score = None
        # Positions alone support a point-clearance estimate, not mesh clearance.
        # Include the supporting cookie box and every other object/fixture.
        for index in range(120):
            angle = wrap(-pi / 2.0 + index * 2.0 * pi / 120.0)
            if any(abs(wrap(angle - old)) < pi / 5.0 for old in attempted):
                continue
            contact = [
                bowl[0] + 0.04 * cos(angle),
                bowl[1] + 0.04 * sin(angle),
                bowl[2] + 0.02,
            ]
            clearance = 1000.0
            for obstacle in obstacles:
                clearance = min(clearance, distance_xy(contact, obstacle))
            # The measured successful direction only breaks clearance ties.
            score = (clearance, -abs(wrap(angle + pi / 2.0)))
            if best_score is None or score > best_score:
                best_score = score
                best_angle = angle
        return best_angle

    def corridor_height(state, start, end, bowl_name):
        height = max(start[2], end[2]) + 0.14
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_squared = dx * dx + dy * dy
        points = []
        for group in ("objects", "fixtures"):
            for name in state[group]:
                if group != "objects" or name != bowl_name:
                    points.append(state[group][name]["pos"])
        for name in state["sites"]:
            if "top" in name or "burner" in name:
                points.append(state["sites"][name])
        for point in points:
            fraction = 0.0
            if length_squared > 0.000001:
                fraction = max(
                    0.0,
                    min(
                        1.0,
                        ((point[0] - start[0]) * dx
                         + (point[1] - start[1]) * dy) / length_squared,
                    ),
                )
            nearest = [
                start[0] + fraction * dx,
                start[1] + fraction * dy,
            ]
            if distance_xy(point, nearest) < 0.13:
                height = max(height, point[2] + 0.14)
        return height

    state = sdk.get_state()
    bowls = [name for name in state["objects"] if "black_bowl" in name]
    boxes = [name for name in state["objects"] if "cookie" in name]
    plates = [name for name in state["objects"] if "plate" in name]
    if not bowls or not boxes or not plates:
        return {"status": "failed", "reason": "Required objects unavailable"}

    bowl_name = bowls[0]
    best_score = 1000000.0
    for name in bowls:
        bowl = state["objects"][name]["pos"]
        for box_name in boxes:
            box = state["objects"][box_name]["pos"]
            score = distance_xy(bowl, box) + 0.25 * abs(bowl[2] - box[2])
            if bowl[2] < box[2] - 0.02:
                score += 1.0
            if score < best_score:
                best_score = score
                bowl_name = name
    plate_name = plates[0]
    attempted = []
    held = False
    grasp_yaw = 0.0
    reference_z = 0.0

    for attempt in range(3):
        if remaining() < 165:
            break
        state = sdk.get_state()
        angle = choose_angle(state, bowl_name, attempted)
        if angle is None:
            break
        attempted.append(angle)
        # Probe-grounded tangential jaw alignment: radial angle + pi/2.
        grasp_yaw = wrap(angle + pi / 2.0)
        bowl = list(state["objects"][bowl_name]["pos"])
        reference_z = bowl[2]
        contact = [
            bowl[0] + 0.04 * cos(angle),
            bowl[1] + 0.04 * sin(angle),
            bowl[2] + 0.02,
        ]
        eef = state["eef_pos"]
        approach_z = corridor_height(state, eef, bowl, bowl_name)
        move([eef[0], eef[1], approach_z], grasp_yaw, "open", 25)
        actuate("open", 7)
        move([contact[0], contact[1], approach_z], grasp_yaw, "open", 35)

        state = sdk.get_state()
        if distance_xy(state["eef_pos"], contact) > 0.035:
            continue

        bowl = list(state["objects"][bowl_name]["pos"])
        reference_z = bowl[2]
        contact = [
            bowl[0] + 0.04 * cos(angle),
            bowl[1] + 0.04 * sin(angle),
            bowl[2] + 0.02,
        ]
        move(contact, grasp_yaw, "open", 38, 0.008)
        state = sdk.get_state()
        if distance_xyz(state["eef_pos"], contact) > 0.018:
            move(contact, grasp_yaw, "open", 14, 0.008)

        state = sdk.get_state()
        eef = state["eef_pos"]
        # Contact can prevent exact convergence; do not gate on reached.
        if distance_xy(eef, contact) > 0.025 or abs(eef[2] - contact[2]) > 0.035:
            continue

        actuate("close", 16)
        state = sdk.get_state()
        before_lift_z = state["objects"][bowl_name]["pos"][2]
        eef = state["eef_pos"]
        move(
            [eef[0], eef[1], eef[2] + 0.10],
            grasp_yaw,
            "close",
            32,
            0.01,
        )
        state = sdk.get_state()
        bowl = state["objects"][bowl_name]["pos"]
        eef = state["eef_pos"]
        lift = bowl[2] - before_lift_z
        held = (
            lift >= 0.03
            and distance_xy(bowl, eef) <= 0.075
            and distance_xyz(bowl, eef) <= 0.13
        )
        if held:
            reference_z = before_lift_z
            break
        actuate("open", 7)

    if not held:
        return {"status": "failed", "reason": "No verified object lift"}

    state = sdk.get_state()
    bowl = list(state["objects"][bowl_name]["pos"])
    plate = list(state["objects"][plate_name]["pos"])
    eef = state["eef_pos"]
    offset = [bowl[i] - eef[i] for i in range(3)]
    object_carry_z = corridor_height(state, bowl, plate, bowl_name)
    carry_z = max(eef[2], object_carry_z - offset[2])
    move([eef[0], eef[1], carry_z], grasp_yaw, "close", 25)

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    eef = state["eef_pos"]
    if bowl[2] - reference_z < 0.03 or distance_xyz(bowl, eef) > 0.13:
        return {"status": "failed", "reason": "Lift lost before transport"}

    offset = [bowl[i] - eef[i] for i in range(3)]
    plate = state["objects"][plate_name]["pos"]
    move(
        [plate[0] - offset[0], plate[1] - offset[1], carry_z],
        grasp_yaw,
        "close",
        45,
    )
    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    if bowl[2] - reference_z < 0.03 or distance_xyz(bowl, eef) > 0.13:
        return {"status": "failed", "reason": "Lift lost during transport"}

    if distance_xy(bowl, plate) > 0.018 and remaining() >= 70:
        move(
            [
                eef[0] + plate[0] - bowl[0],
                eef[1] + plate[1] - bowl[1],
                eef[2],
            ],
            grasp_yaw,
            "close",
            18,
            0.008,
        )

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    eef = state["eef_pos"]
    if distance_xy(bowl, plate) > 0.035 or remaining() < 38:
        return {"status": "failed", "reason": "Placement approach incomplete"}

    release_pos = [
        eef[0] + plate[0] - bowl[0],
        eef[1] + plate[1] - bowl[1],
        eef[2] + plate[2] + 0.018 - bowl[2],
    ]
    move(
        release_pos,
        grasp_yaw,
        "close",
        min(35, remaining() - 28),
        0.008,
    )
    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    if distance_xy(bowl, plate) > 0.035 or bowl[2] > plate[2] + 0.06:
        return {"status": "failed", "reason": "Release pose not established"}

    actuate("open", 10)
    state = sdk.get_state()
    eef = state["eef_pos"]
    move(
        [eef[0], eef[1], eef[2] + 0.12],
        grasp_yaw,
        "open",
        min(24, max(0, remaining() - 5)),
    )
    actuate("open", 5)
    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    placed = (
        distance_xy(bowl, plate) < 0.045
        and plate[2] - 0.015 <= bowl[2] <= plate[2] + 0.055
        and state["eef_pos"][2] - bowl[2] > 0.06
    )
    return {
        "status": "done" if placed else "failed",
        "object": bowl_name,
        "target": plate_name,
        "attempts": len(attempted),
    }
