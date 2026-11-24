def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return max(0, min(450, state["max_steps"]) - state["steps_used"])

    def xy_distance(a, b):
        return hypot(a[0] - b[0], a[1] - b[1])

    def move(pos, yaw, closed, steps=35, tol=0.01):
        budget = min(steps, remaining())
        if budget <= 0:
            return False
        sdk.move_to(pos, yaw=yaw, gripper=1.0 if closed else -1.0,
                    max_steps=budget, tol=tol)
        actual = sdk.get_state()["eef_pos"]
        return sqrt(sum((actual[i] - pos[i]) ** 2 for i in range(3))) < tol + 0.004

    def grip(action, steps=12):
        budget = min(steps, remaining())
        if budget > 0:
            return sdk.set_gripper(action, steps=budget)
        return {}

    def settle(steps, closed):
        budget = min(steps, remaining())
        if budget > 0:
            sdk.step_raw([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          1.0 if closed else -1.0], steps=budget)

    state = sdk.get_state()
    objects = state["objects"]
    sites = state["sites"]
    bowls = [name for name in objects if "black_bowl" in name]
    plates = [name for name in objects if "plate" in name]
    if not bowls or not plates:
        return {"status": "failed", "reason": "Required objects unavailable"}

    center_name = ""
    for name in ["main_table_table_center_region",
                 "main_table_center_region", "table_top"]:
        if name in sites:
            center_name = name
            break
    if not center_name:
        return {"status": "failed", "reason": "Table center unavailable"}

    center = sites[center_name]
    bowl_name = bowls[0]
    for name in bowls:
        if xy_distance(objects[name]["pos"], center) < xy_distance(
                objects[bowl_name]["pos"], center):
            bowl_name = name

    plate_region = sites.get("main_table_plate_region")
    if len(plates) > 1 and plate_region is None:
        return {"status": "failed", "reason": "Ambiguous target plate"}
    plate_name = plates[0]
    if plate_region is not None:
        for name in plates:
            if xy_distance(objects[name]["pos"], plate_region) < xy_distance(
                    objects[plate_name]["pos"], plate_region):
                plate_name = name

    initial_bowl = list(objects[bowl_name]["pos"])
    initial_plate = list(objects[plate_name]["pos"])
    grounding = {
        "object": bowl_name,
        "target": plate_name,
        "center_site": center_name,
        "center_pos": list(center),
        "initial_object_pos": initial_bowl,
        "initial_target_pos": initial_plate,
        "object_default_site": sites.get(bowl_name + "_default_site"),
        "target_default_site": sites.get(plate_name + "_default_site")
    }

    yaw = state["eef_yaw"]
    held = False
    attempts = 0
    grasp_offset = [0.0, 0.0, 0.0]

    # The exposed default sites are object origins, not grasp surfaces.
    # Test lower grasp heights than the previous unsuccessful approaches.
    for attempt in range(3):
        if remaining() < 160:
            break
        attempts = attempt + 1
        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        plate = state["objects"][plate_name]["pos"]
        eef = list(state["eef_pos"])
        clearance = max(bowl[2], plate[2]) + 0.15

        move([eef[0], eef[1], max(clearance, eef[2])],
             yaw, False, 25)
        grip("open", 8)

        if attempt == 1:
            yaw = yaw + pi / 2.0
        height = 0.018
        radius = 0.0
        if attempt == 1:
            height = 0.003
        if attempt == 2:
            height = 0.018
            radius = 0.028

        offset_x = radius * cos(yaw)
        offset_y = radius * sin(yaw)
        bowl = list(sdk.get_state()["objects"][bowl_name]["pos"])
        if not move([bowl[0] + offset_x, bowl[1] + offset_y, clearance],
                    yaw, False, 35):
            continue

        bowl = list(sdk.get_state()["objects"][bowl_name]["pos"])
        move([bowl[0] + offset_x, bowl[1] + offset_y, bowl[2] + 0.065],
             yaw, False, 22, 0.008)
        bowl = list(sdk.get_state()["objects"][bowl_name]["pos"])
        move([bowl[0] + offset_x, bowl[1] + offset_y, bowl[2] + height],
             yaw, False, 28, 0.007)

        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        eef = state["eef_pos"]
        if xy_distance(bowl, eef) > radius + 0.022:
            continue
        before_lift = list(bowl)
        grip("close", 16)
        eef = list(sdk.get_state()["eef_pos"])
        move([eef[0], eef[1], eef[2] + 0.075],
             yaw, True, 25, 0.009)

        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        eef = list(state["eef_pos"])
        if bowl[2] - before_lift[2] < 0.035:
            grip("open", 6)
            continue

        grasp_offset = [bowl[i] - eef[i] for i in range(3)]
        move([eef[0], eef[1],
              max(eef[2], clearance - grasp_offset[2])],
             yaw, True, 25)
        settle(3, True)
        state = sdk.get_state()
        bowl = state["objects"][bowl_name]["pos"]
        eef = state["eef_pos"]
        offset = [bowl[i] - eef[i] for i in range(3)]
        held = (
            bowl[2] > before_lift[2] + 0.07
            and sqrt(sum((offset[i] - grasp_offset[i]) ** 2
                         for i in range(3))) < 0.025
        )
        if held:
            grasp_offset = offset
            break

    if not held:
        return dict(grounding, status="failed",
                    reason="Selected bowl did not follow the gripper",
                    attempts=attempts, steps_used=sdk.get_state()["steps_used"])

    state = sdk.get_state()
    bowl = list(state["objects"][bowl_name]["pos"])
    plate = list(state["objects"][plate_name]["pos"])
    transport_z = max(bowl[2], plate[2] + 0.14)
    move([plate[0] - grasp_offset[0],
          plate[1] - grasp_offset[1],
          transport_z - grasp_offset[2]],
         yaw, True, min(45, max(0, remaining() - 65)), 0.008)

    for height in [0.045, 0.008]:
        state = sdk.get_state()
        bowl = list(state["objects"][bowl_name]["pos"])
        plate = list(state["objects"][plate_name]["pos"])
        eef = list(state["eef_pos"])
        offset = [bowl[i] - eef[i] for i in range(3)]
        if sqrt(sum((offset[i] - grasp_offset[i]) ** 2
                    for i in range(3))) > 0.035:
            return dict(grounding, status="failed", reason="Bowl slipped")
        if xy_distance(bowl, plate) > 0.03:
            move([eef[0] + plate[0] - bowl[0],
                  eef[1] + plate[1] - bowl[1], eef[2]],
                 yaw, True, min(18, max(0, remaining() - 55)), 0.007)
            state = sdk.get_state()
            bowl = list(state["objects"][bowl_name]["pos"])
            eef = list(state["eef_pos"])
            offset = [bowl[i] - eef[i] for i in range(3)]
        if xy_distance(bowl, plate) > 0.025:
            return dict(grounding, status="failed",
                        reason="Plate alignment not verified")
        resting_z = plate[2] + initial_bowl[2] - initial_plate[2] + 0.015
        move([plate[0] - offset[0], plate[1] - offset[1],
              resting_z + height - offset[2]],
             yaw, True, min(25, max(0, remaining() - 30)), 0.007)

    state = sdk.get_state()
    bowl = state["objects"][bowl_name]["pos"]
    plate = state["objects"][plate_name]["pos"]
    resting_z = plate[2] + initial_bowl[2] - initial_plate[2] + 0.015
    if xy_distance(bowl, plate) > 0.025 or abs(bowl[2] - resting_z) > 0.03:
        return dict(grounding, status="failed", reason="Release pose unverified")

    grip("open", 10)
    eef = list(sdk.get_state()["eef_pos"])
    move([eef[0], eef[1], eef[2] + 0.12],
         yaw, False, min(22, max(0, remaining() - 8)), 0.012)
    settle(4, False)
    previous = list(sdk.get_state()["objects"][bowl_name]["pos"])
    settle(4, False)
    state = sdk.get_state()
    bowl = list(state["objects"][bowl_name]["pos"])
    plate = list(state["objects"][plate_name]["pos"])
    expected_z = plate[2] + initial_bowl[2] - initial_plate[2] + 0.015
    placed = (
        xy_distance(bowl, plate) < 0.03
        and abs(bowl[2] - expected_z) < 0.025
        and state["gripper_width"] > 0.05
        and state["eef_pos"][2] - bowl[2] > 0.065
        and sqrt(sum((bowl[i] - previous[i]) ** 2 for i in range(3))) < 0.008
    )
    return dict(grounding, status="done" if placed else "failed",
                placement_verified=placed, attempts=attempts,
                final_object_pos=bowl, final_target_pos=plate,
                steps_used=state["steps_used"])
