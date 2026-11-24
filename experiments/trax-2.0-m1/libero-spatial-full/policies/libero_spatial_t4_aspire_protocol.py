def run_policy(sdk):
    def remaining():
        state = sdk.get_state()
        return min(450, state["max_steps"]) - state["steps_used"]

    def move(pos, yaw, action, steps=20, tol=0.01):
        budget = remaining()
        if budget <= 0:
            return False
        result = sdk.move_to(
            list(pos), yaw=yaw, gripper=action,
            max_steps=min(steps, budget), tol=tol
        )
        return result["reached"]

    def grip(action, steps=6):
        if remaining() > 0:
            return sdk.set_gripper(action, steps=min(steps, remaining()))
        return {}

    def distance(a, b):
        return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    def dot(a, b):
        return a[0] * b[0] + a[1] * b[1]

    state = sdk.get_state()
    bowls = [name for name in state["objects"] if "black_bowl" in name]
    plates = [name for name in state["objects"] if "plate" in name]
    cabinets = [name for name in state["fixtures"] if "wooden_cabinet" in name]
    if not bowls or not plates or not cabinets:
        return {"status": "failed", "reason": "Required entities missing"}

    cabinet = cabinets[0]
    regions = [
        name for name in state["sites"]
        if cabinet in name and "top" in name and "region" in name
    ]
    if not regions:
        return {"status": "failed", "reason": "Top drawer region missing"}

    region_name = regions[0]
    initial_region = list(state["sites"][region_name])
    target = bowls[0]
    for name in bowls:
        if distance(state["objects"][name]["pos"], initial_region) < distance(
            state["objects"][target]["pos"], initial_region
        ):
            target = name
    plate = plates[0]
    initial_bowl = list(state["objects"][target]["pos"])
    cabinet_pos = list(state["fixtures"][cabinet]["pos"])
    cabinet_yaw = state["fixtures"][cabinet]["yaw"]

    handles = [
        name for name in state["sites"]
        if cabinet in name and "handle" in name
    ]
    if not handles:
        handles = [
            name for name in state["sites"]
            if "handle" in name and ("drawer" in name or "cabinet" in name)
        ]
    top_handles = [name for name in handles if "top" in name]
    if top_handles:
        handles = top_handles
    handle_name = None
    for name in handles:
        if handle_name is None:
            handle_name = name
        elif state["sites"][name][2] > state["sites"][handle_name][2]:
            handle_name = name

    initial_handle = None
    if handle_name is not None:
        initial_handle = list(state["sites"][handle_name])

    axes = [
        [cos(cabinet_yaw), sin(cabinet_yaw)],
        [-cos(cabinet_yaw), -sin(cabinet_yaw)],
        [-sin(cabinet_yaw), cos(cabinet_yaw)],
        [sin(cabinet_yaw), -cos(cabinet_yaw)]
    ]
    # Use the drawer interior as the reference for a real handle.
    # The cabinet body's origin need not be at its geometric center.
    if initial_handle is not None:
        reference = [
            initial_handle[0] - initial_region[0],
            initial_handle[1] - initial_region[1]
        ]
    else:
        reference = [
            state["eef_pos"][0] - cabinet_pos[0],
            state["eef_pos"][1] - cabinet_pos[1]
        ]
    front = axes[0]
    for axis in axes:
        if dot(axis, reference) > dot(front, reference):
            front = axis
    front = list(front)

    def displacement():
        current = sdk.get_state()
        if handle_name is not None:
            position = current["sites"][handle_name]
            return [
                position[0] - initial_handle[0],
                position[1] - initial_handle[1]
            ]
        region = current["sites"][region_name]
        region_delta = [
            region[0] - initial_region[0],
            region[1] - initial_region[1]
        ]
        bowl = current["objects"][target]["pos"]
        bowl_delta = [
            bowl[0] - initial_bowl[0],
            bowl[1] - initial_bowl[1]
        ]
        if hypot(region_delta[0], region_delta[1]) > 0.008:
            return region_delta
        return bowl_delta

    def travel():
        delta = displacement()
        return hypot(delta[0], delta[1])

    opened = False
    grip("open", 4)
    drawer_start = sdk.get_state()["steps_used"]

    for attempt in range(3):
        if remaining() < 205:
            break

        state = sdk.get_state()
        region = list(state["sites"][region_name])
        if handle_name is not None:
            handle = list(state["sites"][handle_name])
            handle[2] += [0.0, -0.012, 0.012][attempt]
        else:
            handle = [
                region[0] + front[0] * [0.14, 0.105, 0.175][attempt],
                region[1] + front[1] * [0.14, 0.105, 0.175][attempt],
                region[2] + [-0.035, -0.02, -0.05][attempt]
            ]

        yaw = atan2(front[1], front[0])
        if attempt == 1:
            yaw += pi / 2

        approach = [
            handle[0] + front[0] * 0.055,
            handle[1] + front[1] * 0.055,
            max(state["eef_pos"][2], handle[2] + 0.075)
        ]
        move(approach, yaw, "open", 18)
        approach[2] = handle[2]
        move(approach, yaw, "open", 12)
        move(handle, yaw, "open", 16, 0.006)

        state = sdk.get_state()
        if distance(state["eef_pos"], handle) < 0.025:
            grip("close", 8)
            pull_direction = list(front)
            previous = displacement()
            stagnant = 0
            for increment in range(6):
                if remaining() < 190:
                    break
                state = sdk.get_state()
                eef = list(state["eef_pos"])
                destination = [
                    eef[0] + pull_direction[0] * 0.045,
                    eef[1] + pull_direction[1] * 0.045,
                    eef[2]
                ]
                move(destination, yaw, "close", 11, 0.007)
                current = displacement()
                change = [
                    current[0] - previous[0],
                    current[1] - previous[1]
                ]
                motion = hypot(change[0], change[1])
                if motion > 0.008:
                    stagnant = 0
                    # Follow measured drawer motion, including a reversed sign.
                    magnitude = hypot(current[0], current[1])
                    if magnitude > 0.025:
                        pull_direction = [
                            current[0] / magnitude,
                            current[1] / magnitude
                        ]
                        front = list(pull_direction)
                else:
                    stagnant += 1
                previous = current
                if travel() >= 0.17:
                    break
                if stagnant >= 2:
                    break

        opened = travel() >= 0.105
        grip("open", 4)
        state = sdk.get_state()
        retreat = list(state["eef_pos"])
        retreat[0] += front[0] * 0.04
        retreat[1] += front[1] * 0.04
        move(retreat, yaw, "open", 7)
        retreat[2] += 0.10
        move(retreat, yaw, "open", 12)

        if opened:
            break
        if sdk.get_state()["steps_used"] - drawer_start > 220:
            break

    if not opened:
        return {
            "status": "failed",
            "reason": "Drawer motion insufficient",
            "drawer_travel": travel(),
            "target": target
        }

    delta = displacement()
    magnitude = hypot(delta[0], delta[1])
    if magnitude > 0.025:
        front = [delta[0] / magnitude, delta[1] / magnitude]
    lateral = [-front[1], front[0]]

    held = False
    grasp_yaw = atan2(front[1], front[0])
    offset = [0.0, 0.0, 0.0]

    for attempt in range(2):
        if remaining() < 100:
            break
        state = sdk.get_state()
        bowl = list(state["objects"][target]["pos"])
        direction = front if attempt == 0 else lateral
        grasp_yaw = atan2(direction[1], direction[0])
        grip("open", 4)
        grasp = [
            bowl[0] + direction[0] * 0.038,
            bowl[1] + direction[1] * 0.038,
            bowl[2] + 0.018
        ]
        hover = [grasp[0], grasp[1], bowl[2] + 0.12]
        move(hover, grasp_yaw, "open", 20)
        move(grasp, grasp_yaw, "open", 20, 0.006)
        state = sdk.get_state()
        if distance(state["eef_pos"], grasp) > 0.025:
            retreat = list(state["eef_pos"])
            retreat[2] += 0.08
            move(retreat, grasp_yaw, "open", 10)
            continue

        before = list(state["objects"][target]["pos"])
        grip("close", 9)
        start = list(sdk.get_state()["eef_pos"])
        move(
            [start[0], start[1], start[2] + 0.085],
            grasp_yaw, "close", 20
        )
        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        eef = state["eef_pos"]
        bowl_rise = bowl[2] - before[2]
        eef_rise = eef[2] - start[2]
        if (
            bowl_rise > 0.035
            and eef_rise > 0.035
            and abs(bowl_rise - eef_rise) < 0.035
            and distance(bowl, eef) < 0.12
        ):
            held = True
            offset = [bowl[i] - eef[i] for i in range(3)]
            break

    if not held:
        return {"status": "failed", "reason": "Bowl lift not verified"}

    state = sdk.get_state()
    eef = list(state["eef_pos"])
    plate_pos = state["objects"][plate]["pos"]
    clearance = max(
        eef[2] + 0.04,
        initial_bowl[2] + 0.15 - offset[2],
        plate_pos[2] + 0.14 - offset[2]
    )
    move([eef[0], eef[1], clearance], grasp_yaw, "close", 18)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    eef = state["eef_pos"]
    if distance(bowl, eef) > 0.13:
        return {"status": "failed", "reason": "Bowl slipped"}

    plate_pos = state["objects"][plate]["pos"]
    move(
        [plate_pos[0] - offset[0], plate_pos[1] - offset[1], clearance],
        grasp_yaw, "close", min(30, max(1, remaining() - 40)), 0.008
    )

    for attempt in range(2):
        if remaining() < 20:
            break
        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        plate_pos = state["objects"][plate]["pos"]
        eef = state["eef_pos"]
        if distance(bowl, eef) > 0.13:
            return {"status": "failed", "reason": "Bowl slipped during transfer"}
        destination = [
            eef[0] + plate_pos[0] - bowl[0],
            eef[1] + plate_pos[1] - bowl[1],
            eef[2] + plate_pos[2] + 0.035 - bowl[2]
        ]
        move(
            destination, grasp_yaw, "close",
            min(22, max(1, remaining() - 18)), 0.006
        )
        state = sdk.get_state()
        bowl = state["objects"][target]["pos"]
        plate_pos = state["objects"][plate]["pos"]
        if (
            hypot(bowl[0] - plate_pos[0], bowl[1] - plate_pos[1]) < 0.025
            and 0.0 < bowl[2] - plate_pos[2] < 0.065
        ):
            break

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    if (
        hypot(bowl[0] - plate_pos[0], bowl[1] - plate_pos[1]) > 0.035
        or bowl[2] - plate_pos[2] > 0.08
        or bowl[2] < plate_pos[2] - 0.005
    ):
        return {"status": "failed", "reason": "Placement approach not verified"}

    grip("open", 6)
    retreat = list(sdk.get_state()["eef_pos"])
    retreat[2] += 0.10
    move(retreat, grasp_yaw, "open", 12)
    grip("open", 4)

    state = sdk.get_state()
    bowl = state["objects"][target]["pos"]
    plate_pos = state["objects"][plate]["pos"]
    if (
        hypot(bowl[0] - plate_pos[0], bowl[1] - plate_pos[1]) < 0.05
        and -0.005 < bowl[2] - plate_pos[2] < 0.075
        and distance(bowl, state["eef_pos"]) > 0.08
    ):
        return {"status": "done", "target": target}
    return {"status": "failed", "reason": "Placement not verified"}
