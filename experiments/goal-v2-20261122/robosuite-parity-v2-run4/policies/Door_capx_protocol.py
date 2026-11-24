def door_remaining(sdk):
    state = sdk.get_state()
    return max(0, state["max_steps"] - state["steps_used"])


def door_distance(a, b):
    return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def door_rotation(current, desired):
    x, y, z, w = current
    a, b, c, d = desired
    vector = [
        -d * x + a * w - b * z + c * y,
        -d * y + a * z + b * w - c * x,
        -d * z - a * y + b * x + c * w
    ]
    sign = 1.0 if d * w + a * x + b * y + c * z >= 0.0 else -1.0
    return [max(-0.4, min(0.4, 3.0 * sign * value)) for value in vector]


def door_step(sdk, target, rotation, grip):
    state = sdk.get_state()
    if state["steps_used"] < state["max_steps"]:
        translation = [
            max(-0.9, min(0.9, 35.0 * (target[i] - state["eef_pos"][i])))
            for i in range(3)
        ]
        sdk.step_raw(translation + rotation + [grip], steps=1)


def run_policy(sdk):
    state = sdk.get_state()
    reference = list(state["eef_quat"])
    initial_hinge = state["hinge_qpos"]
    yaw = state["objects"]["door"]["yaw"]
    normal = [-sin(yaw), cos(yaw)]
    radial = [cos(yaw), sin(yaw)]
    attempt = 0

    while door_remaining(sdk) > 15:
        state = sdk.get_state()
        if state["hinge_qpos"] > 0.30:
            break

        sdk.set_gripper("open", steps=min(6, door_remaining(sdk)))
        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        clearance = [
            state["eef_pos"][0],
            state["eef_pos"][1],
            max(state["eef_pos"][2], handle[2] + 0.09)
        ]
        for tick in range(min(16, door_remaining(sdk))):
            state = sdk.get_state()
            door_step(
                sdk, clearance,
                door_rotation(state["eef_quat"], reference), -1.0
            )

        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        if door_remaining(sdk) > 0:
            sdk.move_to(
                [handle[0], handle[1], handle[2] + 0.085],
                gripper="open",
                max_steps=min(28, door_remaining(sdk)),
                tol=0.008
            )

        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        shift = 0.0
        if attempt % 3 == 1:
            shift = 0.008
        elif attempt % 3 == 2:
            shift = -0.008
        target = [
            handle[0] + shift * radial[0],
            handle[1] + shift * radial[1],
            handle[2]
        ]
        if door_remaining(sdk) > 0:
            sdk.move_to(
                target, gripper="open",
                max_steps=min(30, door_remaining(sdk)), tol=0.005
            )
        if door_remaining(sdk) == 0:
            break
        sdk.set_gripper("close", steps=min(12, door_remaining(sdk)))

        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        if door_distance(state["eef_pos"], handle) > 0.05:
            attempt += 1
            continue

        offset = [
            max(-0.015, min(0.015, state["eef_pos"][i] - handle[i]))
            for i in range(3)
        ]
        latch = state["handle_qpos"]
        if latch is None:
            latch = 0.0
        latch_start = latch
        turn_sign = 1.0
        hold_quat = list(state["eef_quat"])
        unlocked = abs(latch) > 0.35
        anchor_handle = list(handle)
        anchor_hinge = state["hinge_qpos"]
        tangent = None
        lost = 0
        stalled = 0
        best_hinge = state["hinge_qpos"]

        for tick in range(min(240, door_remaining(sdk))):
            state = sdk.get_state()
            hinge = state["hinge_qpos"]
            if hinge > 0.30:
                break

            handle = list(state["objects"]["handle"]["pos"])
            latch = state["handle_qpos"]
            if latch is None:
                latch = 0.0
            separation = door_distance(state["eef_pos"], handle)

            if separation > 0.075 or (
                state["gripper_width"] < 0.006 and tick > 15
            ):
                lost += 1
            else:
                lost = 0
            if lost >= 8:
                break

            if hinge > best_hinge + 0.002:
                best_hinge = hinge
                stalled = 0
            else:
                stalled += 1

            angle = hinge - initial_hinge
            direction = [
                cos(angle) * normal[i] - sin(angle) * radial[i]
                for i in range(2)
            ]

            gain = hinge - anchor_hinge
            if gain > 0.012:
                dx = handle[0] - anchor_handle[0]
                dy = handle[1] - anchor_handle[1]
                travel = hypot(dx, dy)
                if travel > 0.002:
                    tangent = [dx / travel, dy / travel]
                anchor_handle = list(handle)
                anchor_hinge = hinge

            if tangent is not None and hinge > 0.05:
                direction = [
                    0.5 * direction[i] + 0.5 * tangent[i]
                    for i in range(2)
                ]
                magnitude = max(0.001, hypot(direction[0], direction[1]))
                direction = [value / magnitude for value in direction]

            if not unlocked and (abs(latch) > 0.38 or hinge > 0.025):
                unlocked = True
                hold_quat = list(state["eef_quat"])
                stalled = 0

            if unlocked and abs(latch) < 0.22 and hinge < 0.025:
                unlocked = False

            pull = 0.045 if unlocked else 0.003
            if unlocked and stalled > 35:
                pull = 0.065
            if separation > 0.05:
                pull *= 0.5

            target = [
                handle[0] + offset[0] + pull * direction[0],
                handle[1] + offset[1] + pull * direction[1],
                handle[2] + offset[2]
            ]

            if unlocked:
                rotation = door_rotation(state["eef_quat"], hold_quat)
            else:
                if tick == 16 and latch < latch_start - 0.025:
                    turn_sign = -1.0
                turn = 0.18 * turn_sign
                rotation = [turn * normal[0], turn * normal[1], 0.0]
                target[2] -= 0.006

            door_step(sdk, target, rotation, 1.0)

            if stalled > 110:
                break

        attempt += 1

    state = sdk.get_state()
    return {
        "status": "done" if state["hinge_qpos"] > 0.30 else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "steps_used": state["steps_used"]
    }
