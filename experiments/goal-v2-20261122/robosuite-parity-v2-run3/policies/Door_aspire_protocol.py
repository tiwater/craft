def door_product(a, b):
    return [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]


def door_turn(q, axis, angle):
    sine = sin(angle * 0.5)
    return door_product(
        [sine * axis[0], sine * axis[1], sine * axis[2], cos(angle * 0.5)],
        q,
    )


def door_budget(state):
    return max(0, state["max_steps"] - state["steps_used"])


def door_control(sdk, target, orientation, grip):
    state = sdk.get_state()
    if door_budget(state) == 0 or state["hinge_qpos"] > 0.30:
        return state
    q = state["eef_quat"]
    error = door_product(orientation, [-q[0], -q[1], -q[2], q[3]])
    length = sqrt(sum(error[i] * error[i] for i in range(3)))
    scale = 0.0
    if length > 0.000001:
        scale = 2.0 * atan2(length, abs(error[3])) / length
        if error[3] < 0.0:
            scale = -scale
    action = [
        max(-0.9, min(0.9, 30.0 * (target[i] - state["eef_pos"][i])))
        for i in range(3)
    ]
    for i in range(3):
        action.append(max(-0.4, min(0.4, scale * error[i] / 0.5)))
    action.append(grip)
    sdk.step_raw(action, steps=1)
    return sdk.get_state()


def run_policy(sdk):
    state = sdk.get_state()
    attempts = 0
    peak_hinge = state["hinge_qpos"]
    peak_latch = abs(state["handle_qpos"])

    # Large handle deflections with slightly negative hinge angles indicate
    # that pulling toward goal_pos loaded the closed-door stop. Use the
    # door normal and measured positive hinge motion to choose opening force.
    while door_budget(state) > 35 and state["hinge_qpos"] <= 0.30:
        attempts += 1
        sdk.set_gripper("open", steps=min(5, door_budget(state)))
        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        yaw = state["objects"]["door"]["yaw"]

        sdk.move_to(
            [handle[0], handle[1], handle[2] + 0.07],
            yaw=yaw,
            gripper="open",
            max_steps=min(25, door_budget(state)),
            tol=0.012,
        )
        state = sdk.get_state()
        for stage in range(2):
            if door_budget(state) == 0 or state["hinge_qpos"] > 0.30:
                break
            handle = list(state["objects"]["handle"]["pos"])
            sdk.move_to(
                [
                    handle[0],
                    handle[1],
                    handle[2] + (0.018 if stage == 0 else 0.0),
                ],
                yaw=yaw,
                gripper="open",
                max_steps=min(12, door_budget(state)),
                tol=0.006,
            )
            state = sdk.get_state()

        if door_budget(state) == 0 or state["hinge_qpos"] > 0.30:
            break

        handle = list(state["objects"]["handle"]["pos"])
        distance = sqrt(sum(
            (state["eef_pos"][i] - handle[i]) ** 2 for i in range(3)
        ))
        if distance > 0.045:
            continue

        base = list(state["eef_quat"])
        for tick in range(min(12, door_budget(state))):
            state = door_control(sdk, handle, base, 1.0)

        handle = list(state["objects"]["handle"]["pos"])
        offset = [
            max(-0.018, min(0.018, state["eef_pos"][i] - handle[i]))
            for i in range(3)
        ]
        reference_hinge = state["hinge_qpos"]
        axis = [-sin(yaw), cos(yaw), 0.0]
        normal = [-sin(yaw), cos(yaw)]
        twist = 0.0
        twist_direction = 1.0
        latch_checkpoint = abs(state["handle_qpos"])
        twist_checkpoint = 0.0
        latch_ticks = 0
        pull = 0.008
        pull_sign = 1.0
        reversed_pull = False
        probe_ticks = 0
        probe_hinge = state["hinge_qpos"]
        lost_ticks = 0
        stalled_ticks = 0
        progress_hinge = state["hinge_qpos"]
        tangent = None
        sample_handle = list(handle)
        sample_hinge = state["hinge_qpos"]
        sample_latch = state["handle_qpos"]
        phase_start = state["steps_used"]

        while door_budget(state) > 0 and state["hinge_qpos"] <= 0.30:
            hinge = state["hinge_qpos"]
            latch = abs(state["handle_qpos"])
            handle = list(state["objects"]["handle"]["pos"])
            elapsed = state["steps_used"] - phase_start
            angle = hinge - reference_hinge

            # Regulate moderate latch travel instead of winding the wrist
            # farther while the door is pressed against its closed stop.
            if hinge < 0.07:
                latch_ticks += 1
                if latch < 0.50:
                    twist += 0.022 * twist_direction
                elif latch > 0.72:
                    twist -= 0.018 * twist_direction
                twist = max(-1.05, min(1.05, twist))

                if latch_ticks >= 14:
                    change = latch - latch_checkpoint
                    motion = twist - twist_checkpoint
                    if abs(motion) > 0.12:
                        if change < -0.025:
                            twist_direction = -twist_direction
                        elif abs(change) < 0.015 and latch < 0.35:
                            twist_direction = -twist_direction
                    latch_checkpoint = latch
                    twist_checkpoint = twist
                    latch_ticks = 0

            c = cos(angle)
            s = sin(angle)
            direction = [
                pull_sign * (c * normal[0] - s * normal[1]),
                pull_sign * (s * normal[0] + c * normal[1]),
            ]

            # Lever rotation also moves the handle site. Learn a hinge
            # tangent only after unlatching, with nearly constant latch angle.
            gain = hinge - sample_hinge
            travel_x = handle[0] - sample_handle[0]
            travel_y = handle[1] - sample_handle[1]
            travel = hypot(travel_x, travel_y)
            if gain > 0.025:
                if (
                    travel > 0.004
                    and abs(state["handle_qpos"] - sample_latch) < 0.10
                ):
                    tangent = [travel_x / travel, travel_y / travel]
                sample_handle = list(handle)
                sample_hinge = hinge
                sample_latch = state["handle_qpos"]

            if tangent is not None and hinge > 0.08:
                dx = 0.8 * tangent[0] + 0.2 * direction[0]
                dy = 0.8 * tangent[1] + 0.2 * direction[1]
                length = hypot(dx, dy)
                if length > 0.000001:
                    direction = [dx / length, dy / length]

            if latch >= 0.40 or hinge > 0.025:
                pull = min(0.065, pull + 0.002)
                probe_ticks += 1
            else:
                pull = max(0.008, pull - 0.003)
                probe_ticks = 0
                probe_hinge = hinge

            # A failed directional probe changes force direction, not just
            # grasp or wrist angle. Never reverse established opening motion.
            if (
                probe_ticks >= 24
                and hinge < 0.025
                and hinge - probe_hinge < 0.006
                and not reversed_pull
            ):
                pull_sign = -pull_sign
                reversed_pull = True
                pull = 0.012
                probe_ticks = 0
                tangent = None
                probe_hinge = hinge

            down = 0.012 if latch < 0.35 and hinge < 0.025 else 0.0
            target = [
                handle[0] + c * offset[0] - s * offset[1] + pull * direction[0],
                handle[1] + s * offset[0] + c * offset[1] + pull * direction[1],
                handle[2] + offset[2] - down,
            ]
            orientation = door_turn(base, axis, twist)
            orientation = door_turn(orientation, [0.0, 0.0, 1.0], angle)
            state = door_control(sdk, target, orientation, 1.0)
            peak_hinge = max(peak_hinge, state["hinge_qpos"])
            peak_latch = max(peak_latch, abs(state["handle_qpos"]))

            handle = state["objects"]["handle"]["pos"]
            separation = sqrt(sum(
                (state["eef_pos"][i] - handle[i]) ** 2 for i in range(3)
            ))
            if separation > 0.09 or (
                state["gripper_width"] < 0.012 and separation > 0.04
            ):
                lost_ticks += 1
            else:
                lost_ticks = 0
            if lost_ticks >= 5:
                break

            if state["hinge_qpos"] > progress_hinge + 0.005:
                progress_hinge = state["hinge_qpos"]
                stalled_ticks = 0
            else:
                stalled_ticks += 1

            if stalled_ticks >= 95 and door_budget(state) > 65:
                break
            if elapsed >= 160 and hinge < 0.04 and door_budget(state) > 65:
                break

    state = sdk.get_state()
    return {
        "status": "done" if state["hinge_qpos"] > 0.30 else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "steps_used": state["steps_used"],
        "grasp_attempts": attempts,
        "peak_hinge_qpos": peak_hinge,
        "peak_handle_deflection": peak_latch,
    }
