def run_policy(sdk):
    def clip(value, limit):
        return max(-limit, min(limit, value))

    def remaining():
        state = sdk.get_state()
        return max(0, min(400, state["max_steps"]) - state["steps_used"])

    def opened(state):
        return state["hinge_qpos"] is not None and state["hinge_qpos"] > 0.30

    def move(target, yaw, ticks):
        count = min(ticks, remaining())
        if count > 0:
            sdk.move_to(
                target, yaw=yaw, gripper="open",
                max_steps=count, tol=0.006
            )

    def actuate(action, ticks):
        count = min(ticks, remaining())
        if count > 0:
            sdk.set_gripper(action, steps=count)

    state = sdk.get_state()
    if state["hinge_qpos"] is None or state["handle_qpos"] is None:
        return {"status": "failed", "reason": "Missing door joint state"}

    best_hinge = state["hinge_qpos"]
    best_latch = abs(state["handle_qpos"])
    preferred_direction = None

    # The low final latch angles and large vertical errors indicate that
    # downward chasing can lose contact. Capture the handle first, keep
    # translation near its center, and identify useful wrist rotation.
    for attempt in range(4):
        if remaining() < 32:
            break
        state = sdk.get_state()
        if opened(state):
            break

        actuate("open", 5)
        state = sdk.get_state()
        if attempt > 0:
            retreat = list(state["eef_pos"])
            retreat[2] += 0.065
            move(retreat, None, 10)

        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        door_yaw = state["objects"]["door"]["yaw"]
        yaw_offsets = [0.0, pi / 2.0, pi / 4.0, -pi / 4.0]
        yaw = door_yaw + yaw_offsets[attempt]
        move([handle[0], handle[1], handle[2] + 0.055], yaw, 20)
        state = sdk.get_state()
        move(list(state["objects"]["handle"]["pos"]), yaw, 23)
        actuate("close", 9)

        state = sdk.get_state()
        if opened(state):
            break
        handle = list(state["objects"]["handle"]["pos"])
        offset = [
            clip(state["eef_pos"][axis] - handle[axis], 0.010)
            for axis in range(3)
        ]

        c = cos(door_yaw)
        s = sin(door_yaw)
        wrist_axes = [
            [c, s, 0.0],
            [-c, -s, 0.0],
            [-s, c, 0.0],
            [s, -c, 0.0],
        ]
        wrist_index = 0
        wrist_scores = [-100.0, -100.0, -100.0, -100.0]
        wrist_start = abs(state["handle_qpos"])
        wrist_ticks = 0
        wrist_selected = False
        latch_stall = 0

        # The door body position need not be the hinge pivot. Probe
        # horizontal directions and select them by measured hinge motion.
        directions = [
            [-c, -s], [c, s], [s, -c], [-s, c],
            [(-c + s) / sqrt(2.0), (-s - c) / sqrt(2.0)],
            [(c - s) / sqrt(2.0), (s + c) / sqrt(2.0)],
            [(-c - s) / sqrt(2.0), (-s + c) / sqrt(2.0)],
            [(c + s) / sqrt(2.0), (s - c) / sqrt(2.0)],
        ]
        if preferred_direction is not None:
            directions = [preferred_direction] + directions

        direction_index = 0
        direction_ticks = 0
        direction_start = state["hinge_qpos"]
        direction_best = direction_start
        locked_direction = False
        pull_direction = list(directions[0])
        pull_reference = state["hinge_qpos"]
        hinge_stall = 0
        lost = 0
        latch_ready = abs(state["handle_qpos"]) > 0.32
        attempt_start = state["steps_used"]
        previous_latch = abs(state["handle_qpos"])

        while remaining() > 0:
            state = sdk.get_state()
            hinge = state["hinge_qpos"]
            latch = abs(state["handle_qpos"])
            best_hinge = max(best_hinge, hinge)
            best_latch = max(best_latch, latch)
            if opened(state):
                break

            elapsed = state["steps_used"] - attempt_start
            if elapsed >= 140 and not locked_direction:
                break

            handle = list(state["objects"]["handle"]["pos"])
            eef = state["eef_pos"]
            distance = sqrt(sum(
                (eef[axis] - handle[axis]) ** 2
                for axis in range(3)
            ))
            if distance > 0.095 or (
                elapsed > 12 and state["gripper_width"] < 0.009
                and distance > 0.035
            ):
                lost += 1
            else:
                lost = 0
            if lost >= 5:
                break

            if latch > previous_latch + 0.006:
                latch_stall = 0
            else:
                latch_stall += 1
            previous_latch = latch

            if not wrist_selected:
                if latch > 0.38:
                    wrist_selected = True
                elif wrist_ticks >= 9:
                    wrist_scores[wrist_index] = latch - wrist_start
                    wrist_index += 1
                    if wrist_index >= len(wrist_axes):
                        wrist_index = 0
                        for index in range(1, len(wrist_scores)):
                            if wrist_scores[index] > wrist_scores[wrist_index]:
                                wrist_index = index
                        wrist_selected = True
                    wrist_start = latch
                    wrist_ticks = 0

            if latch > 0.34:
                latch_ready = True
            elif latch < 0.22 and hinge < 0.045:
                latch_ready = False

            wrist_rate = clip(0.45 * (0.48 - latch), 0.12)
            if latch > 0.48:
                wrist_rate = 0.0
            if hinge > 0.10:
                wrist_rate = 0.0

            if wrist_selected and latch_stall >= 24 and latch < 0.28:
                wrist_index = (wrist_index + 1) % len(wrist_axes)
                latch_stall = 0

            pull = 0.0
            if latch_ready or hinge > 0.045:
                if hinge > direction_best + 0.002:
                    direction_best = hinge
                    hinge_stall = 0
                else:
                    hinge_stall += 1

                if not locked_direction:
                    direction_ticks += 1
                    pull_direction = list(directions[direction_index])
                    if hinge > direction_start + 0.012:
                        locked_direction = True
                        preferred_direction = list(pull_direction)
                        pull_reference = hinge
                        hinge_stall = 0
                    elif direction_ticks >= 11:
                        direction_index += 1
                        if direction_index >= len(directions):
                            break
                        direction_ticks = 0
                        direction_start = hinge
                        direction_best = hinge
                        pull_direction = list(directions[direction_index])

                if locked_direction and hinge_stall > 24:
                    locked_direction = False
                    direction_index = (direction_index + 1) % len(directions)
                    direction_ticks = 0
                    direction_start = hinge
                    direction_best = hinge
                    hinge_stall = 0

                pull = 0.045 if not locked_direction else 0.065
                if not locked_direction and direction_ticks < 2:
                    pull = 0.0

            angle = hinge - pull_reference if locked_direction else 0.0
            direction = [
                cos(angle) * pull_direction[0] -
                sin(angle) * pull_direction[1],
                sin(angle) * pull_direction[0] +
                cos(angle) * pull_direction[1],
            ]

            # Small vertical assistance preserves finger engagement;
            # wrist torque supplies most of the latch rotation.
            depression = min(0.012, max(0.0, 0.04 * (0.40 - latch)))
            if hinge > 0.08:
                depression = 0.0
            target = [
                handle[0] + offset[0] + pull * direction[0],
                handle[1] + offset[1] + pull * direction[1],
                handle[2] + offset[2] - depression,
            ]
            translation = [
                clip(18.0 * (target[axis] - eef[axis]), 0.75)
                for axis in range(3)
            ]
            wrist_axis = wrist_axes[wrist_index]
            sdk.step_raw([
                translation[0], translation[1], translation[2],
                wrist_rate * wrist_axis[0],
                wrist_rate * wrist_axis[1],
                wrist_rate * wrist_axis[2],
                1.0,
            ], steps=1)
            wrist_ticks += 1

            if elapsed >= 62 and latch < 0.18 and hinge < 0.035:
                break

    state = sdk.get_state()
    return {
        "status": "done" if opened(state) else "failed",
        "hinge_qpos": state["hinge_qpos"],
        "handle_qpos": state["handle_qpos"],
        "best_hinge_qpos": max(best_hinge, state["hinge_qpos"]),
        "best_handle_magnitude": max(best_latch, abs(state["handle_qpos"])),
        "steps_used": state["steps_used"],
    }
