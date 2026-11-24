def run_policy(sdk):
    def clip(value, limit):
        return max(-limit, min(limit, value))

    def remaining(state):
        return min(400, state["max_steps"]) - state["steps_used"]

    def successful(state):
        return state["hinge_qpos"] is not None and state["hinge_qpos"] > 0.30

    def door_normal(state):
        q = state["objects"]["door"]["quat"]
        x, y, z, w = q
        normal = [2.0 * (x * y - z * w),
                  1.0 - 2.0 * (x * x + z * z), 0.0]
        length = hypot(normal[0], normal[1])
        if length < 0.1:
            handle = state["objects"]["handle"]["pos"]
            door = state["objects"]["door"]["pos"]
            normal = [-(handle[1] - door[1]), handle[0] - door[0], 0.0]
            length = hypot(normal[0], normal[1])
        if length < 0.001:
            return [1.0, 0.0, 0.0]
        return [normal[0] / length, normal[1] / length, 0.0]

    def move(position, yaw, grip, budget):
        state = sdk.get_state()
        ticks = min(budget, remaining(state))
        if ticks > 0:
            sdk.move_to(position, yaw=yaw, gripper=grip,
                        max_steps=ticks, tol=0.008)

    def track(position, rotation, ticks):
        for tick in range(ticks):
            state = sdk.get_state()
            if remaining(state) <= 0 or successful(state):
                return
            eef = state["eef_pos"]
            action = [clip((position[i] - eef[i]) / 0.05, 0.65)
                      for i in range(3)]
            action += [clip(rotation[i], 0.35) for i in range(3)]
            action += [1.0]
            sdk.step_raw(action, steps=1)

    state = sdk.get_state()
    if successful(state):
        return {"status": "done", "hinge_qpos": state["hinge_qpos"]}

    turn_sign = 1.0
    pull_sign = 1.0

    for attempt in range(3):
        state = sdk.get_state()
        if remaining(state) < 35:
            break

        handle = state["objects"]["handle"]["pos"]
        normal = door_normal(state)
        toward_robot = sum((state["eef_pos"][i] - handle[i]) * normal[i]
                           for i in range(3))
        if toward_robot < 0.0:
            normal = [-value for value in normal]

        yaw = atan2(normal[1], normal[0])
        if attempt == 1:
            yaw += pi / 2.0

        approach = [handle[i] + 0.07 * normal[i] for i in range(3)]
        approach[2] += 0.075
        move(approach, yaw, "open", 30)

        state = sdk.get_state()
        handle = state["objects"]["handle"]["pos"]
        approach = [handle[i] + 0.025 * normal[i] for i in range(3)]
        move(approach, yaw, "open", 22)

        state = sdk.get_state()
        handle = list(state["objects"]["handle"]["pos"])
        move(handle, yaw, "open", 18)

        state = sdk.get_state()
        ticks = min(12, remaining(state))
        if ticks <= 0:
            break
        sdk.set_gripper("close", steps=ticks)

        state = sdk.get_state()
        handle = state["objects"]["handle"]["pos"]
        offset = [clip(state["eef_pos"][i] - handle[i], 0.018)
                  for i in range(3)]
        previous_latch = state["handle_qpos"]
        if previous_latch is None:
            previous_latch = 0.0

        # Probe both wrist directions, keeping the grasp centered on the
        # moving handle. Retain the direction that increases latch angle.
        for block in range(7):
            state = sdk.get_state()
            if successful(state) or remaining(state) <= 0:
                break
            latch = state["handle_qpos"]
            if latch is not None and latch > 0.40:
                break

            handle = state["objects"]["handle"]["pos"]
            target = [handle[i] + offset[i] for i in range(3)]
            target[2] -= 0.008
            rotation = [turn_sign * 0.22 * normal[i] for i in range(3)]
            track(target, rotation, 5)

            state = sdk.get_state()
            latch = state["handle_qpos"]
            if latch is None:
                latch = 0.0
            if block == 1 and latch < previous_latch + 0.015:
                turn_sign = -turn_sign
            previous_latch = latch

        state = sdk.get_state()
        previous_hinge = state["hinge_qpos"]
        if previous_hinge is None:
            previous_hinge = 0.0
        stalled = 0

        # Recompute the pull direction from the live door orientation as
        # the door swings. Small targets avoid pulling away from the handle.
        for block in range(32):
            state = sdk.get_state()
            if successful(state):
                return {"status": "done",
                        "hinge_qpos": state["hinge_qpos"],
                        "steps_used": state["steps_used"]}
            if remaining(state) <= 0:
                break

            current_normal = door_normal(state)
            if sum(current_normal[i] * normal[i] for i in range(3)) < 0.0:
                current_normal = [-value for value in current_normal]
            normal = current_normal

            handle = state["objects"]["handle"]["pos"]
            target = [handle[i] + offset[i] +
                      pull_sign * 0.035 * normal[i] for i in range(3)]

            latch = state["handle_qpos"]
            twisting = 0.0
            if latch is not None and latch < 0.35:
                twisting = turn_sign * 0.16
                target[2] -= 0.008

            rotation = [twisting * normal[i] for i in range(3)]
            track(target, rotation, 4)

            state = sdk.get_state()
            hinge = state["hinge_qpos"]
            if hinge is None:
                hinge = 0.0
            progress = hinge - previous_hinge
            if progress < 0.001:
                stalled += 1
            else:
                stalled = 0

            if progress < -0.003:
                pull_sign = -pull_sign
                stalled = 0
            elif stalled == 4:
                pull_sign = -pull_sign
            elif stalled >= 9:
                break

            previous_hinge = hinge

        state = sdk.get_state()
        if successful(state):
            return {"status": "done", "hinge_qpos": state["hinge_qpos"],
                    "steps_used": state["steps_used"]}
        if remaining(state) > 0:
            sdk.set_gripper("open", steps=min(5, remaining(state)))

    state = sdk.get_state()
    return {"status": "done" if successful(state) else "failed",
            "hinge_qpos": state["hinge_qpos"],
            "handle_qpos": state["handle_qpos"],
            "steps_used": state["steps_used"]}
