"""Verify the common gait against the real planner and a fake executor."""

import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import uuid

from planner_node_smoke import spin_for, wait_for


STANDING = [500, 540, 210] * 3 + [500, 460, 790] * 3
WALK_PERIOD_S = 4.0
WALK_DUTY = 0.55
SWING_INTERVALS = 9
WALK_DURATIONS = [200] * (2 * (1 + SWING_INTERVALS))
LIFT_HEIGHT_MM = 40.0
OUTWARD_OFFSET_MM = 25.0
# An odd number of swing intervals places two samples equally far from the midpoint.
BEFORE_APEX_STEP = SWING_INTERVALS // 2
BEFORE_APEX_PHASE = BEFORE_APEX_STEP / SWING_INTERVALS
NEAR_APEX_FRACTION = (4 * BEFORE_APEX_PHASE * (1 - BEFORE_APEX_PHASE)) ** 3
RECOVERY_DURATIONS = ([200] * 4 + [400] * 4) * 2
FIRST_SPEED = 0.002847222222222222
FORWARD_SPEED_LIMIT = 0.035
TELEOP_REVERSE_SPEED = 0.035
TELEOP_LATERAL_SPEED = 0.020
TELEOP_YAW_SPEED = 0.20
LINEAR_STEP = 0.00542
YAW_STEP = 0.03
RAMP_UP_SPEEDS = [step * LINEAR_STEP for step in range(1, 7)]
FORWARD_CYCLE_SPEEDS = [*RAMP_UP_SPEEDS, FORWARD_SPEED_LIMIT, *reversed(RAMP_UP_SPEEDS)]


def foot_in_body(config, values, leg, first_servo_id):
    servo, mounting, geometry = (config[key] for key in ("servo", "mounting", "model_geometry"))
    angles = []
    first_index = first_servo_id - 1
    for index, position in enumerate(values[first_index:first_index + 3], start=first_servo_id):
        key = str(index)
        relative = (position - servo["reference_position"]) * servo["degrees_per_position_unit"]
        angles.append(math.radians(mounting["joint_reference_angles_deg"][key] +
                                   mounting["joint_angle_signs"][key] * relative))
    yaw, pitch, knee = angles
    second = geometry["second_leg_length"]
    third = -geometry["actual_foot_contact_point"]["position_mm"][1]
    reach = geometry["first_leg_length"] + second * math.cos(pitch) + third * math.cos(pitch + knee)
    height = second * math.sin(pitch) + third * math.sin(pitch + knee)
    local = (reach * math.sin(yaw), -reach * math.cos(yaw), height)
    transform = mounting["fixed_body_to_first_axis_transforms"][leg]
    return tuple(origin + sum(coefficient * value for coefficient, value in zip(row, local, strict=True))
                 for origin, row in zip(transform["translation_mm"], transform["rotation"], strict=True))


def right_front_foot(config, values):
    return foot_in_body(config, values, "right_front", 1)


def check_plan(plan, config, cycles, recovery=None):
    walk_frames = len(WALK_DURATIONS)
    walked = cycles * walk_frames
    actual_recovery = len(plan.segments) > walked
    if recovery is not None:
        assert actual_recovery == recovery
    expected = WALK_DURATIONS * cycles + (RECOVERY_DURATIONS if actual_recovery else [])
    actual = [segment.move_duration_ms for segment in plan.segments]
    assert actual == expected, (actual, expected)
    boundaries = [index for index, segment in enumerate(plan.segments, start=1) if segment.walking_cycle_end]
    assert boundaries == [walk_frames * index for index in range(1, cycles + 1)]
    offset = 0
    servo, mounting = (config[key] for key in ("servo", "mounting"))
    for segment in plan.segments:
        assert segment.start_offset_ms == offset
        assert len(segment.positions) == 18
        for index, position in enumerate(segment.positions, start=1):
            axis = str(index)
            angle = mounting["joint_reference_angles_deg"][axis] + mounting["joint_angle_signs"][axis] * (
                position - servo["reference_position"]) * servo["degrees_per_position_unit"]
            limits = mounting["installed_joint_limits"][axis]
            assert limits["min_deg"] - 1e-10 <= angle <= limits["max_deg"] + 1e-10
        offset += segment.move_duration_ms
    nominal = right_front_foot(config, STANDING)
    for boundary in boundaries:
        lift = max(right_front_foot(config, segment.positions)[2] - nominal[2]
                   for segment in plan.segments[boundary - walk_frames:boundary])
        assert math.isclose(lift, LIFT_HEIGHT_MM * NEAR_APEX_FRACTION, abs_tol=1.0), (boundary, lift)
    if actual_recovery:
        recovery_lift = max(right_front_foot(config, segment.positions)[2] - nominal[2]
                            for segment in plan.segments[walked:])
        assert math.isclose(recovery_lift, 8.0, abs_tol=1.0), recovery_lift
        assert math.dist(right_front_foot(config, plan.segments[-1].positions), nominal) <= 1.0
    return boundaries


def landing_error(plan, config, boundary, velocity, leg, first_servo_id):
    # At the cycle boundary both right corner legs land at their nominal positions in the projected
    # body pose half a stance ahead. Express that target in the body frame at the actual cycle boundary.
    x, y, _ = foot_in_body(config, STANDING, leg, first_servo_id)
    vx, vy, yaw_rate = velocity
    half_stance = WALK_PERIOD_S * WALK_DUTY / 2
    angle = yaw_rate * half_stance
    sine, cosine = math.sin(angle), math.cos(angle)
    along = sine / yaw_rate if yaw_rate else half_stance
    across = (1 - cosine) / yaw_rate if yaw_rate else 0
    expected = (cosine * x - sine * y + 1000 * (along * vx - across * vy),
                sine * x + cosine * y + 1000 * (across * vx + along * vy))
    measured = foot_in_body(config, plan.segments[boundary - 1].positions, leg, first_servo_id)[:2]
    return math.dist(measured, expected)


def check_landing(plan, config, boundary, velocity):
    error = landing_error(plan, config, boundary, velocity, "right_front", 1)
    assert error <= 1.0, (boundary, velocity, error)


def teleop_landing_error(plan, config, velocity):
    return max(landing_error(plan, config, len(WALK_DURATIONS), velocity, leg, first_servo_id)
               for leg, first_servo_id in (("right_front", 1), ("right_rear", 7)))


def step_toward(previous, target):
    dx, dy, dw = (after - before for before, after in zip(previous, target, strict=True))
    linear = math.hypot(dx, dy)
    fraction = min(1.0, LINEAR_STEP / linear if linear else 1.0, YAW_STEP / abs(dw) if dw else 1.0)
    return target if fraction == 1.0 else (previous[0] + fraction * dx,
                                          previous[1] + fraction * dy, previous[2] + fraction * dw)


def check_forward_speeds(plan, config, boundaries, speeds):
    for boundary, speed in zip(boundaries, speeds, strict=True):
        check_landing(plan, config, boundary, (speed, 0.0, 0.0))


def check_forward_outward_offset(plan, config, first_speed):
    start = right_front_foot(config, STANDING)
    sample_index = len(WALK_DURATIONS) - SWING_INTERVALS + BEFORE_APEX_STEP - 1
    sample = right_front_foot(config, plan.segments[sample_index].positions)
    landed = right_front_foot(config, plan.segments[len(WALK_DURATIONS) - 1].positions)
    root = config["mounting"]["fixed_body_to_first_axis_transforms"]["right_front"]["translation_mm"]
    cycle_advance = first_speed * WALK_PERIOD_S * 1000.0
    root_x_at_lift = root[0] + cycle_advance * WALK_DUTY
    direction = (start[0] - root_x_at_lift, start[1] - root[1])
    # Express the sample and landing in the planning frame, then subtract the quintic swing baseline.
    phase = BEFORE_APEX_PHASE
    cycle_ratio = WALK_DUTY + (1 - WALK_DUTY) * phase
    blend = phase ** 3 * (10 - 15 * phase + 6 * phase ** 2)
    baseline_x = (1 - blend) * start[0] + blend * (landed[0] + cycle_advance)
    baseline_y = (1 - blend) * start[1] + blend * landed[1]
    measured = (sample[0] + cycle_advance * cycle_ratio - baseline_x, sample[1] - baseline_y)
    expected = tuple(OUTWARD_OFFSET_MM * NEAR_APEX_FRACTION * component / math.hypot(*direction)
                     for component in direction)
    assert math.dist(measured, expected) <= 1.0, (measured, expected)


def exercise(executable, hardware_config, child_log):
    import rclpy
    from hexapod_interfaces.msg import ExecutionStatus, MotionCommand, PlannerStatus, PlanStop, ServoPlan, StandingReference
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import DurabilityPolicy, QoSProfile

    config = json.loads(hardware_config.read_text(encoding="utf-8"))
    namespace = "/planner_gait_" + uuid.uuid4().hex
    args = [str(executable), "--ros-args", "-r", "__ns:=" + namespace,
            "-p", "hardware_config:=" + json.dumps(str(hardware_config)),
            "-p", "standing_positions:=" + json.dumps(STANDING), "-p", "plan_lead_ms:=1000",
            "-p", "command_timeout_ms:=3500"]
    rclpy.init(args=[])
    fake = rclpy.create_node("fake_executor", namespace=namespace)
    clock = Clock(clock_type=ClockType.SYSTEM_TIME)
    plans, statuses, stops = [], [], []
    plan_sub = fake.create_subscription(ServoPlan, "servo_plan", plans.append, 10)
    status_sub = fake.create_subscription(PlannerStatus, "planner_status", statuses.append, 30)
    stop_sub = fake.create_subscription(PlanStop, "plan_stop", stops.append, 10)
    command_pub = fake.create_publisher(MotionCommand, "motion_command", 10)
    reference_pub = fake.create_publisher(StandingReference, "standing_reference",
                                          QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    execution_pub = fake.create_publisher(ExecutionStatus, "execution_status", 10)
    sequence = 0

    def request(kind, stamp=None, **fields):
        nonlocal sequence
        sequence += 1
        command_pub.publish(MotionCommand(kind=kind, sequence=sequence,
                                          stamp=clock.now().to_msg() if stamp is None else stamp, **fields))
        return sequence

    def reference():
        stamp = clock.now().to_msg()
        reference_pub.publish(StandingReference(stamp=stamp, ready=True))
        wait_for(fake, rclpy, lambda: any(s.state == "standing" and s.standing_stamp == stamp for s in statuses),
                 "standing reference")
        return stamp

    def walk(cycles, vx, vy=0.0, wz=0.0):
        return request(MotionCommand.WALK_CYCLES, vx_m_s=vx, vy_m_s=vy, wz_rad_s=wz, cycle_count=cycles)

    def joystick(x, y, yaw):
        return request(MotionCommand.TELEOP, vx_m_s=x, vy_m_s=y, wz_rad_s=yaw)

    def find_plan(request_sequence, after=0):
        for status in statuses:
            if status.command_sequence != request_sequence:
                continue
            if status.state in ("rejected", "fault"):
                raise AssertionError(f"Planner {status.state}: {status.reason}")
            if status.state == "planned" and status.plan_id > after:
                return next((plan for plan in plans if plan.plan_id == status.plan_id), None)
        return None

    def get_plan(request_sequence, after=0):
        wait_for(fake, rclpy, lambda: find_plan(request_sequence, after), "gait plan", timeout_s=10.0)
        return find_plan(request_sequence, after)

    def reply(plan, state, count=0):
        execution_pub.publish(ExecutionStatus(plan_id=plan.plan_id, state=state, completed_segments=count))

    def accept(plan):
        reply(plan, ExecutionStatus.ACCEPTED)
        wait_for(fake, rclpy, lambda: any(s.plan_id == plan.plan_id and s.state == "accepted" for s in statuses),
                 "execution acknowledgement")

    def complete(plan):
        accept(plan)
        reply(plan, ExecutionStatus.COMPLETED, len(plan.segments))

    def stop(plan, prefix):
        stop_sequence = request(MotionCommand.STOP)
        wait_for(fake, rclpy, lambda: any(s.plan_id == plan.plan_id for s in stops), "plan stop")
        reply(plan, ExecutionStatus.INTERRUPTED, prefix)
        return stop_sequence

    def standing(sequence, plan_id):
        wait_for(fake, rclpy, lambda: any(s.state == "standing" and s.command_sequence == sequence
                                         and s.plan_id == plan_id for s in statuses), "returned to standing")

    def reject_velocity(velocity):
        number = walk(1, *velocity)

        def rejected():
            return next((s for s in statuses if s.command_sequence == number and s.state == "rejected"), None)

        wait_for(fake, rclpy, rejected, f"velocity limit rejection for {velocity}")
        assert "velocity limits" in rejected().reason
        assert not any(s.command_sequence == number and s.state == "planned" for s in statuses)

    def hold_joystick(sample, target):
        confirmed = reference()
        number = joystick(*sample)
        previous_id, previous_positions = 0, STANDING
        speed = (0.0, 0.0, 0.0)
        at_target = False
        while True:
            speed = step_toward(speed, target)
            plan = get_plan(number, after=previous_id)
            assert plan.standing_stamp == confirmed and list(plan.initial_positions) == previous_positions
            check_plan(plan, config, 1, recovery=False)
            error = teleop_landing_error(plan, config, speed)
            assert error <= 1.0, (sample, speed, error)
            accept(plan)
            if at_target:
                return plan, number
            at_target = speed == target
            reply(plan, ExecutionStatus.COMPLETED, len(plan.segments))
            previous_id, previous_positions = plan.plan_id, list(plan.segments[-1].positions)

    def finish_joystick_stop(current, number=None, reason=None):
        if number is None:
            number = request(MotionCommand.STOP)

        def stopping():
            return next((s for s in statuses if s.state == "stopping" and s.command_sequence == number), None)

        wait_for(fake, rclpy, stopping, "joystick stop")
        if reason is not None:
            assert stopping().reason == reason

        stop_plans = []
        # The speed limits and increments bound the number of braking cycles.
        for _ in range(9):
            reply(current, ExecutionStatus.COMPLETED, len(current.segments))

            def returned():
                return any(s.state == "standing" and s.command_sequence == number for s in statuses)

            def next_stop_plan():
                return find_plan(number, after=current.plan_id)

            wait_for(fake, rclpy, lambda: returned() or next_stop_plan(), "joystick braking or standing")
            if returned():
                return stop_plans
            following = next_stop_plan()
            assert list(following.initial_positions) == list(current.segments[-1].positions)
            cycles = sum(segment.walking_cycle_end for segment in following.segments)
            assert cycles in (0, 1)
            check_plan(following, config, cycles, recovery=cycles == 0)
            stop_plans.append(following)
            accept(following)
            current = following
        raise AssertionError("Joystick stop did not finish braking and return to standing")

    try:
        with subprocess.Popen(args, stdout=child_log, stderr=subprocess.STDOUT) as process:
            try:
                wait_for(fake, rclpy, lambda: all(p.get_subscription_count() for p in
                         (command_pub, reference_pub, execution_pub)) and all(s.get_publisher_count() for s in
                         (plan_sub, status_sub, stop_sub)), "topic discovery", timeout_s=8.0)
                confirmed = reference()
                for velocity in ((0.0351, 0.0, 0.0), (-0.0051, 0.0, 0.0), (0.0, 0.0051, 0.0),
                                 (0.008, 0.0001, 0.0), (0.008, 0.0, 0.0001), (0.0, 0.0, 0.031)):
                    reject_velocity(velocity)
                plan = get_plan(walk(len(FORWARD_CYCLE_SPEEDS), FORWARD_SPEED_LIMIT))
                assert plan.standing_stamp == confirmed and list(plan.initial_positions) == STANDING
                boundaries = check_plan(plan, config, len(FORWARD_CYCLE_SPEEDS), recovery=True)
                check_forward_speeds(plan, config, boundaries, FORWARD_CYCLE_SPEEDS)
                check_forward_outward_offset(plan, config, FORWARD_CYCLE_SPEEDS[0])
                accept(plan)
                # The fake executor is authoritative about having finished a cycle; PTY tests check actual timing.
                prefix = boundaries[len(RAMP_UP_SPEEDS)]
                stop_sequence = stop(plan, prefix)
                previous_id, previous_positions = plan.plan_id, list(plan.segments[prefix - 1].positions)
                for step in range(1, len(RAMP_UP_SPEEDS) + 1):
                    speed = FORWARD_SPEED_LIMIT - step * LINEAR_STEP
                    braking = get_plan(stop_sequence, after=previous_id)
                    assert list(braking.initial_positions) == previous_positions
                    boundaries = check_plan(braking, config, 1, recovery=False)
                    check_forward_speeds(braking, config, boundaries, [speed])
                    complete(braking)
                    previous_id, previous_positions = braking.plan_id, list(braking.segments[-1].positions)
                recovery = get_plan(stop_sequence, after=previous_id)
                assert list(recovery.initial_positions) == previous_positions
                check_plan(recovery, config, 0, recovery=True)
                complete(recovery)
                standing(stop_sequence, recovery.plan_id)

                reference()
                short = get_plan(walk(1, -0.005))
                boundaries = check_plan(short, config, 1, recovery=True)
                check_forward_speeds(short, config, boundaries, [-FIRST_SPEED])
                accept(short)
                standing(stop(short, 0), 0)

                reference()
                slower = get_plan(walk(3, 0.005))
                boundaries = check_plan(slower, config, 3, recovery=True)
                check_forward_speeds(slower, config, boundaries, [0.005] * 3)

                for velocity in ((0.0, 0.005, 0.0), (0.0, 0.0, -0.03), (0.004, 0.003, 0.03)):
                    confirmed = reference()
                    directional = get_plan(walk(1, *velocity))
                    assert directional.standing_stamp == confirmed and list(directional.initial_positions) == STANDING
                    boundary, = check_plan(directional, config, 1)
                    check_landing(directional, config, boundary, tuple(value * FIRST_SPEED / 0.005 for value in velocity))

                reverse, _ = hold_joystick((-1.0, 0.0, 0.0), (-TELEOP_REVERSE_SPEED, 0.0, 0.0))
                reverse_stop = finish_joystick_stop(reverse)
                expected_brakes = [(-TELEOP_REVERSE_SPEED + step * LINEAR_STEP, 0.0, 0.0)
                                   for step in range(1, 7)]
                assert len(reverse_stop) == len(expected_brakes) + 1
                for braking, speed in zip(reverse_stop[:-1], expected_brakes, strict=True):
                    assert braking.segments[-1].walking_cycle_end
                    error = teleop_landing_error(braking, config, speed)
                    assert error <= 1.0, (speed, error)
                assert not any(segment.walking_cycle_end for segment in reverse_stop[-1].segments)

                mixed = (0.12, -0.16, 0.1)
                mix_scale = 0.2 / 0.38
                combined_speed = (mixed[0] * FORWARD_SPEED_LIMIT * mix_scale,
                                  mixed[1] * TELEOP_LATERAL_SPEED * mix_scale,
                                  mixed[2] * TELEOP_YAW_SPEED * mix_scale)
                for sample, target in (
                    ((0.0, 1.0, 0.0), (0.0, TELEOP_LATERAL_SPEED, 0.0)),
                    ((0.0, -1.0, 0.0), (0.0, -TELEOP_LATERAL_SPEED, 0.0)),
                    ((0.0, 0.0, 1.0), (0.0, 0.0, TELEOP_YAW_SPEED)),
                    ((0.0, 0.0, -1.0), (0.0, 0.0, -TELEOP_YAW_SPEED)),
                    (mixed, combined_speed),
                ):
                    active, _ = hold_joystick(sample, target)
                    finish_joystick_stop(active)

                peak = (FORWARD_SPEED_LIMIT, 0.0, 0.0)
                current, _ = hold_joystick((1.0, 0.0, 0.0), peak)
                turn_sequence = joystick(0.0, -1.0, 1.0)
                # Deliver the new direction before advancing the fake executor past the cycle boundary.
                spin_for(fake, rclpy, 0.1)
                reply(current, ExecutionStatus.COMPLETED, len(current.segments))
                turning = get_plan(turn_sequence, after=current.plan_id)
                assert list(turning.initial_positions) == list(current.segments[-1].positions)
                check_plan(turning, config, 1, recovery=False)
                first_step = step_toward(peak, (0.0, -TELEOP_LATERAL_SPEED / 2.0, TELEOP_YAW_SPEED / 2.0))
                # Reachability checking may shorten the change, or use the proven deceleration cycle.
                choices = [tuple(before + factor * (after - before)
                                 for before, after in zip(peak, first_step, strict=True))
                           for factor in (1.0, 0.75, 0.5, 0.25)]
                choices.append(step_toward(peak, (0.0, 0.0, 0.0)))
                errors = [teleop_landing_error(turning, config, choice) for choice in choices]
                assert min(errors) <= 1.0, (choices, errors)
                accept(turning)
                stopped_on_transition = any(s.state == "stopping" and s.command_sequence == turn_sequence
                                            for s in statuses)
                finish_joystick_stop(turning, number=turn_sequence if stopped_on_transition else None)

                partial = (0.2, 0.0, 0.0)
                speed = (partial[0] * FORWARD_SPEED_LIMIT, 0.0, 0.0)
                current, _ = hold_joystick(partial, speed)
                finish_joystick_stop(current, number=joystick(0.0, 0.0, 0.0), reason="joystick centered")

                current, number = hold_joystick(partial, speed)
                finish_joystick_stop(current, number=number, reason="teleop command expired")
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=3.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
    finally:
        fake.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    executable, config = (Path(value).resolve(strict=True) for value in sys.argv[1:3])
    os.environ["ROS_DOMAIN_ID"] = str(20 + os.getpid() % 80)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ.pop("ROS_STATIC_PEERS", None)
    with tempfile.TemporaryFile(mode="w+t", encoding="utf-8", errors="replace") as log:
        try:
            exercise(executable, config, log)
        except Exception:
            log.seek(0)
            print(log.read(), file=sys.stderr)
            raise
