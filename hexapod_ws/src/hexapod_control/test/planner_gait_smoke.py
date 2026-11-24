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
LINEAR_STEP = 0.00542
RAMP_UP_SPEEDS = [step * LINEAR_STEP for step in range(1, 7)]
FORWARD_CYCLE_SPEEDS = [*RAMP_UP_SPEEDS, FORWARD_SPEED_LIMIT, *reversed(RAMP_UP_SPEEDS)]


def right_front_foot(config, values):
    servo, mounting, geometry = (config[key] for key in ("servo", "mounting", "model_geometry"))
    angles = []
    for index, position in enumerate(values[:3], start=1):
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
    transform = mounting["fixed_body_to_first_axis_transforms"]["right_front"]
    return tuple(origin + sum(coefficient * value for coefficient, value in zip(row, local, strict=True))
                 for origin, row in zip(transform["translation_mm"], transform["rotation"], strict=True))


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


def check_landing(plan, config, boundary, velocity):
    # At the cycle boundary the right-front leg lands at the nominal foot position in the projected
    # body pose half a stance ahead. Express that target in the body frame at the actual cycle boundary.
    x, y, _ = right_front_foot(config, STANDING)
    vx, vy, yaw_rate = velocity
    half_stance = WALK_PERIOD_S * WALK_DUTY / 2
    angle = yaw_rate * half_stance
    sine, cosine = math.sin(angle), math.cos(angle)
    along = sine / yaw_rate if yaw_rate else half_stance
    across = (1 - cosine) / yaw_rate if yaw_rate else 0
    expected = (cosine * x - sine * y + 1000 * (along * vx - across * vy),
                sine * x + cosine * y + 1000 * (across * vx + along * vy))
    measured = right_front_foot(config, plan.segments[boundary - 1].positions)[:2]
    assert math.dist(measured, expected) <= 1.0, (boundary, measured, expected)


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
    from rclpy.duration import Duration
    from rclpy.qos import DurabilityPolicy, QoSProfile

    config = json.loads(hardware_config.read_text(encoding="utf-8"))
    namespace = "/planner_gait_" + uuid.uuid4().hex
    args = [str(executable), "--ros-args", "-r", "__ns:=" + namespace,
            "-p", "hardware_config:=" + json.dumps(str(hardware_config)),
            "-p", "standing_positions:=" + json.dumps(STANDING), "-p", "plan_lead_ms:=1000",
            "-p", "command_timeout_ms:=3000"]
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

                confirmed = reference()
                teleop = get_plan(request(MotionCommand.TELEOP, vx_m_s=0.6, vy_m_s=0.8, wz_rad_s=-0.5))
                assert teleop.standing_stamp == confirmed and list(teleop.initial_positions) == STANDING
                check_plan(teleop, config, 1, recovery=False)

                reference()
                # Seed a still-fresh sample so the filter approaches its target quickly. The second
                # cycle distinguishes the joystick speed limit from the first cycle's step limit.
                spin_for(fake, rclpy, 0.6)
                stamp = (clock.now() - Duration(seconds=0.5)).to_msg()
                teleop_sequence = request(MotionCommand.TELEOP, stamp=stamp, vx_m_s=1.0)
                teleop_first = get_plan(teleop_sequence)
                complete(teleop_first)
                teleop_second = get_plan(teleop_sequence, after=teleop_first.plan_id)
                boundaries = check_plan(teleop_second, config, 1, recovery=False)
                check_forward_speeds(teleop_second, config, boundaries, [0.005])
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
