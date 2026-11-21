"""Exercise the planner on an isolated local ROS graph with a fake executor."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import uuid


def wait_for(node, rclpy, predicate, description, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError(f"Timed out waiting for {description}")
        rclpy.spin_once(node, timeout_sec=0.05)


def spin_for(node, rclpy, duration_s):
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)


def exercise(executable, hardware_config, child_log):
    import rclpy
    from rclpy.clock import Clock, ClockType
    from hexapod_interfaces.msg import (
        ExecutionStatus,
        MotionCommand,
        PlanStop,
        PlannerStatus,
        ServoPlan,
        StandingReference,
    )

    namespace = "/planner_smoke_" + uuid.uuid4().hex
    # 同一模型下 (0,+12,-72)° 的离线站姿样例，按物理舵机 ID 排列。
    standing = [500, 550, 200] * 3 + [500, 450, 800] * 3
    args = [
        str(executable), "--ros-args", "-r", "__ns:=" + namespace,
        "-p", "hardware_config:=" + json.dumps(str(hardware_config)),
        "-p", "standing_positions:=" + json.dumps(standing),
        "-p", "cycle_period_s:=2.4",
        "-p", "intervals_per_stage:=5",
        "-p", "linear_step_m_s:=0.0015",
        "-p", "time_scale:=1.0",
        "-p", "min_segment_ms:=1",
    ]

    rclpy.init(args=[])
    fake = rclpy.create_node("fake_executor", namespace=namespace)
    system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
    plans = []
    statuses = []
    stops = []
    plan_sub = fake.create_subscription(ServoPlan, "servo_plan", plans.append, 10)
    status_sub = fake.create_subscription(PlannerStatus, "planner_status", statuses.append, 30)
    stop_sub = fake.create_subscription(PlanStop, "plan_stop", stops.append, 10)
    command_pub = fake.create_publisher(MotionCommand, "motion_command", 10)
    reference_pub = fake.create_publisher(StandingReference, "standing_reference", 10)
    execution_pub = fake.create_publisher(ExecutionStatus, "execution_status", 10)
    try:
        with subprocess.Popen(args, stdout=child_log, stderr=subprocess.STDOUT) as process:
            try:
                def ready():
                    if process.poll() is not None:
                        raise AssertionError(f"Planner exited with status {process.returncode}")
                    return (
                        all(publisher.get_subscription_count() for publisher in
                            (command_pub, reference_pub, execution_pub))
                        and plan_sub.get_publisher_count()
                        and status_sub.get_publisher_count()
                        and stop_sub.get_publisher_count()
                    )

                wait_for(fake, rclpy, ready, "planner topic discovery", timeout_s=8.0)

                def walk(sequence):
                    command_pub.publish(MotionCommand(
                        kind=MotionCommand.WALK_CYCLES,
                        sequence=sequence,
                        stamp=system_clock.now().to_msg(),
                        vx_m_s=0.0015,
                        cycle_count=1,
                    ))

                walk(1)
                wait_for(fake, rclpy, lambda: any(s.command_sequence == 1 for s in statuses),
                         "diagnostic for missing standing reference")
                if plans:
                    raise AssertionError("Planner issued motion without a confirmed standing reference")

                reference_pub.publish(StandingReference(
                    stamp=system_clock.now().to_msg(), positions=standing,
                ))
                wait_for(fake, rclpy, lambda: any(s.state == "standing" for s in statuses),
                         "standing reference acceptance")

                walk(2)
                wait_for(fake, rclpy, lambda: bool(plans), "a finite walking plan", timeout_s=8.0)
                plan = plans[0]
                if list(plan.initial_positions) != standing or not plan.plan_id or not plan.segments:
                    raise AssertionError("Plan does not start at the confirmed standing reference")
                if sum(segment.walking_cycle_end for segment in plan.segments) != 1:
                    raise AssertionError("Exactly one complete walking cycle was requested")
                if plan.segments[-1].walking_cycle_end:
                    raise AssertionError("The fixture must finish with uncounted stance recovery")
                end_ms = 0
                for segment in plan.segments:
                    if segment.start_offset_ms != end_ms or segment.move_duration_ms <= 0:
                        raise AssertionError("Plan segment times are discontinuous")
                    if len(segment.positions) != 18 or any(p < 140 or p > 860 for p in segment.positions):
                        raise AssertionError("Plan segment violates the fixture's servo limits")
                    end_ms += segment.move_duration_ms

                def report(plan_id, state, completed_segments):
                    execution_pub.publish(ExecutionStatus(
                        plan_id=plan_id, state=state, completed_segments=completed_segments,
                    ))

                report(plan.plan_id, ExecutionStatus.ACCEPTED, 0)
                wait_for(fake, rclpy, lambda: any(s.plan_id == plan.plan_id and s.state == "accepted"
                                                 for s in statuses), "execution acknowledgement")
                planned_end_ns = (
                    plan.motion_start.sec * 1_000_000_000 + plan.motion_start.nanosec
                    + (end_ms + 20) * 1_000_000
                )
                wait_for(fake, rclpy, lambda: system_clock.now().nanoseconds >= planned_end_ns,
                         "the planned completion time", timeout_s=8.0)
                report(plan.plan_id + 1, ExecutionStatus.COMPLETED, len(plan.segments))
                spin_for(fake, rclpy, 0.3)

                def completed():
                    return any(s.command_sequence == 2 and s.state == "standing"
                               for s in statuses)

                if completed():
                    raise AssertionError("An unrelated execution status completed the current plan")
                report(plan.plan_id, ExecutionStatus.COMPLETED, len(plan.segments))
                wait_for(fake, rclpy, completed, "standing after matching completion")
                if len(plans) != 1:
                    raise AssertionError("Planner published an unexpected second finite plan")

                walk(3)
                wait_for(fake, rclpy, lambda: len(plans) == 2, "second finite walking plan")
                interrupted = plans[1]
                if len(interrupted.segments) < 2 or interrupted.segments[0].walking_cycle_end:
                    raise AssertionError("Test requires an interrupted partial walking cycle")
                report(interrupted.plan_id, ExecutionStatus.ACCEPTED, 0)
                command_pub.publish(MotionCommand(
                    kind=MotionCommand.STOP,
                    sequence=4,
                    stamp=system_clock.now().to_msg(),
                ))
                wait_for(fake, rclpy, lambda: any(s.plan_id == interrupted.plan_id for s in stops),
                         "a stop request for the active finite plan")
                report(interrupted.plan_id, ExecutionStatus.INTERRUPTED, 1)
                wait_for(fake, rclpy, lambda: any(s.plan_id == interrupted.plan_id and s.state == "fault"
                                                 for s in statuses), "fault at a partial cycle interruption")
                walk(5)
                wait_for(fake, rclpy, lambda: any(s.command_sequence == 5 and s.state == "rejected"
                                                 for s in statuses), "reference invalidated after fault")
                if len(plans) != 2:
                    raise AssertionError("Planner reused an invalidated standing reference")

                def teleop_rejection(sequence, x, y, yaw, expected_reason):
                    command_pub.publish(MotionCommand(
                        kind=MotionCommand.TELEOP,
                        sequence=sequence,
                        stamp=system_clock.now().to_msg(),
                        vx_m_s=x,
                        vy_m_s=y,
                        wz_rad_s=yaw,
                    ))
                    wait_for(fake, rclpy, lambda: any(s.command_sequence == sequence
                                                     and s.state == "rejected"
                                                     and s.reason == expected_reason
                                                     for s in statuses), expected_reason)

                missing_reference = "a confirmed standing reference is required"
                teleop_rejection(6, 1.0, 0.0, 1.0, missing_reference)
                teleop_rejection(7, 0.6, 0.8, -0.5, missing_reference)
                teleop_rejection(8, 1.0, 1.0, 0.0,
                                 "TELEOP requires normalized planar and yaw inputs")
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


def main():
    executable, hardware_config = (Path(value).resolve(strict=True) for value in sys.argv[1:])
    os.environ["ROS_DOMAIN_ID"] = str(20 + os.getpid() % 80)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ.pop("ROS_STATIC_PEERS", None)
    with tempfile.TemporaryFile(mode="w+t", encoding="utf-8", errors="replace") as child_log:
        try:
            exercise(executable, hardware_config, child_log)
        except Exception:
            child_log.seek(0)
            print(child_log.read(), file=sys.stderr)
            raise


if __name__ == "__main__":
    main()
