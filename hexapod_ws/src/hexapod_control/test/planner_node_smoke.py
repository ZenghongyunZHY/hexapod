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
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
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
        "-p", "stance_ratio:=0.625",
        "-p", "double_support_intervals:=1",
        "-p", "swing_intervals:=5",
        "-p", "lift_height_m:=0.003",
        "-p", "outward_offset_m:=0.0",
        "-p", "recovery_lift_height_m:=0.003",
        "-p", "linear_step_m_s:=0.0015",
        "-p", "min_segment_ms:=1",
        "-p", "command_timeout_ms:=2000",
        "-p", "feedback_grace_ms:=100",
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
    reference_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
    reference_pub = fake.create_publisher(StandingReference, "standing_reference", reference_qos)
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

                def walk(sequence, stamp=None):
                    command_pub.publish(MotionCommand(
                        kind=MotionCommand.WALK_CYCLES,
                        sequence=sequence,
                        stamp=stamp if stamp is not None else system_clock.now().to_msg(),
                        vx_m_s=0.0015,
                        cycle_count=1,
                    ))

                def stop(sequence):
                    command_pub.publish(MotionCommand(kind=MotionCommand.STOP, sequence=sequence,
                                                       stamp=system_clock.now().to_msg()))

                def teleop(sequence, x=0.6, y=0.8, yaw=-0.5):
                    command_pub.publish(MotionCommand(kind=MotionCommand.TELEOP, sequence=sequence,
                                                       stamp=system_clock.now().to_msg(),
                                                       vx_m_s=x, vy_m_s=y, wz_rad_s=yaw))

                def publish_reference(ready):
                    stamp = system_clock.now().to_msg()
                    reference_pub.publish(StandingReference(stamp=stamp, ready=ready))
                    expected_state = "standing" if ready else "waiting_for_reference"
                    wait_for(fake, rclpy,
                             lambda: any(s.command_sequence == 0 and s.state == expected_state
                                         and s.standing_stamp == stamp for s in statuses),
                             f"standing reference {expected_state}")
                    return stamp

                def assert_ignored(commands, expected_plans, expected_stops=None):
                    spin_for(fake, rclpy, 0.2)
                    assert len(plans) == expected_plans
                    if expected_stops is not None:
                        assert len(stops) == expected_stops
                    assert not any(s.command_sequence in commands for s in statuses), "A discarded command was processed"

                walk(1)
                assert_ignored({1}, 0, 0)
                first_ready = publish_reference(True)
                walk(2)
                wait_for(fake, rclpy, lambda: bool(plans), "a finite walking plan", timeout_s=8.0)
                plan = plans[0]
                if list(plan.initial_positions) != standing or plan.standing_stamp != first_ready or not plan.plan_id or not plan.segments:
                    raise AssertionError("Plan does not start at the nominal position in the confirmed epoch")
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

                def report(plan_id, state, completed_segments, maximum_finish_delay_ms=0):
                    execution_pub.publish(ExecutionStatus(
                        plan_id=plan_id, state=state, completed_segments=completed_segments,
                        maximum_finish_delay_ms=maximum_finish_delay_ms,
                    ))

                report(plan.plan_id, ExecutionStatus.ACCEPTED, 0, maximum_finish_delay_ms=1200)
                wait_for(fake, rclpy, lambda: any(s.plan_id == plan.plan_id and s.state == "accepted"
                                                 for s in statuses), "execution acknowledgement")
                def acknowledgements():
                    return [s for s in statuses if s.plan_id == plan.plan_id and s.state == "accepted"]

                assert acknowledgements()[0].completion_timeout_ms > end_ms + 900
                # A duplicate must not publish a new timeout or replace the accepted budget.
                report(plan.plan_id, ExecutionStatus.ACCEPTED, 0, maximum_finish_delay_ms=0)
                planned_end_ns = (
                    plan.motion_start.sec * 1_000_000_000 + plan.motion_start.nanosec
                    + (end_ms + 20) * 1_000_000
                )
                wait_for(fake, rclpy, lambda: system_clock.now().nanoseconds >= planned_end_ns,
                         "the planned completion time", timeout_s=end_ms / 1000 + 4)
                report(plan.plan_id + 1, ExecutionStatus.COMPLETED, len(plan.segments))
                spin_for(fake, rclpy, 0.3)
                assert len(acknowledgements()) == 1
                assert not any(s.plan_id == plan.plan_id and s.state == "fault" for s in statuses)

                def completed():
                    return any(s.command_sequence == 2 and s.state == "standing" and s.standing_stamp == first_ready
                               for s in statuses)

                if completed():
                    raise AssertionError("An unrelated execution status completed the current plan")
                report(plan.plan_id, ExecutionStatus.COMPLETED, len(plan.segments))
                wait_for(fake, rclpy, completed, "standing after matching completion")
                if len(plans) != 1:
                    raise AssertionError("Planner published an unexpected second finite plan")

                walk(3)
                wait_for(fake, rclpy, lambda: len(plans) == 2, "second finite walking plan")
                preempted = plans[1]
                report(preempted.plan_id, ExecutionStatus.ACCEPTED, 0)
                wait_for(fake, rclpy, lambda: any(s.plan_id == preempted.plan_id and s.state == "accepted"
                                                 for s in statuses), "acknowledgement before preemption")
                publish_reference(False)
                stop(4)
                walk(5)
                teleop(6)
                report(preempted.plan_id, ExecutionStatus.INTERRUPTED, 1)
                assert_ignored({4, 5, 6}, 2, 0)

                failed_stamp = publish_reference(False)
                reference_pub.publish(StandingReference(stamp=first_ready, ready=True))
                walk(7)
                assert_ignored({7}, 2, 0)

                second_ready = publish_reference(True)
                reference_pub.publish(StandingReference(stamp=failed_stamp, ready=False))
                walk(8, second_ready)
                assert_ignored({8}, 2, 0)
                walk(9)
                wait_for(fake, rclpy, lambda: len(plans) == 3, "fresh command after transition")
                interrupted = plans[2]
                assert list(interrupted.initial_positions) == standing and interrupted.standing_stamp == second_ready
                if len(interrupted.segments) < 2 or interrupted.segments[0].walking_cycle_end:
                    raise AssertionError("Test requires an interrupted partial walking cycle")
                report(interrupted.plan_id, ExecutionStatus.ACCEPTED, 0)
                stop(10)
                wait_for(fake, rclpy, lambda: any(s.plan_id == interrupted.plan_id for s in stops),
                         "a stop request for the active finite plan")
                report(interrupted.plan_id, ExecutionStatus.INTERRUPTED, 1)
                wait_for(fake, rclpy, lambda: any(s.plan_id == interrupted.plan_id and s.state == "fault"
                                                 for s in statuses), "fault at a partial cycle interruption")
                walk(11)
                assert_ignored({11}, 3)
                final_ready = publish_reference(True)
                teleop(12, 1.0, 1.0, 0.0)
                wait_for(fake, rclpy, lambda: any(s.command_sequence == 12 and s.state == "rejected"
                                                 and "normalized" in s.reason for s in statuses),
                         "invalid normalized TELEOP rejection")

                walk(13)
                wait_for(fake, rclpy, lambda: len(plans) == 4, "early authoritative completion fixture")
                completed_early = plans[3]
                report(completed_early.plan_id, ExecutionStatus.ACCEPTED, 0)
                wait_for(fake, rclpy, lambda: any(s.plan_id == completed_early.plan_id and s.state == "accepted"
                                                 for s in statuses), "acknowledgement before early completion")
                report(completed_early.plan_id, ExecutionStatus.COMPLETED, len(completed_early.segments))
                wait_for(fake, rclpy, lambda: any(s.plan_id == completed_early.plan_id and s.state == "standing"
                                                 and s.standing_stamp == final_ready for s in statuses),
                         "authoritative completion before the nominal clock boundary")

                walk(14)
                wait_for(fake, rclpy, lambda: len(plans) == 5, "clock range fixture")
                outside_clock = plans[4]
                report(outside_clock.plan_id, ExecutionStatus.ACCEPTED, 0, maximum_finish_delay_ms=2**64 - 1)
                wait_for(fake, rclpy, lambda: any(s.plan_id == outside_clock.plan_id and s.state == "fault"
                                                 and "clock range" in s.reason for s in statuses),
                         "unrepresentable completion delay rejection")
                assert all(s.completion_timeout_ms == 0 for s in statuses if s.state != "accepted")
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
