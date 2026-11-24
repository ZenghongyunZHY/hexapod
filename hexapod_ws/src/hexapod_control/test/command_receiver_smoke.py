"""Exercise the command receiver's standing handoff and joystick forwarding."""

from math import isclose, sqrt
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import uuid

from planner_node_smoke import spin_for, wait_for


def exercise(executable, child_log):
    import rclpy
    from builtin_interfaces.msg import Time
    from hexapod_interfaces.msg import MotionCommand, PlannerStatus, StandingReference
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Joy

    namespace = "/command_smoke_" + uuid.uuid4().hex
    args = [str(executable), "--ros-args", "-r", "__ns:=" + namespace,
            "-p", "input_timeout_ms:=2000", "-p", "stop_button:=0"]
    rclpy.init(args=[])
    node = rclpy.create_node("test_commands", namespace=namespace)
    system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
    ref_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    reference_pub = node.create_publisher(StandingReference, "standing_reference", ref_qos)
    request_pub = node.create_publisher(MotionCommand, "command_request", 10)
    status_pub = node.create_publisher(PlannerStatus, "planner_status", 10)
    joy_pub = node.create_publisher(Joy, "joy", 10)
    outgoing = []
    motion_sub = node.create_subscription(MotionCommand, "motion_command", outgoing.append, 10)

    def command(kind, stamp=None):
        request_pub.publish(MotionCommand(kind=kind, cycle_count=int(kind == MotionCommand.WALK_CYCLES),
                                          vx_m_s=0.001, stamp=system_clock.now().to_msg() if stamp is None else stamp))

    def press_stop(stamp=None):
        joy = Joy(axes=[0.0, 0.0, 0.0], buttons=[1])
        joy.header.stamp = system_clock.now().to_msg() if stamp is None else stamp
        joy_pub.publish(joy)

    def reference(ready, stamp=None):
        event_stamp = system_clock.now().to_msg() if stamp is None else stamp
        reference_pub.publish(StandingReference(stamp=event_stamp, ready=ready))
        spin_for(node, rclpy, 0.15)
        return event_stamp

    def assert_dropped(expected_count):
        spin_for(node, rclpy, 0.2)
        assert len(outgoing) == expected_count, "A disabled or old command reached motion_command"

    try:
        with subprocess.Popen(args, stdout=child_log, stderr=subprocess.STDOUT) as process:
            try:
                wait_for(node, rclpy,
                         lambda: all(pub.get_subscription_count() for pub in
                                     (reference_pub, request_pub, status_pub, joy_pub)) and motion_sub.get_publisher_count(),
                         "command receiver topic discovery", timeout_s=8.0)

                kinds = (MotionCommand.TELEOP, MotionCommand.WALK_CYCLES, MotionCommand.STOP)
                for kind in kinds:
                    command(kind)
                press_stop()
                assert_dropped(0)

                first_ready = reference(True)
                command(MotionCommand.TELEOP)
                wait_for(node, rclpy, lambda: len(outgoing) == 1, "first fresh TELEOP")
                assert outgoing[-1].kind == MotionCommand.TELEOP and outgoing[-1].vx_m_s == 0.001

                reference(False)
                during_transition = system_clock.now().to_msg()
                for kind in kinds:
                    command(kind)
                press_stop()
                assert_dropped(1)

                failed = reference(False)
                reference(True, first_ready)
                command(MotionCommand.WALK_CYCLES)
                assert_dropped(1)

                second_ready = reference(True)
                reference(False, failed)
                status_pub.publish(PlannerStatus(state="fault", standing_stamp=first_ready))
                for kind in kinds:
                    command(kind, during_transition)
                    command(kind, second_ready)
                    command(kind, Time())
                press_stop(during_transition)
                assert_dropped(1)

                command(MotionCommand.TELEOP)
                wait_for(node, rclpy, lambda: len(outgoing) == 2, "fresh TELEOP in the new epoch")
                command(MotionCommand.WALK_CYCLES)
                wait_for(node, rclpy, lambda: len(outgoing) == 3, "fresh route in the new epoch")
                press_stop()
                wait_for(node, rclpy, lambda: len(outgoing) == 4, "daily joystick STOP")
                assert [msg.kind for msg in outgoing] == [MotionCommand.TELEOP, *kinds]

                center = (0.0, 0.0, 0.0)
                diagonal = (1.0, 1.0, -0.5)
                component = 1.0 / sqrt(2.0)
                samples = ((center, center), (diagonal, (component, component, -0.5)), (center, center))
                for count, (axes, expected) in enumerate(samples, start=5):
                    joy = Joy(axes=list(axes), buttons=[0])
                    joy.header.stamp = system_clock.now().to_msg()
                    joy_pub.publish(joy)
                    wait_for(node, rclpy, lambda: len(outgoing) == count, "joystick axis forwarding")
                    sample = outgoing[-1]
                    assert sample.kind == MotionCommand.TELEOP and sample.stamp == joy.header.stamp
                    actual = (sample.vx_m_s, sample.vy_m_s, sample.wz_rad_s)
                    assert all(isclose(value, target) for value, target in zip(actual, expected)), (actual, expected)

                assert all(a.sequence < b.sequence for a, b in zip(outgoing, outgoing[1:]))
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=3.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    executable = Path(sys.argv[1]).resolve(strict=True)
    os.environ["ROS_DOMAIN_ID"] = str(20 + os.getpid() % 80)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ.pop("ROS_STATIC_PEERS", None)
    with tempfile.TemporaryFile(mode="w+t", encoding="utf-8", errors="replace") as child_log:
        try:
            exercise(executable, child_log)
        except Exception:
            child_log.seek(0)
            print(child_log.read(), file=sys.stderr)
            raise
