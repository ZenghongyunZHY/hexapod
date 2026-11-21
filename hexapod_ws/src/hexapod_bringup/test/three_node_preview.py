"""Run the production executables against the software controller in a local ROS graph."""

from contextlib import ExitStack
import json
from math import isclose, sqrt
import os
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryFile
import time
from uuid import uuid4

from ament_index_python.packages import get_package_prefix
from hexapod_interfaces.msg import ExecutionStatus, MotionCommand, PlannerStatus, PlanStop, ServoPlan
import rclpy
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger


STANDING = [500, 550, 200] * 3 + [500, 450, 800] * 3


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def end_ns(plan, segment):
    return stamp_ns(plan.motion_start) + (segment.start_offset_ms + segment.move_duration_ms) * 1_000_000


def stop_process(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


class Graph:
    def __init__(self, node, children):
        self.node = node
        self.children = children
        self.clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.commands, self.plans, self.stops, self.states, self.execution = [], [], [], [], []
        self.request = node.create_publisher(MotionCommand, "command_request", 10)
        self.joy = node.create_publisher(Joy, "joy", 10)
        self.confirm = node.create_client(Trigger, "confirm_standing")
        self.subs = [
            node.create_subscription(MotionCommand, "motion_command", self.commands.append, 30),
            node.create_subscription(ServoPlan, "servo_plan", self.plans.append, 10),
            node.create_subscription(PlanStop, "plan_stop", self.stops.append, 10),
            node.create_subscription(PlannerStatus, "planner_status", self.states.append, 30),
            node.create_subscription(ExecutionStatus, "execution_status", self.record_execution, 20),
        ]

    def record_execution(self, status):
        self.execution.append((status, self.clock.now().nanoseconds))

    def ready(self):
        namespace = self.node.get_namespace()
        return (self.request.get_subscription_count() and self.joy.get_subscription_count()
                and self.confirm.service_is_ready()
                and all(subscription.get_publisher_count() for subscription in self.subs)
                and all(self.node.count_subscribers(f"{namespace}/{topic}") >= 2
                        for topic in ("motion_command", "servo_plan", "execution_status", "plan_stop"))
                and self.node.count_subscribers(namespace + "/standing_reference") >= 1)

    def wait(self, predicate, description, timeout=8):
        deadline = time.monotonic() + timeout
        while True:
            for name, child in self.children.items():
                assert child.poll() is None, (name, child.returncode)
            assert not any(s.state == "fault" for s in self.states), self.states
            result = predicate()
            if result:
                return result
            assert time.monotonic() < deadline, (description, self.states, self.execution)
            rclpy.spin_once(self.node, timeout_sec=0.025)

    def command(self, kind, **fields):
        start = len(self.commands)
        self.request.publish(MotionCommand(kind=kind, stamp=self.clock.now().to_msg(), **fields))
        return self.wait(lambda: next((m for m in self.commands[start:] if m.kind == kind), None), "command forwarding")

    def state(self, name, sequence, plan_id=None):
        return self.wait(lambda: next((s for s in self.states if s.state == name and s.command_sequence == sequence
                                       and (plan_id is None or s.plan_id == plan_id)), None), name)

    def result(self, plan, state, timeout=8):
        return self.wait(lambda: next((item for item in self.execution
                                       if item[0].plan_id == plan.plan_id and item[0].state == state), None),
                         f"execution state {state}", timeout)

    def accepted_plan(self, sequence):
        planned = self.state("planned", sequence)
        plan = self.wait(lambda: next((p for p in self.plans if p.plan_id == planned.plan_id), None), "servo_plan")
        self.result(plan, ExecutionStatus.ACCEPTED)
        self.state("accepted", sequence, plan.plan_id)
        return plan


def exercise(graph):
    graph.wait(graph.ready, "node discovery", 12)
    command = graph.command(MotionCommand.WALK_CYCLES, vx_m_s=0.0015, cycle_count=1)
    assert "standing reference" in graph.state("rejected", command.sequence).reason
    assert not graph.plans

    for axes in ([0.0] * 4, [1.0, 1.0, 0.0, 0.0], [0.0] * 4):
        joy = Joy(axes=axes)
        joy.header.stamp = graph.clock.now().to_msg()
        graph.joy.publish(joy)
        sample = graph.wait(lambda: next((m for m in graph.commands if m.kind == MotionCommand.TELEOP
                                         and m.stamp == joy.header.stamp), None), "joystick sample")
        assert isclose(sample.vx_m_s, axes[0] / sqrt(2.0))
        assert isclose(sample.vy_m_s, axes[0] / sqrt(2.0))

    pending = graph.confirm.call_async(Trigger.Request())
    response = graph.wait(lambda: pending.result() if pending.done() else None, "standing confirmation")
    assert response.success, response.message
    graph.state("standing", 0, 0)

    command = graph.command(MotionCommand.WALK_CYCLES, vx_m_s=0.0015, cycle_count=1)
    plan = graph.accepted_plan(command.sequence)
    assert list(plan.initial_positions) == STANDING
    assert sum(s.walking_cycle_end for s in plan.segments) == 1
    completed, received_ns = graph.result(plan, ExecutionStatus.COMPLETED, 18)
    assert completed.completed_segments == len(plan.segments)
    assert received_ns >= end_ns(plan, plan.segments[-1])
    graph.state("standing", command.sequence, plan.plan_id)

    command = graph.command(MotionCommand.WALK_CYCLES, vx_m_s=0.0015, cycle_count=2)
    walking = graph.accepted_plan(command.sequence)
    assert walking.initial_positions == plan.segments[-1].positions
    graph.wait(lambda: graph.clock.now().nanoseconds >= stamp_ns(walking.motion_start) + 50_000_000, "first segment")
    stop = graph.command(MotionCommand.STOP)
    graph.wait(lambda: any(m.plan_id == walking.plan_id for m in graph.stops), "plan_stop")
    result, received_ns = graph.result(walking, ExecutionStatus.INTERRUPTED)
    boundary = next(i for i, s in enumerate(walking.segments) if s.walking_cycle_end)
    assert result.completed_segments == boundary + 1
    assert received_ns >= end_ns(walking, walking.segments[boundary])
    recovery = graph.accepted_plan(stop.sequence)
    assert recovery.initial_positions == walking.segments[boundary].positions
    graph.result(recovery, ExecutionStatus.COMPLETED, 18)
    graph.state("standing", stop.sequence, recovery.plan_id)


def main():
    config = str(Path(sys.argv[1]).resolve(strict=True))
    os.environ["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ.pop("ROS_STATIC_PEERS", None)
    namespace = "/three_node_preview_" + uuid4().hex
    rclpy.init(args=[])
    node = rclpy.create_node("three_node_observer", namespace=namespace)
    try:
        with ExitStack() as stack:
            children, logs = {}, {}
            for package, name, parameters in (
                ("hexapod_control", "hexapod_command", {}),
                ("hexapod_hardware", "hexapod_serial",
                 {"mode": "preview", "hardware_config": config, "preview_positions": STANDING}),
                ("hexapod_control", "hexapod_planner",
                 {"hardware_config": config, "standing_positions": STANDING, "cycle_period_s": 2.4,
                  "intervals_per_stage": 5, "linear_step_m_s": 0.0015, "time_scale": 2.0,
                  "min_segment_ms": 110, "plan_lead_ms": 600}),
            ):
                executable = Path(get_package_prefix(package), "lib", package, name)
                if os.name == "nt":
                    executable = executable.with_suffix(".exe")
                args = [str(executable.resolve(strict=True)), "--ros-args", "-r", "__ns:=" + namespace]
                for key, value in parameters.items():
                    args.extend(("-p", f"{key}:={json.dumps(value)}"))
                log = stack.enter_context(TemporaryFile(mode="w+t", encoding="utf-8", errors="replace"))
                logs[name] = log
                children[name] = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
                stack.callback(stop_process, children[name])
            try:
                exercise(Graph(node, children))
            except Exception:
                for name, log in logs.items():
                    log.seek(0)
                    print(f"{name}:\n{log.read()}", file=sys.stderr)
                raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
