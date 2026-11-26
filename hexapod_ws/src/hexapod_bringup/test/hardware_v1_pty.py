"""Offline V1 integration with all three ROS nodes and a private virtual UART."""

from contextlib import ExitStack
import json
import os
from pathlib import Path
import select
import struct
import subprocess
import sys
from tempfile import TemporaryFile
import time
from uuid import uuid4

from ament_index_python.packages import get_package_prefix
from hexapod_interfaces.msg import ExecutionStatus, MotionCommand, PlannerStatus, ServoPlan, StandingReference
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger


STANDING = [500, 540, 210] * 3 + [500, 460, 790] * 3
STANDING_MS = 2000
SERVO_IDS = tuple(range(1, 19))
HEADER = b"\x55\x55"


class VirtualBoard:
    def __init__(self, stack):
        self.master, slave = os.openpty()
        stack.callback(os.close, self.master)
        stack.callback(os.close, slave)
        self.device = os.ttyname(slave)
        self.buffer = bytearray()
        self.positions = STANDING.copy()
        self.yaw_offset = 40
        self.moves = []
        self.readbacks = []

    def respond(self, opcode, payload):
        os.write(self.master, HEADER + bytes((len(payload) + 2, opcode)) + payload)

    def poll(self):
        if select.select((self.master,), (), (), 0)[0]:
            self.buffer.extend(os.read(self.master, 4096))
        while len(self.buffer) >= 4:
            assert self.buffer[:2] == HEADER, self.buffer[:2]
            size = self.buffer[2] + 2
            assert size >= 4
            if len(self.buffer) < size:
                break
            opcode, payload = self.buffer[3], bytes(self.buffer[4:size])
            del self.buffer[:size]
            if opcode == 15:
                assert not payload
                self.respond(opcode, struct.pack("<H", 7400))
            elif opcode == 21:
                assert tuple(payload) == (18, *SERVO_IDS)
                measured = [self.positions[0] + self.yaw_offset, *self.positions[1:]]
                self.readbacks.append((time.monotonic(), measured))
                reply = b"\x12" + b"".join(struct.pack("<BH", i, value) for i, value in zip(SERVO_IDS, measured))
                self.respond(opcode, reply)
            elif opcode == 3:
                count, duration = struct.unpack("<BH", payload[:3])
                joints = list(struct.iter_unpack("<BH", payload[3:]))
                assert count == 18 and tuple(joint for joint, _ in joints) == SERVO_IDS
                self.positions = [position for _, position in joints]
                self.moves.append((time.monotonic(), duration, self.positions))
            else:
                raise AssertionError(f"Unexpected UART request: {opcode}")


def stop_process(child):
    if child.poll() is None:
        child.terminate()
        try:
            child.wait(timeout=3)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait()


def stamp_key(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class Graph:
    def __init__(self, node, board, children):
        self.node = node
        self.board = board
        self.children = children
        self.clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.references, self.statuses, self.commands, self.plans = [], [], [], []
        self.executions = []
        self.requests = node.create_publisher(MotionCommand, "command_request", 10)
        self.late_plans = node.create_publisher(ServoPlan, "servo_plan", 10)
        self.transition = node.create_client(Trigger, "transition_to_standing")
        node.create_subscription(StandingReference, "standing_reference", self.references.append,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(PlannerStatus, "planner_status", self.statuses.append, 10)
        node.create_subscription(MotionCommand, "motion_command", self.commands.append, 10)
        node.create_subscription(ServoPlan, "servo_plan", self.plans.append, 10)
        node.create_subscription(ExecutionStatus, "execution_status", self.executions.append, 10)

    def poll(self):
        for name, child in self.children.items():
            if child.poll() is not None:
                raise RuntimeError(f"{name} exited: {child.returncode}")
        self.board.poll()
        rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait(self, predicate, description, seconds=12):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.poll()
            if value := predicate():
                return value
        raise AssertionError(f"Timed out waiting for {description}")

    def settle(self, seconds=0.2):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.poll()

    def command(self, kind, stamp=None):
        self.requests.publish(MotionCommand(
            kind=kind, stamp=self.clock.now().to_msg() if stamp is None else stamp,
            vx_m_s=0.0015 if kind == MotionCommand.WALK_CYCLES else 0.0,
            cycle_count=2 if kind == MotionCommand.WALK_CYCLES else 0,
        ))

    def reference(self, ready, after=0):
        return next((item for item in reversed(self.references)
                     if item.ready == ready and stamp_key(item.stamp) > after), None)

    def ready(self, after=0):
        reference = self.reference(True, after)
        if reference is None:
            return None
        return reference if any(status.state == "standing" and status.standing_stamp == reference.stamp
                                for status in self.statuses) else None

    def reset(self):
        return self.transition.call_async(Trigger.Request())

    def execution(self, plan_id, state):
        return next((item for item in self.executions if item.plan_id == plan_id and item.state == state), None)


def exercise_stop(graph, plan):
    board = graph.board
    first_move = len(board.moves) - 1
    cycle_ends = [i + 1 for i, segment in enumerate(plan.segments) if segment.walking_cycle_end]
    assert len(cycle_ends) == 2
    prefix_length = cycle_ends[0]
    assert prefix_length > 1

    graph.command(MotionCommand.STOP)
    interrupted = graph.wait(lambda: graph.execution(plan.plan_id, ExecutionStatus.INTERRUPTED),
                             "execution stopped at the first complete walking cycle")
    assert interrupted.completed_segments == prefix_length
    following_move = first_move + prefix_length
    assert [move[1:] for move in board.moves[first_move:following_move]] == [
        (segment.move_duration_ms, list(segment.positions)) for segment in plan.segments[:prefix_length]
    ]
    assert graph.execution(plan.plan_id, ExecutionStatus.COMPLETED) is None

    recovery = graph.wait(lambda: next((item for item in graph.plans
                                        if item.plan_id > plan.plan_id and item.standing_stamp == plan.standing_stamp),
                                       None), "recovery from the stopped cycle boundary")
    last_time, last_duration, last_positions = board.moves[following_move - 1]
    assert list(recovery.initial_positions) == last_positions
    assert not any(segment.walking_cycle_end for segment in recovery.segments)
    graph.wait(lambda: len(board.moves) > following_move, "first recovery frame on UART")
    recovery_time, recovery_duration, recovery_positions = board.moves[following_move]
    first_recovery_segment = recovery.segments[0]
    assert (recovery_duration, recovery_positions) == (
        first_recovery_segment.move_duration_ms, list(first_recovery_segment.positions)
    )
    # The virtual board timestamps complete frames when polled, after the serial write.
    assert recovery_time - last_time >= last_duration / 1000 - 0.03

    final_segment = recovery.segments[-1]
    recovery_seconds = (final_segment.start_offset_ms + final_segment.move_duration_ms) / 1000
    completed = graph.wait(lambda: graph.execution(recovery.plan_id, ExecutionStatus.COMPLETED),
                           "execution completed the recovery", seconds=recovery_seconds + 4)
    assert completed.completed_segments == len(recovery.segments)
    graph.wait(lambda: any(status.plan_id == recovery.plan_id and status.state == "standing"
                           and status.standing_stamp == plan.standing_stamp for status in graph.statuses),
               "planner returned to standing after recovery")


def exercise(graph):
    board = graph.board
    graph.wait(lambda: graph.transition.service_is_ready() and graph.requests.get_subscription_count()
               and graph.late_plans.get_subscription_count()
               and graph.node.count_publishers("standing_reference")
               and graph.node.count_publishers("execution_status"), "three-node discovery")
    assert graph.node.resolve_service_name("confirm_standing") not in dict(graph.node.get_service_names_and_types())
    graph.command(MotionCommand.WALK_CYCLES)
    graph.settle()
    assert not graph.commands and not board.moves

    failed = graph.reset()
    graph.wait(lambda: len(board.moves) == 1, "first single standing frame")
    assert board.moves[-1][1:] == (STANDING_MS, STANDING)
    duplicate = graph.reset()
    graph.command(MotionCommand.TELEOP)
    graph.command(MotionCommand.WALK_CYCLES)
    graph.command(MotionCommand.STOP)
    result = graph.wait(lambda: duplicate.result() if duplicate.done() else None, "discarded duplicate transition")
    assert not result.success
    result = graph.wait(lambda: failed.result() if failed.done() else None, "failed FK confirmation")
    assert not result.success and not graph.reference(True)
    assert len(board.moves) == len(board.readbacks) == 1
    assert board.readbacks[0][0] - board.moves[0][0] >= STANDING_MS / 1000 - 0.03
    graph.command(MotionCommand.WALK_CYCLES)
    graph.settle()
    assert not graph.commands and len(board.moves) == 1

    board.yaw_offset = 1
    recovered = graph.reset()
    result = graph.wait(lambda: recovered.result() if recovered.done() else None, "retried standing transition")
    assert result.success, result.message
    ready = graph.wait(graph.ready, "serial and planner ready")
    graph.settle(0.1)
    assert len(board.moves) == 2 and board.moves[-1][1:] == (STANDING_MS, STANDING)
    assert board.readbacks[-1][1] != STANDING

    graph.command(MotionCommand.WALK_CYCLES)
    old_plan = graph.wait(lambda: next((item for item in graph.plans if item.standing_stamp == ready.stamp), None),
                          "walking plan from nominal standing")
    assert list(old_plan.initial_positions) == STANDING
    graph.wait(lambda: len(board.moves) == 3, "first walking frame")
    previous_epoch = stamp_key(ready.stamp)
    preempted = graph.reset()
    graph.wait(lambda: graph.reference(False, previous_epoch), "old readiness revoked")
    for kind in (MotionCommand.TELEOP, MotionCommand.WALK_CYCLES, MotionCommand.STOP):
        graph.command(kind)
    result = graph.wait(lambda: preempted.result() if preempted.done() else None, "standing after preemption")
    assert result.success, result.message
    new_ready = graph.wait(lambda: graph.ready(previous_epoch), "new nominal reference")
    assert len(board.moves) == 4 and board.moves[-1][1:] == (STANDING_MS, STANDING), board.moves
    assert len(graph.commands) == 1 and graph.commands[0].kind == MotionCommand.WALK_CYCLES

    graph.command(MotionCommand.WALK_CYCLES, ready.stamp)
    old_plan.motion_start = (graph.clock.now() + Duration(seconds=0.8)).to_msg()
    graph.late_plans.publish(old_plan)
    graph.settle(1.2)
    assert len(graph.commands) == 1 and len(board.moves) == 4

    graph.command(MotionCommand.WALK_CYCLES)
    new_plan = graph.wait(lambda: next((item for item in graph.plans if item.standing_stamp == new_ready.stamp), None),
                          "new command sourced from the replacement epoch")
    assert list(new_plan.initial_positions) == STANDING
    graph.wait(lambda: len(board.moves) == 5, "new walking frame on UART")
    print("V1: one-frame standing, tolerance reject/retry, active walking preemption, dropped input, and nominal new epoch passed")
    exercise_stop(graph, new_plan)
    print("V1: STOP completed one walking cycle, then recovered to standing after the last walking action")


def main():
    config = str(Path(sys.argv[1]).resolve(strict=True))
    os.environ["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ.pop("ROS_STATIC_PEERS", None)
    namespace = "/hexapod_v1_" + uuid4().hex
    rclpy.init(args=[])
    observer = rclpy.create_node("v1_observer", namespace=namespace)
    try:
        with ExitStack() as stack:
            board = VirtualBoard(stack)
            children, logs = {}, {}
            for package, name, parameters in (
                ("hexapod_control", "hexapod_command", {}),
                ("hexapod_control", "hexapod_planner",
                 {"hardware_config": config, "standing_positions": STANDING, "cycle_period_s": 4.8,
                  "stance_ratio": 0.625, "double_support_intervals": 1, "swing_intervals": 5,
                  "lift_height_m": 0.003, "outward_offset_m": 0.0, "recovery_lift_height_m": 0.003,
                  "plan_lead_ms": 650, "linear_step_m_s": 0.0015}),
                ("hexapod_hardware", "hexapod_serial",
                 {"hardware_config": config, "standing_positions": STANDING,
                  "serial_port": board.device}),
            ):
                executable = Path(get_package_prefix(package), "lib", package, name).resolve(strict=True)
                args = [str(executable), "--ros-args", "-r", "__ns:=" + namespace]
                for key, value in parameters.items():
                    args.extend(("-p", f"{key}:={json.dumps(value)}"))
                log = stack.enter_context(TemporaryFile(mode="w+t", encoding="utf-8", errors="replace"))
                logs[name] = log
                children[name] = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
                stack.callback(stop_process, children[name])
            try:
                exercise(Graph(observer, board, children))
            except Exception:
                for name, log in logs.items():
                    log.seek(0)
                    print(f"{name}:\n{log.read()}", file=sys.stderr)
                raise
    finally:
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
