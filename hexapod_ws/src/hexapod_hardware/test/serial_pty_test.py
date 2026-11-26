"""Verify the C++ controller with a Linux pseudoterminal and a fake controller board."""

import os
import pty
import select
import signal
import struct
import subprocess
import sys
import termios
import time


HEADER = b"\x55\x55"
SERVO_IDS = tuple(range(1, 19))
POSITION_COMMAND = 21
VOLTAGE_COMMAND = 15
MOVE_COMMAND = 3


def position_for(identifier):
    return 400 + 17 * identifier


def reply(master, command, payload):
    message = HEADER + bytes((len(payload) + 2, command)) + payload
    os.write(master, b"noise" + message[:3])
    time.sleep(0.002)
    os.write(master, message[3:])


def positions_reply(master, ids):
    reply(master, POSITION_COMMAND, bytes((len(ids),)) + b"".join(struct.pack("<BH", i, position_for(i)) for i in ids))


def packets(buffer):
    while len(buffer) >= 4:
        if buffer[:2] != HEADER:
            raise AssertionError(f"unexpected controller request header {buffer[:2]!r}")
        end = buffer[2] + 2
        if len(buffer) < end:
            return
        command, payload = buffer[3], bytes(buffer[4:end])
        del buffer[:end]
        yield command, payload


def run_case(executable, mode):
    master, slave = pty.openpty()
    process = subprocess.Popen((executable, mode, os.ttyname(slave)))
    buffer = bytearray()
    requests = []
    moves = []
    deadline = time.monotonic() + 5.0
    try:
        while time.monotonic() < deadline and master is not None:
            readable, _, _ = select.select((master,), (), (), 0.02)
            if not readable:
                if process.poll() is not None:
                    break
                continue
            buffer.extend(os.read(master, 256))
            for command, payload in packets(buffer):
                if command == VOLTAGE_COMMAND:
                    assert mode == "normal" and not payload
                    attributes = termios.tcgetattr(slave)
                    assert attributes[4:6] == [termios.B9600, termios.B9600]
                    assert (attributes[2] & termios.CSIZE) == termios.CS8
                    assert not attributes[2] & (termios.PARENB | termios.CSTOPB)
                    reply(master, command, struct.pack("<H", 7400))
                elif command == POSITION_COMMAND:
                    ids = tuple(payload[1:])
                    assert payload[0] == len(ids)
                    requests.append(ids)
                    if mode == "disconnect":
                        os.close(master)
                        master = None
                        break
                    if mode == "duplicate":
                        positions_reply(master, (1, 1))
                    elif len(requests) == 1:
                        if mode == "timeout":
                            time.sleep(0.18)
                        positions_reply(master, tuple(i for i in ids if i != 7))
                    elif mode == "normal":
                        positions_reply(master, ids)
                elif command == MOVE_COMMAND:
                    assert mode == "normal", command
                    moves.append(payload)
                else:
                    raise AssertionError(f"unexpected controller command {command}")
        if process.wait(timeout=1) != 0:
            raise AssertionError(f"C++ UART test failed in {mode} mode")
        if mode in ("normal", "timeout"):
            assert requests == [SERVO_IDS, (7,)], requests
        else:
            assert requests == [SERVO_IDS], requests
        if mode == "normal":
            assert len(moves) == 1, moves
            count, duration = struct.unpack("<BH", moves[0][:3])
            targets = tuple(struct.iter_unpack("<BH", moves[0][3:]))
            assert (count, duration) == (18, 120)
            expected = tuple((i, position_for(i)) for i in SERVO_IDS)
            assert targets == expected
        else:
            assert not moves
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        os.close(slave)
        if master is not None:
            os.close(master)


def run_transition(executable, hardware_config, mode):
    master, slave = pty.openpty()
    with subprocess.Popen((executable, os.ttyname(slave), hardware_config, mode)) as child:
        try:
            buffer = bytearray()
            moves, read_times = [], []
            voltage_queries = 0
            disconnect_at = None
            deadline = time.monotonic() + 8
            while child.poll() is None and time.monotonic() < deadline:
                if disconnect_at is not None and time.monotonic() >= disconnect_at:
                    os.close(master)
                    master = None
                    break
                if not select.select((master,), (), (), 0.01)[0]:
                    continue
                buffer.extend(os.read(master, 4096))
                for command, payload in packets(buffer):
                    if command == VOLTAGE_COMMAND:
                        voltage_queries += 1
                        reply(master, command, struct.pack("<H", 7400))
                    elif command == POSITION_COMMAND:
                        assert tuple(payload) == (18, *SERVO_IDS)
                        read_times.append(time.monotonic())
                        measured = [550] * 18
                        measured[0] += 30 if mode == "failure" and len(read_times) == 1 else 1
                        reply(master, command, b"\x12" + b"".join(struct.pack("<BH", i, value)
                                                                 for i, value in zip(SERVO_IDS, measured)))
                    elif command == MOVE_COMMAND:
                        count, duration = struct.unpack("<BH", payload[:3])
                        targets = list(struct.iter_unpack("<BH", payload[3:]))
                        assert count == 18 and tuple(i for i, _ in targets) == SERVO_IDS
                        values = [value for _, value in targets]
                        assert values == [values[0]] * 18
                        moves.append((time.monotonic(), duration, values[0]))
                        if mode == "preempt" and values[0] == 575:
                            disconnect_at = time.monotonic() + 0.5
                        if values[0] == 580:
                            if mode == "write_timeout":
                                termios.tcflow(slave, termios.TCOOFF)
                            elif mode == "late_wake":
                                child.send_signal(signal.SIGSTOP)
                                time.sleep(0.45)
                                child.send_signal(signal.SIGCONT)
                    else:
                        raise AssertionError(command)
            assert child.wait(timeout=1) == 0
            if mode == "probe":
                assert voltage_queries >= 1 and not read_times and not moves
                return
            if mode in ("write_timeout", "late_wake"):
                if select.select((master,), (), (), 0.05)[0]:
                    buffer.extend(os.read(master, 4096))
                assert len(read_times) == 1 and not buffer, (read_times, buffer)
                assert [(duration, position) for _, duration, position in moves] == [(280, 550), (240, 580)], moves
                return
            assert len(read_times) == 2
            if mode == "preempt":
                expected = [(280, 550), (500, 560), (280, 550), (160, 575), (160, 576)]
                assert moves[2][0] - moves[1][0] >= 0.5, moves
                transition_indices = (0, 2)
            else:
                expected = [(280, 550), (280, 550), (160, 575), (160, 576), (160, 577)]
                transition_indices = (0, 1)
            assert [(duration, position) for _, duration, position in moves] == expected, moves
            # Packet capture uses the fake board's independent scheduler clock.
            assert moves[-1][0] - moves[-2][0] >= 0.150, moves
            for read_at, move_index in zip(read_times, transition_indices):
                assert read_at - moves[move_index][0] >= 0.28, (read_times, moves)
        finally:
            if child.poll() is None:
                child.kill()
                child.wait()
            if master is not None:
                os.close(master)
            os.close(slave)


if __name__ == "__main__":
    for case in ("normal", "timeout", "duplicate", "disconnect"):
        run_case(sys.argv[1], case)
        print(f"virtual UART {case} passed", flush=True)
    if len(sys.argv) == 4:
        for mode in ("preempt", "failure", "write_timeout", "late_wake", "probe"):
            run_transition(sys.argv[2], sys.argv[3], mode)
            print(f"virtual UART runtime {mode} passed", flush=True)
