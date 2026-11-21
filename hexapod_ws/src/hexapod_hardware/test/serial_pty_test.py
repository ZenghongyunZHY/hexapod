"""Verify the C++ controller with a Linux pseudoterminal and a fake controller board."""

import os
import pty
import select
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


def run_case(executable, mode, hardware_config=None):
    master, slave = pty.openpty()
    arguments = (os.ttyname(slave), hardware_config) if mode == "runtime" else (mode, os.ttyname(slave))
    process = subprocess.Popen((executable, *arguments))
    buffer = bytearray()
    requests = []
    moves = []
    deadline = time.monotonic() + 5.0
    disconnect_at = None
    try:
        while time.monotonic() < deadline and master is not None:
            if disconnect_at is not None and time.monotonic() >= disconnect_at:
                os.close(master)
                master = None
                break
            readable, _, _ = select.select((master,), (), (), 0.02)
            if not readable:
                if process.poll() is not None:
                    break
                continue
            buffer.extend(os.read(master, 256))
            for command, payload in packets(buffer):
                if command == VOLTAGE_COMMAND:
                    assert mode in ("normal", "runtime") and not payload
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
                    elif mode in ("normal", "runtime"):
                        positions_reply(master, ids)
                elif command == MOVE_COMMAND:
                    moves.append(payload)
                    if mode == "runtime":
                        disconnect_at = time.monotonic() + 0.4
                else:
                    raise AssertionError(f"unexpected controller command {command}")
        if process.wait(timeout=1) != 0:
            raise AssertionError(f"C++ UART test failed in {mode} mode")
        if mode in ("normal", "timeout", "runtime"):
            assert requests == [SERVO_IDS, (7,)], requests
        else:
            assert requests == [SERVO_IDS], requests
        if mode in ("normal", "runtime"):
            assert len(moves) == 1, moves
            count, duration = struct.unpack("<BH", moves[0][:3])
            targets = tuple(struct.iter_unpack("<BH", moves[0][3:]))
            assert (count, duration) == (18, 120)
            expected = tuple((i, position_for(i) + int(mode == "runtime" and i == 1)) for i in SERVO_IDS)
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


if __name__ == "__main__":
    for case in ("normal", "timeout", "duplicate", "disconnect"):
        run_case(sys.argv[1], case)
        print(f"virtual UART {case} passed", flush=True)
    if len(sys.argv) == 4:
        run_case(sys.argv[2], "runtime", sys.argv[3])
        print("virtual UART worker execution and disconnect passed", flush=True)
