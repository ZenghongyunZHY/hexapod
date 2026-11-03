#!/usr/bin/env python3
"""Exercise the real executables against a pseudo-UART; never opens GPIO UART."""
import argparse
import csv
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import threading
import time


class Board:
    def __init__(self, mode='normal'):
        self.master, self.slave = pty.openpty()
        self.port = os.ttyname(self.slave)
        os.set_blocking(self.master, False)
        self.mode = mode
        self.commands = []
        self.moves = []
        self.errors = []
        self.pose = [500] * 18
        self.origin = list(self.pose)
        self.target = list(self.pose)
        self.move_start = 0
        self.duration = 0
        self.first_move = threading.Event()
        self.position_requests = []
        self.partial_sent = False
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.loop, daemon=True)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *_):
        self.stop.set()
        self.thread.join(timeout=1)
        os.close(self.master)
        os.close(self.slave)

    def current(self):
        fraction = min(1, (time.monotonic() - self.move_start) / self.duration) if self.duration else 1
        return [round(a + (b-a)*fraction) for a,b in zip(self.origin, self.target)]

    def respond(self, command, payload):
        packet = bytes([0x55, 0x55, len(payload)+2, command, *payload])
        # Deliberate noise and fragmented header: checks parser resynchronization.
        os.write(self.master, b'\x00\x55\x01' + packet[:3])
        os.write(self.master, packet[3:])

    def handle(self, packet):
        command, data = packet[3], packet[4:]
        self.commands.append(command)
        if self.mode == 'silent':
            return
        if command == 15:
            value = 9000 if self.mode == 'overvoltage' else 7400
            if self.mode.startswith('voltage_'):
                value = int(self.mode.split('_')[1])
            self.respond(15, [value & 255, value >> 8])
        elif command == 21:
            assert len(data) == data[0]+1 and 1 <= data[0] <= 18, 'Incorrect position query size'
            requested = list(data[1:])
            assert len(set(requested)) == len(requested) and all(1 <= i <= 18 for i in requested)
            self.position_requests.append(requested)
            positions = self.current()
            returned = [i for i in requested if not (self.mode == 'missing_id' and i == 18)]
            if not self.partial_sent and 7 in returned and (self.mode == 'transient_missing' or
                    (self.mode == 'partial_after_motion' and self.moves)):
                returned.remove(7)
                self.partial_sent = True
            if self.mode == 'duplicate_id' and len(returned) > 1:
                returned[-1] = returned[0]
            payload = [len(returned)]
            for servo_id in returned:
                position = positions[servo_id-1]
                payload.extend([servo_id, position & 255, position >> 8])
            if self.mode == 'bad_count':
                payload[0] += 1
            self.respond(21, payload)
        elif command == 3:
            assert len(packet) == 61 and packet[2] == 59 and data[0] == 18
            duration = data[1] | (data[2] << 8)
            assert 20 <= duration <= 5000
            positions = []
            for i in range(18):
                index = 3+i*3
                assert data[index] == i+1
                value = data[index+1] | (data[index+2] << 8)
                assert 0 <= value <= 1000
                positions.append(value)
            self.origin = self.current()
            self.target = positions
            self.move_start = time.monotonic()
            self.duration = duration/1000
            self.moves.append((duration, positions))
            self.first_move.set()
        else:
            raise AssertionError(f'Unexpected command {command}: no action groups or unloading allowed')

    def loop(self):
        pending = bytearray()
        try:
            while not self.stop.is_set():
                if not select.select([self.master], [], [], .02)[0]:
                    continue
                pending.extend(os.read(self.master, 1024))
                while len(pending) >= 4:
                    assert pending[:2] == b'\x55\x55', 'Malformed outgoing header'
                    count = pending[2]+2
                    if len(pending) < count:
                        break
                    packet = bytes(pending[:count])
                    del pending[:count]
                    self.handle(packet)
        except Exception as error:
            self.errors.append(str(error))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('executables', type=Path)
    parser.add_argument('frames', type=Path)
    args=parser.parse_args()
    binaries=args.executables.resolve()
    frame_path=args.frames.resolve()
    with frame_path.open() as stream:
        rows=csv.DictReader(line for line in stream if not line.startswith('#'))
        table={row['name']:[int(row[f'id{i}']) for i in range(1,19)] for row in rows}

    def run(exe, extra, timeout=5):
        return subprocess.run([str(binaries/exe),'--frames',str(frame_path),*extra],text=True,capture_output=True,timeout=timeout)

    result=run('walk_demo',['--port','/device/that/does/not/exist'])
    assert result.returncode == 0 and 'DRY RUN' in result.stdout
    assert result.stdout.count(' cycle=6 ') == 6 and ' cycle=7 ' not in result.stdout
    print('PASS: default mode never opens a UART; six complete cycles previewed.',flush=True)

    result=run('stand_demo',['--execute','--dry-run'])
    assert result.returncode != 0 and 'Choose only one mode' in result.stderr
    for mode in ('normal','missing_id','duplicate_id','bad_count','overvoltage','silent'):
        with Board(mode) as board:
            result=run('stand_demo',['--probe' if mode=='normal' else '--execute','--port',board.port])
            assert result.returncode == (0 if mode=='normal' else 1),result.stderr
            assert not board.moves and not board.errors,(mode,board.errors)
        print(f'PASS: {mode} communication preflight sends no motion.',flush=True)

    with Board('transient_missing') as board:
        result=run('stand_demo',['--probe','--port',board.port])
        assert result.returncode == 0,(result.stdout,result.stderr,board.errors)
        assert board.position_requests == [list(range(1,19)),[7]],board.position_requests
        assert 'POSITION_RETRY' in result.stderr and not board.moves and not board.errors
    print('PASS: a 17-servo response is completed by a targeted ID7 read, without motion.',flush=True)

    for voltage, accepted in ((5999,False),(6000,True),(8661,True),(8700,True),(8701,False)):
        with Board(f'voltage_{voltage}') as board:
            result=run('stand_demo',['--probe','--port',board.port])
            assert result.returncode == (0 if accepted else 1),(voltage,result.stderr)
            assert not board.moves and not board.errors,(voltage,board.errors)
        print(f'PASS: demo voltage {voltage} mV '+('accepted' if accepted else 'rejected')+'; no motion.',flush=True)

    with Board() as board:
        process=subprocess.Popen([str(binaries/'stand_demo'),'--frames',str(frame_path),'--execute','--port',board.port],text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        try:
            assert board.first_move.wait(3),'No first standing target'
            time.sleep(.15)
            process.send_signal(signal.SIGINT)
            stdout,stderr=process.communicate(timeout=3)
            assert process.returncode == 130,(stdout,stderr)
            assert len(board.moves) == 2 and board.moves[-1][0] == 20,(board.moves,stderr)
            assert board.moves[-1][1] != table['stand'],'Interrupt must not force the final standing target'
            assert 'HOLD:' in stderr and not board.errors,(stderr,board.errors)
        finally:
            if process.poll() is None:
                process.kill();process.wait()
    print('PASS: Ctrl+C replaces an in-flight movement with measured-position hold.',flush=True)

    with Board('partial_after_motion') as board:
        result=run('walk_demo',['--execute','--port',board.port],timeout=75)
        assert result.returncode == 0,(result.stdout,result.stderr,board.errors)
        completions=[line for line in result.stdout.splitlines() if line.startswith('CYCLE_COMPLETE ')]
        assert completions == [f'CYCLE_COMPLETE {i}/6' for i in range(1,7)],completions
        assert len(board.moves) == 44,len(board.moves)
        assert board.moves[0][1] == board.moves[-1][1] == table['stand']
        assert [7] in board.position_requests and 'POSITION_RETRY' in result.stderr
        assert set(board.commands) == {3,15,21} and not board.errors,board.errors
    print('PASS: exactly six complete cycles, 44 finite frames including standing/recovery, ends at identical standing; no action groups or unloading.',flush=True)
    print('ALL VIRTUAL-UART TESTS PASSED. This does not verify physical standing or walking.',flush=True)


if __name__=='__main__':
    main()
