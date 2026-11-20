"""Exercise the real multi-cycle preview executable and check its CSV contract."""

import csv
import io
import json
import math
from pathlib import Path
import re
import subprocess
import sys
from tempfile import TemporaryDirectory


TIME_TOLERANCE_S = 1e-9
PHASES = (
    ((0.0, 0.5, 2.0, 2.5, 4.0), 10),
    ((4.0, 5.5, 7.0), 6),
    ((7.0, 8.25, 9.5, 10.75, 12.0), 4),
    ((12.0, 12.15625, 13.25, 13.40625, 14.5), 8),
)


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def run_preview(executable, *arguments):
    return subprocess.run(
        [str(executable), *map(str, arguments)],
        capture_output=True,
        text=True,
        encoding="utf-8",
        timeout=10,
        check=False,
    )


def parse_positions(fields, config, location):
    servo = config["servo"]
    mounting = config["mounting"]
    minimum, maximum = servo["user_configured_position_limits"]
    positions = []
    for servo_id, field in enumerate(fields, start=1):
        require(re.fullmatch(r"[0-9]+", field),
                f"{location} p{servo_id} must be an integer field")
        position = int(field)
        require(minimum <= position <= maximum,
                f"{location} p{servo_id} outside the configured position range")
        axis = str(servo_id)
        angle_deg = (
            mounting["joint_reference_angles_deg"][axis]
            + mounting["joint_angle_signs"][axis]
            * (position - servo["reference_position"])
            * servo["degrees_per_position_unit"]
        )
        limits = mounting["installed_joint_limits"][axis]
        require(limits["min_deg"] - 1e-10 <= angle_deg <= limits["max_deg"] + 1e-10,
                f"{location} p{servo_id} outside its installed angular limits")
        positions.append(position)
    return tuple(positions)


def verify_csv(stdout, config):
    rows = list(csv.reader(io.StringIO(stdout)))
    expected_header = [
        "cycle_index", "end_frame_index", "start_time_s", "end_time_s", "duration_s",
        *[f"p{servo_id}" for servo_id in range(1, 19)],
    ]
    require(len(rows) == 101, "CSV must contain a header and all 100 segments")
    require(rows[0] == expected_header, "CSV must have exactly 23 ordered columns")

    row_index = 1
    previous_end = 0.0
    for cycle_index, (boundaries, intervals_per_stage) in enumerate(PHASES):
        end_frame_index = 0
        cycle_positions = set()
        for stage_start, stage_end in zip(boundaries, boundaries[1:]):
            expected_duration = (stage_end - stage_start) / intervals_per_stage
            for step in range(intervals_per_stage):
                end_frame_index += 1
                location = f"cycle {cycle_index}, end frame {end_frame_index}"
                row = rows[row_index]
                row_index += 1
                require(len(row) == len(expected_header), f"{location} has missing or extra values")
                require(row[:2] == [str(cycle_index), str(end_frame_index)],
                        f"{location} has incorrect cycle or frame index")
                start, end, duration = map(float, row[2:5])
                expected_start = stage_start + step * expected_duration
                expected_end = stage_start + (step + 1) * expected_duration
                for name, actual, expected in (
                    ("start", start, expected_start),
                    ("end", end, expected_end),
                    ("duration", duration, expected_duration),
                ):
                    require(math.isclose(actual, expected, rel_tol=0.0, abs_tol=TIME_TOLERANCE_S),
                            f"{location} {name} time {actual} does not match {expected}")
                require(end > start and duration > 0.0, f"{location} has a non-positive duration")
                require(math.isclose(start, previous_end, rel_tol=0.0, abs_tol=TIME_TOLERANCE_S),
                        f"{location} breaks the continuous timeline")
                positions = parse_positions(row[5:], config, location)
                if cycle_index == 0 and end_frame_index == 1:
                    expected_first = (500, 550, 200) * 3 + (500, 450, 800) * 3
                    require(positions == expected_first, "first segment must use physical ID order")
                cycle_positions.add(positions)
                previous_end = end
        require(len(cycle_positions) > 1, f"cycle {cycle_index} never changes the nominal positions")


def verify_failed_invocation(executable, *arguments):
    result = run_preview(executable, *arguments)
    require(result.returncode != 0, f"invalid invocation succeeded: {arguments}")
    require(result.stdout == "", f"invalid invocation printed partial CSV: {arguments}")
    require(result.stderr.strip(), f"invalid invocation omitted its error diagnostic: {arguments}")
    return result.stderr


def main():
    require(len(sys.argv) == 3, "usage: tripod_sequence_preview_cli_tests.py executable hardware.json")
    executable, config_path = (Path(arg).resolve() for arg in sys.argv[1:])
    config = json.loads(config_path.read_text(encoding="utf-8-sig"))
    result = run_preview(executable, config_path)
    require(result.returncode == 0, f"preview failed: {result.stderr}")
    require("OFFLINE" in result.stderr and "100 segments" in result.stderr,
            "missing offline success diagnostic for all 100 segments")
    verify_csv(result.stdout, config)

    verify_failed_invocation(executable)
    verify_failed_invocation(executable, config_path, "unexpected")
    with TemporaryDirectory(prefix="hexapod preview ") as temporary:
        root = Path(temporary)
        verify_failed_invocation(executable, root / "missing.json")
        malformed = root / "malformed.json"
        malformed.write_text("{", encoding="utf-8")
        verify_failed_invocation(executable, malformed)

        # The preview's initial right-front knee angle fits this interval.
        # Its subsequent planned motion requires a target outside the interval.
        config["mounting"]["installed_joint_limits"]["3"] = {"min_deg": -72.05, "max_deg": -71.95}
        restrictive = root / "restrictive limits.json"
        restrictive.write_text(json.dumps(config), encoding="utf-8")
        diagnostic = verify_failed_invocation(executable, restrictive)
        require("cycle_index=" in diagnostic,
                "restrictive configuration must reach sequence planning and identify the failed cycle")

    print("PASS: preview CLI, 100x23 CSV, four cycle boundaries, joint limits and error paths")


if __name__ == "__main__":
    main()
