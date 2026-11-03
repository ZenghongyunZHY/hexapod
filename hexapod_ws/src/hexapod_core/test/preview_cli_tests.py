"""Verify the real offline preview executable, including its CSV and error paths."""

import csv
import io
import json
import math
from pathlib import Path
import subprocess
import sys


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def run_preview(executable, *arguments):
    return subprocess.run(
        [str(executable), *map(str, arguments)],
        capture_output=True,
        text=True,
        encoding="utf-8",
        timeout=20,
        check=False,
    )


def main():
    require(len(sys.argv) == 3, "usage: preview_cli_tests.py executable hardware.json")
    executable = Path(sys.argv[1]).resolve()
    config_path = Path(sys.argv[2]).resolve()
    config = json.loads(config_path.read_text(encoding="utf-8-sig"))
    result = run_preview(executable, config_path)
    require(result.returncode == 0, f"preview failed: {result.stderr}")
    require("20 segments planned successfully" in result.stderr, "missing success diagnostic")
    reader = csv.DictReader(io.StringIO(result.stdout))
    expected_header = [
        "time_s", "target_x_m", "target_y_m", "target_z_m",
        "nominal_x_m", "nominal_y_m", "nominal_z_m",
        *[f"p{index}" for index in range(1, 19)],
    ]
    require(reader.fieldnames == expected_header, "CSV must have exactly 25 ordered columns")
    rows = list(reader)
    require(len(rows) == 20, "CSV must contain all 20 segment endpoints")
    minimum, maximum = config["servo"]["user_configured_position_limits"]
    expected_initial = [500, 550, 200] * 3 + [500, 450, 800] * 3
    expected_moving_ids = {4, 5, 6}
    initial_x = -0.00004
    initial_y = -0.079 - (
        0.042 + 0.0742 * math.cos(math.radians(12))
        + 0.11253 * math.cos(math.radians(-60))
    )
    initial_z = 0.021 + 0.0742 * math.sin(math.radians(12)) \
        + 0.11253 * math.sin(math.radians(-60))
    largest_csv_error_m = 0.0
    moving_positions = []
    for index, row in enumerate(rows, start=1):
        require(None not in row and all(value is not None for value in row.values()),
                "CSV row has missing or extra values")
        numbers = {key: float(row[key]) for key in expected_header[:7]}
        require(all(math.isfinite(value) for value in numbers.values()), "nonfinite CSV value")
        require(abs(numbers["time_s"] - index * 0.1) < 1e-12,
                "planned time must increase from 0.1 to 2 seconds")
        require(abs(numbers["target_y_m"] - initial_y) <= 0.500001e-6,
                "ideal swing must keep body Y constant")
        require(initial_x - 0.500001e-6 <= numbers["target_x_m"]
                <= initial_x + 0.010 + 0.500001e-6, "ideal X outside planned range")
        positions = [int(row[f"p{servo}"]) for servo in range(1, 19)]
        require(all(minimum <= position <= maximum for position in positions),
                "position outside configured installed range")
        for servo, position in enumerate(positions, start=1):
            if servo not in expected_moving_ids:
                require(position == expected_initial[servo - 1], "one of 15 fixed axes moved")
        moving_positions.append(tuple(positions[3:6]))
        error = math.sqrt(sum(
            (numbers[f"target_{axis}_m"] - numbers[f"nominal_{axis}_m"]) ** 2
            for axis in "xyz"
        ))
        largest_csv_error_m = max(largest_csv_error_m, error)
    require(len(set(moving_positions)) > 1, "moving leg never changes")
    require(abs(float(rows[-1]["target_x_m"]) - (initial_x + 0.010)) <= 0.500001e-6,
            "landing endpoint must be 10 mm forward")
    require(abs(float(rows[9]["target_z_m"]) - (initial_z + 0.008)) <= 0.500001e-6,
            "midpoint must be 8 mm above baseline")
    require(abs(float(rows[-1]["target_z_m"]) - initial_z) <= 0.500001e-6,
            "landing endpoint must return to baseline height")
    require(largest_csv_error_m < 0.002, "nominal CSV error exceeds offline test bound")

    missing_path = config_path.parent / "__missing_preview_cli_config__.json"
    require(not missing_path.exists(), "missing-config test path unexpectedly exists")
    for arguments in [(), (missing_path,), (config_path, "unexpected")]:
        failed = run_preview(executable, *arguments)
        require(failed.returncode != 0 and failed.stdout == "",
                "invalid invocation must fail without partial CSV output")
        require("ERROR:" in failed.stderr, "failed invocation needs an error diagnostic")
    print(f"PASS: preview CLI, 20x25 CSV, 15 fixed axes, error paths; "
          f"max rounded-CSV nominal error={largest_csv_error_m * 1000:.6f} mm")


if __name__ == "__main__":
    main()
