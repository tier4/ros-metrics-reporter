#! /usr/bin/env python3

from pathlib import Path
import shlex

from ros_metrics_reporter.util import run_command


def run_test(base_dir: Path, lcovrc: Path, test_label: str = "") -> None:
    if not test_label:
        lcov_base = "lcov"
        coveragepy_base = "coveragepy"
    else:
        lcov_base = "lcov." + test_label
        coveragepy_base = "coveragepy." + test_label

    run_command(
        args=shlex.split(
            f"colcon lcov-result --initial --lcov-config-file {lcovrc} --lcov-base {lcov_base}"
        ),
        cwd=base_dir,
    )
    run_command(
        args=shlex.split(
            f"colcon test --event-handlers console_cohesion+ --ctest-args -L {test_label}"
        ),
        cwd=base_dir,
    )
    run_command(
        args=shlex.split(
            f"colcon lcov-result --lcov-config-file {lcovrc} --verbose --lcov-base {lcov_base}"
        ),
        cwd=base_dir,
    )
    run_command(
        args=shlex.split(
            f"colcon coveragepy-result --verbose --coverage-report-args -m --coveragepy-base {coveragepy_base}"
        ),
        cwd=base_dir,
    )
