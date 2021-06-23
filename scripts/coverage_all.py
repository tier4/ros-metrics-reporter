#! /usr/bin/env python3

from pathlib import Path
from .util import run_command
from .run_lcov import initialize_lcov, run_lcov


COVERAGE_FLAGS = "-fprofile-arcs -ftest-coverage -DCOVERAGE_RUN=1 -O0"


def coverage_all(base_dir: Path, output_dir: Path, timestamp: str, lcovrc: Path):

    output_lcov_dir = output_dir / "lcov_result" / timestamp / "all"
    if not output_lcov_dir.exists():
        output_lcov_dir.mkdir(parents=True)

    # Build with correct flags
    if not run_command(
        args=[
            "colcon",
            "build",
            "--event-handlers",
            "console_cohesion+",
            "--cmake-args",
            "-DCMAKE_BUILD_TYPE=Debug",
            '-DCMAKE_CXX_FLAGS="{}"'.format(COVERAGE_FLAGS),
            '-DCMAKE_C_FLAGS="{}"'.format(COVERAGE_FLAGS),
        ]
    ):
        print("Build failed.")
        return

    if not initialize_lcov(base_dir=base_dir, output_dir=output_lcov_dir, lcovrc=lcovrc):
        return

    if not run_command(
        args=[
            "colcon",
            "test",
            "--event-handlers",
            "console_cohesion+",
            "--return-code-on-test-failure",
        ]
    ):
        print("Unit/integration testing failed.")
        return

    run_lcov(base_dir=base_dir, output_dir=output_lcov_dir, lcovrc=lcovrc)
