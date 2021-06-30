#! /usr/bin/env python3

from pathlib import Path
from util import run_command, run_command_pipe
from run_lcov import initialize_lcov, run_lcov
from path_match import path_match
import shlex


COVERAGE_FLAGS = "-fprofile-arcs -ftest-coverage -DCOVERAGE_RUN=1 -O0"


def coverage_single_package(
    package_name: str,
    package_path: str,
    base_dir: Path,
    output_dir: Path,
    lcovrc: Path,
    exclude: list,
):

    if "_msgs" in package_name:
        print("Skipped " + package_name)
        return

    if path_match(package_path, exclude):
        print("Match exclude path. Skipped " + package_name)
        return

    # Build with correct flags
    if not run_command(
        args=shlex.split(
            'colcon build \
            --event-handlers console_cohesion+ \
            --cmake-args -DCMAKE_BUILD_TYPE=Debug \
            -DCMAKE_CXX_FLAGS="{0}" \
            -DCMAKE_C_FLAGS="{0}" \
            --packages-up-to {1}'.format(
                package_name, COVERAGE_FLAGS
            ),
        )
    ):
        print("Build failed.")
        return

    output_package_dir = output_dir / package_name
    if not output_package_dir.exists():
        output_package_dir.mkdir(parents=True)

    if not initialize_lcov(
        base_dir=base_dir,
        output_dir=output_package_dir,
        lcovrc=lcovrc,
        package_name=package_name,
    ):
        return

    if not run_command(
        args=shlex.split(
            "colcon test \
            --event-handlers console_cohesion+ \
            --packages-select {} \
            --return-code-on-test-failure".format(
                package_name
            )
        )
    ):
        print("Unit/integration testing failed.")
        return

    run_lcov(
        base_dir=base_dir,
        output_dir=output_package_dir,
        lcovrc=lcovrc,
        package_name=package_name,
    )


def coverage_package(
    base_dir: Path, output_dir: Path, timestamp: str, lcovrc: Path, exclude: list
):

    output_lcov_dir = output_dir / "lcov_result" / timestamp
    if not output_lcov_dir.exists():
        output_lcov_dir.mkdir(parents=True)

    package_list = run_command_pipe(["colcon", "list"]).splitlines()
    for line in package_list:
        package = line.split()
        coverage_single_package(
            package_name=package[0],
            package_path=package[1],
            base_dir=base_dir,
            output_dir=output_dir,
            lcovrc=lcovrc,
            exclude=exclude,
        )
