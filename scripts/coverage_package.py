#! /usr/bin/env python3

from pathlib import Path
from typing import List
from util import run_command_pipe
from run_lcov import initialize_lcov, run_lcov
from path_match import path_match
from colcon import colcon_get_packages


def coverage_single_package(
    package_name: str,
    package_path: str,
    base_dir: Path,
    output_dir: Path,
    lcovrc: Path,
    exclude: List[str],
):

    if "_msgs" in package_name:
        print("Skipped message package: " + package_name)
        return

    if path_match(package_path, exclude):
        print("Match exclude path. Skipped " + package_name)
        print(
            "DEBUG: in coverage_package PATH="
            + package_path
            + " exclude="
            + " ".join(exclude)
        )
        return

    colcon_get_packages(package_name)

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

    run_lcov(
        base_dir=base_dir,
        output_dir=output_package_dir,
        lcovrc=lcovrc,
        package_name=package_name,
    )

    print("Generated package coverage: " + package_name)


def coverage_package(
    base_dir: Path, output_dir: Path, lcovrc: Path, exclude: List[str]
):

    if not output_dir.exists():
        output_dir.mkdir(parents=True)

    package_list = run_command_pipe(["colcon", "list"]).splitlines()
    for line in package_list:
        package = line.split()
        package_full_path = str(base_dir / package[1]) + "/"
        coverage_single_package(
            package_name=package[0],
            package_path=package_full_path,
            base_dir=base_dir,
            output_dir=output_dir,
            lcovrc=lcovrc,
            exclude=exclude,
        )
