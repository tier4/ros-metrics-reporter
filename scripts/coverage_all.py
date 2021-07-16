#! /usr/bin/env python3

from pathlib import Path
from typing import List
from run_lcov import initialize_lcov, run_lcov
from colcon import colcon_get_all_packages


def coverage_all(base_dir: Path, output_dir: Path, lcovrc: Path, exclude: List[str]):

    output_lcov_dir = output_dir / "all"
    if not output_lcov_dir.exists():
        output_lcov_dir.mkdir(parents=True)

    colcon_get_all_packages(base_dir)

    if not initialize_lcov(
        base_dir=base_dir, output_dir=output_lcov_dir, lcovrc=lcovrc
    ):
        return

    run_lcov(
        base_dir=base_dir, output_dir=output_lcov_dir, lcovrc=lcovrc, exclude=exclude
    )
