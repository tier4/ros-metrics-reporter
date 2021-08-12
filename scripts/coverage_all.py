#! /usr/bin/env python3

from pathlib import Path
from typing import List
from run_lcov import initialize_lcov, run_lcov
from colcon_directory import colcon_get_all_packages


class CoverageAll:
    def __init__(self, output_dir: Path, base_dir: Path, lcovrc: Path):
        self.__initialize_success = False
        self.__output_lcov_dir = output_dir / "all"
        self.__base_dir = base_dir
        self.__lcovrc = lcovrc

    def initialize(self):
        if not self.__output_lcov_dir.exists():
            self.__output_lcov_dir.mkdir(parents=True)

        colcon_get_all_packages(self.__base_dir)

        if initialize_lcov(
            base_dir=self.__base_dir,
            output_dir=self.__output_lcov_dir,
            lcovrc=self.__lcovrc,
        ):
            self.__initialize_success = True

    def measure_coverage(self, exclude: List[str]):
        if not self.__initialize_success:
            return

        colcon_get_all_packages(self.__base_dir)

        run_lcov(
            base_dir=self.__base_dir,
            output_dir=self.__output_lcov_dir,
            lcovrc=self.__lcovrc,
            exclude=exclude,
        )
