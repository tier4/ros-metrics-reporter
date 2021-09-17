#! /usr/bin/env python3

from pathlib import Path
from typing import List
from util import run_command_pipe, path_match
from run_lcov import initialize_lcov, run_lcov
from colcon_directory import colcon_get_package


class CoveragePackage:
    def __init__(self, output_dir: Path, base_dir: Path, lcovrc: Path):
        self.__package_list = run_command_pipe(
            ["colcon", "list"], cwd=base_dir
        ).splitlines()
        self.__initialize_failed_list = []
        self.__base_dir = base_dir
        self.__output_dir = output_dir
        self.__lcovrc = lcovrc

    def __is_exclude(
        self,
        package_name: str,
        package_path: str,
        exclude: List[str],
    ) -> bool:
        if "_msgs" in package_name:
            print("Skipped message package: " + package_name)
            return True

        if path_match(package_path, exclude):
            print("Match exclude path. Skipped " + package_name)
            print(
                "DEBUG: in coverage_package PATH="
                + package_path
                + " exclude="
                + " ".join(exclude)
            )
            return True
        return False

    def __initialize_single_package(
        self,
        package_name: str,
    ):
        if not colcon_get_package(self.__base_dir, package_name):
            print("Coverage " + package_name + " failed")
            self.__initialize_failed_list.append(package_name)
            return

        output_package_dir = self.__output_dir / package_name
        if not output_package_dir.exists():
            output_package_dir.mkdir(parents=True)

        if not initialize_lcov(
            base_dir=self.__base_dir,
            output_dir=output_package_dir,
            lcovrc=self.__lcovrc,
            package_name=package_name,
        ):
            self.__initialize_failed_list.append(package_name)
            return

    def initialize(self, exclude: List[str]):
        if not self.__output_dir.exists():
            self.__output_dir.mkdir(parents=True)

        for line in self.__package_list:
            package = line.split()
            package_name = package[0]
            package_full_path = str(self.__base_dir / package[1]) + "/"

            if self.__is_exclude(package_name, package_full_path, exclude):
                self.__initialize_failed_list.append(package_name)
                continue

            self.__initialize_single_package(
                package_name=package_name,
            )

    def measure_coverage(self):
        for line in self.__package_list:
            package = line.split()
            package_name = package[0]

            if package_name in self.__initialize_failed_list:
                continue

            colcon_get_package(self.__base_dir, package_name)

            run_lcov(
                base_dir=self.__base_dir,
                output_dir=self.__output_dir / package_name,
                lcovrc=self.__lcovrc,
                package_name=package_name,
            )

            print("Generated package coverage: " + package_name)
