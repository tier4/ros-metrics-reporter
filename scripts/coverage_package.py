#! /usr/bin/env python3

from pathlib import Path
from typing import List
from util import run_command_pipe, path_match
from run_lcov import initialize_lcov, run_lcov, generate_html_report
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
            print(f"Skipped message package: {package_name}")
            return True

        if path_match(package_path, exclude):
            print(f"Match exclude path. Skipped {package_name}")
            print(
                f"DEBUG: in coverage_package PATH={package_path} exclude={' '.join(exclude)}"
            )
            return True
        return False

    def __initialize_single_package(
        self,
        package_name: str,
    ):
        if not colcon_get_package(self.__base_dir, package_name):
            print(f"Coverage {package_name} failed")
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
            package_name, package_path, _ = line.split()
            package_full_path = str(self.__base_dir / package_path) + "/"

            if self.__is_exclude(package_name, package_full_path, exclude):
                self.__initialize_failed_list.append(package_name)
                continue

            self.__initialize_single_package(
                package_name=package_name,
            )

    def measure_coverage(self):
        for line in self.__package_list:
            package_name, *_ = line.split()

            if package_name in self.__initialize_failed_list:
                continue

            colcon_get_package(self.__base_dir, package_name)

            run_lcov(
                base_dir=self.__base_dir,
                output_dir=self.__output_dir / package_name,
                lcovrc=self.__lcovrc,
                package_name=package_name,
            )

            print(f"Generated package coverage: {package_name}")

    def generate_html_report(self, exclude: List[str]):
        if not self.__output_dir.exists():
            self.__output_dir.mkdir(parents=True)

        for line in self.__package_list:
            package_name, package_path, _ = line.split()
            package_full_path = str(self.__base_dir / package_path) + "/"

            if self.__is_exclude(package_name, package_full_path, exclude):
                self.__initialize_failed_list.append(package_name)
                continue

            generate_html_report(
                coverage_info_path=self.__base_dir
                / "build"
                / package_name
                / "coverage.info",
                base_dir=self.__base_dir,
                output_dir=self.__output_dir / package_name,
                lcovrc=self.__lcovrc,
            )
