#! /usr/bin/env python3

from pathlib import Path
from typing import List

from ros_metrics_reporter.util import path_match
from ros_metrics_reporter.run_lcov import (
    generate_html_report,
    filter_report,
)
from ros_metrics_reporter.package_info import PackageInfo


class CoveragePackage:
    def __init__(self, output_dir: Path, lcovrc: Path):
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

    def generate_html_report(self, package_info: PackageInfo, exclude: List[str]):
        if not self.__output_dir.exists():
            self.__output_dir.mkdir(parents=True)

        for package in package_info:
            print(f"Generating Coverage report for {package.name}...")
            package_path_str = str(package.path) + "/"
            if self.__is_exclude(package.name, package_path_str, exclude):
                continue

            output_package_dir = self.__output_dir / package.name
            if not output_package_dir.exists():
                output_package_dir.mkdir(parents=True)

            filtered_path = filter_report(
                coverage_info_path=package_info.ros_ws
                / "build"
                / package.name
                / "coverage.info",
                base_dir=package_info.ros_ws,
                output_dir=output_package_dir,
                lcovrc=self.__lcovrc,
                exclude=exclude,
            )

            generate_html_report(
                coverage_info_path=filtered_path,
                base_dir=package_info.ros_ws,
                output_dir=output_package_dir,
                lcovrc=self.__lcovrc,
            )
