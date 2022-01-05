#! /usr/bin/env python3

from pathlib import Path
from typing import List

from ros_metrics_reporter.util import path_match
import ros_metrics_reporter.coverage.run_lcov as run_lcov
from ros_metrics_reporter.package_info import Package, PackageInfo


class CoveragePackage:
    def __init__(
        self,
        base_dir: Path,
        package_info: PackageInfo,
        output_dir: Path,
        lcovrc: Path,
        exclude: List[str],
    ):
        self.__package_info = package_info
        self.__base_dir = base_dir
        self.__output_lcov_root_dir = output_dir
        self.__lcovrc = lcovrc
        self.__exclude = exclude

    def __is_exclude(
        self,
        package_name: str,
        package_path: Path,
    ) -> bool:
        if "_msgs" in package_name:
            print(f"Skipped message package: {package_name}")
            return True

        if path_match(str(package_path), self.__exclude):
            print(f"Match exclude path. Skipped {package_name}")
            print(
                f"DEBUG: in coverage_package PATH={package_path} exclude={' '.join(self.__exclude)}"
            )
            return True
        return False

    def __generate_html_report(self, output_dir: Path, package: Package):
        coverage_info_path = self.__base_dir / "build" / package.name / "coverage.info"

        if not coverage_info_path.exists():
            print(
                f"Coverage report for {package.name} does not exist: {coverage_info_path}"
            )
            return

        print(f"Generating Coverage report for {package.name}...")
        if not output_dir.exists():
            output_dir.mkdir(parents=True)

        filtered_path = run_lcov.filter_report(
            coverage_info_path=coverage_info_path,
            base_dir=self.__base_dir,
            output_dir=output_dir,
            lcovrc=self.__lcovrc,
            exclude=self.__exclude,
        )

        run_lcov.generate_html_report(
            coverage_info_path=filtered_path,
            base_dir=self.__base_dir,
            output_dir=output_dir,
            lcovrc=self.__lcovrc,
        )

    def generate_html_reports(self, test_label: str = ""):
        for package in self.__package_info:
            if self.__is_exclude(package.name, package.path):
                continue
            output_package_dir = self.__output_lcov_root_dir / package.name / test_label
            self.__generate_html_report(output_package_dir, package)
