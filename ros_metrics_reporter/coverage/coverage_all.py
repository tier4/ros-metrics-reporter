#! /usr/bin/env python3

from pathlib import Path
from typing import List

import run_lcov


class CoverageAll:
    def __init__(
        self, output_dir: Path, base_dir: Path, lcovrc: Path, exclude: List[str]
    ):
        self.__output_lcov_dir = output_dir / "all"
        self.__base_dir = base_dir.absolute()
        self.__lcovrc = lcovrc
        self.__exclude = exclude

    def __generate_html_report(self, output_dir: Path, coverage_info_dir_name: str):
        if not output_dir.exists():
            output_dir.mkdir(parents=True)

        coverage_info_path = (
            self.__base_dir / coverage_info_dir_name / "total_coverage.info"
        )
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

    def generate_html_report(self, test_label: str = ""):
        # Generate HTML report for all packages
        print("Generating Coverage report for all packages...")

        # Set test directory name to 'lcov' if test_label is empty
        # Otherwise, set it to 'lcov.test_label'
        if not test_label:
            self.__generate_html_report(self.__output_lcov_dir, "lcov")
        else:
            output_dir = self.__output_lcov_dir / test_label
            coverage_info_dir_name = f"lcov.{test_label}"
            self.__generate_html_report(output_dir, coverage_info_dir_name)
