#! /usr/bin/env python3

from pathlib import Path
from typing import List

from ros_metrics_reporter.run_lcov import (
    generate_html_report,
    filter_report,
)


class CoverageAll:
    def __init__(self, output_dir: Path, base_dir: Path, lcovrc: Path):
        self.__output_lcov_dir = output_dir / "all"
        self.__base_dir = base_dir.absolute()
        self.__lcovrc = lcovrc

    def generate_html_report(self, exclude: List[str]):
        if not self.__output_lcov_dir.exists():
            self.__output_lcov_dir.mkdir(parents=True)

        filtered_path = filter_report(
            coverage_info_path=self.__base_dir / "lcov" / "total_coverage.info",
            base_dir=self.__base_dir,
            output_dir=self.__output_lcov_dir,
            lcovrc=self.__lcovrc,
            exclude=exclude,
        )

        generate_html_report(
            coverage_info_path=filtered_path,
            base_dir=self.__base_dir,
            output_dir=self.__output_lcov_dir,
            lcovrc=self.__lcovrc,
        )
