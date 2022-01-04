from ros_metrics_reporter.coverage.run_lcov import calculate_total_coverage
from ros_metrics_reporter.coverage.run_test import run_test
from ros_metrics_reporter.coverage.coverage_all import CoverageAll
from ros_metrics_reporter.coverage.coverage_package import CoveragePackage
from ros_metrics_reporter.package_info import PackageInfo
from ros_metrics_reporter.util import DirectoryBackup
from ros_metrics_reporter.scraping.lcov import LcovScraping
from typing import List
from pathlib import Path
from ros_metrics_reporter.coverage.coverage_data import *
from ros_metrics_reporter.coverage.read_config import read_lcovrc
from uuid import uuid4


def find_files(directory_list: List[DirectoryBackup], pattern: str) -> List[str]:
    file_list = []
    for directory in directory_list:
        result = [*directory.backup_path.glob(pattern)]
        if result:
            file_list.extend(result)
    return file_list


class CodeCoverageTaskRunner:
    def __init__(self, args):
        self.base_dir = args.base_dir
        self.build_dir = self.base_dir / "build"
        self.html_dir = self.base_dir / str(uuid4())
        self.test_label = args.test_label
        self.lcovrc = args.lcovrc
        self.exclude = args.exclude
        self.coverage_data = CoverageData(threshold=read_lcovrc(self.lcovrc))

    def __run_total_coverage(self):
        run_test(self.base_dir, self.lcovrc)

        self.coverage_all.generate_html_report()
        self.coverage_package.generate_html_reports()

    def __run_label_coverage(self, packages: PackageInfo):
        backup_package_artifacts = []
        backup_total_artifacts = []
        for label in self.test_label:
            run_test(self.base_dir, self.lcovrc, label)

            # Backup directory
            build_dir = DirectoryBackup(self.build_dir)
            build_dir.backup()
            backup_package_artifacts.append(build_dir)
            lcov_dir = DirectoryBackup(self.base_dir / ("lcov." + label))
            lcov_dir.backup()
            backup_total_artifacts.append(lcov_dir)

            self.coverage_all.generate_html_report(label)
            self.coverage_package.generate_html_reports(label)

        # Merge coverage result to calculate total coverage
        for package in packages:
            package_coverage_files = find_files(
                backup_package_artifacts, f"**/{package.name}/coverage.info"
            )
            coverage_info_path = self.build_dir / package.name / "coverage.info"
            calculate_total_coverage(
                package_coverage_files, coverage_info_path, self.lcovrc
            )
        self.coverage_package.generate_html_reports()

        total_coverage_files = find_files(
            backup_total_artifacts, "**/total_coverage.info"
        )
        total_coverage_info_path = self.base_dir / "lcov" / "total_coverage.info"
        calculate_total_coverage(
            total_coverage_files, total_coverage_info_path, self.lcovrc
        )
        self.coverage_all.generate_html_report()

    def run(self, packages: PackageInfo):
        self.coverage_all = CoverageAll(
            base_dir=self.base_dir,
            output_dir=self.html_dir,
            lcovrc=self.lcovrc,
            exclude=self.exclude,
        )
        self.coverage_package = CoveragePackage(
            base_dir=self.base_dir,
            package_info=packages,
            output_dir=self.html_dir,
            lcovrc=self.lcovrc,
            exclude=self.exclude,
        )

        if not self.test_label:
            self.__run_total_coverage()
        else:
            self.__run_label_coverage(packages)

    def save_coverage_value(self, output_dir: Path):
        # Save threshold value
        self.coverage_data.threshold.write(output_dir / "lcov_threshold.json")

        lcov_scraping = LcovScraping()

        # Save coverage value for total coverage
        coverage_list = lcov_scraping.scraping(
            lcov_dir=self.html_dir,
            output_dir=output_dir,
        )
        self.coverage_data.add_coverages(coverage_list)

        # Save coverage value for labeled coverage
        for label in self.test_label:
            coverage_list = lcov_scraping.scraping(
                lcov_dir=self.html_dir,
                output_dir=output_dir,
                test_label=label,
            )
            self.coverage_data.add_coverages(coverage_list)
