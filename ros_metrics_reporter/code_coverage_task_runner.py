from ros_metrics_reporter.coverage.run_lcov import calculate_total_coverage
from ros_metrics_reporter.coverage.run_test import run_test
from ros_metrics_reporter.coverage.coverage_all import CoverageAll
from ros_metrics_reporter.coverage.coverage_package import CoveragePackage
from ros_metrics_reporter.package_info import PackageInfo
from ros_metrics_reporter.util import DirectoryBackup
from ros_metrics_reporter.scraping.lcov import LcovScraping
from typing import List
from pathlib import Path


def find_files(directory_list: List[DirectoryBackup], pattern: str) -> List[str]:
    file_list = []
    for directory in directory_list:
        result = directory.backup_path.glob(pattern)
        file_list.append(list(result))
    return file_list


class CodeCoverageTaskRunner:
    def __init__(self, args):
        self.base_dir = args.base_dir
        self.output_dir = args.output_dir / "lcov_result" / args.timestamp
        self.test_label = args.test_label
        self.lcovrc = args.lcovrc
        self.exclude = args.exclude

    def run(self, packages: PackageInfo):
        coverage_all = CoverageAll(
            base_dir=self.base_dir,
            output_dir=self.output_dir,
            lcovrc=self.lcovrc,
            exclude=self.exclude,
        )
        coverage_package = CoveragePackage(
            base_dir=self.base_dir,
            package_info=packages,
            output_dir=self.output_dir,
            lcovrc=self.lcovrc,
            exclude=self.exclude,
        )

        if not self.test_label:
            run_test(self.base_dir, self.lcovrc)

            coverage_all.generate_html_report()
            coverage_package.generate_html_reports()
            return

        backup_package_artifacts = []
        backup_total_artifacts = []
        for label in self.test_label:
            run_test(self.base_dir, self.lcovrc, label)

            # Backup directory
            build_dir = DirectoryBackup(self.base_dir / "build")
            build_dir.backup()
            backup_package_artifacts.append(build_dir)
            lcov_dir = DirectoryBackup(self.base_dir / ("lcov." + label))
            lcov_dir.backup()
            backup_total_artifacts.append(lcov_dir)

            coverage_all.generate_html_report(label)
            coverage_package.generate_html_reports(label)

        # Merge coverage result to calculate total coverage
        for package in packages:
            package_coverage_files = find_files(
                backup_package_artifacts, f"**/{package.name}/coverage.info"
            )
            calculate_total_coverage(
                package_coverage_files, self.output_dir / package.name, self.lcovrc
            )

        total_coverage_files = find_files(
            backup_total_artifacts, "**/total_coverage.info"
        )
        calculate_total_coverage(
            total_coverage_files, self.output_dir / "all", self.lcovrc
        )

    def save_coverage_value(self, output_dir: Path):
        LcovScraping().scraping(
            lcov_dir=self.output_dir,
            output_dir=output_dir,
        )
