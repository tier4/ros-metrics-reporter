#! /usr/bin/env python3

from pathlib import Path
from typing import List
import shlex

from ros_metrics_reporter.util import run_command


def concat_output_path(base_dir: Path, file_name: str) -> str:
    return f'"{str(base_dir / file_name)}"'


def filter_report(
    coverage_info_path: str,
    base_dir: Path,
    output_dir: Path,
    lcovrc: Path,
    exclude: List[str] = [],
) -> str:
    """ Filter test, build, and install files and generate html """

    exclude_list_str = " ".join([f'"{s}"' for s in exclude])
    filtered_coverage_info_path = concat_output_path(output_dir, "coverage.filtered")

    if not run_command(
        args=shlex.split(
            f'lcov \
            --config-file {str(lcovrc)} \
            -r "{coverage_info_path}" \
            "{str(base_dir)}/build/*" \
            "{str(base_dir)}/install/*" \
            "*/test/*" \
            "*/CMakeCCompilerId.c" \
            "*/CMakeCXXCompilerId.cpp" \
            "*_msgs/*" \
            "*/usr/*" \
            "*/opt/*" \
            {exclude_list_str} \
            -o {filtered_coverage_info_path}'
        )
    ):
        print("Filtering failed.")
        return
    return filtered_coverage_info_path


def generate_html_report(
    coverage_info_path: str,
    base_dir: Path,
    output_dir: Path,
    lcovrc: Path,
):
    if not run_command(
        args=shlex.split(
            f"genhtml \
            --config-file {str(lcovrc)} \
            -p {str(base_dir)} \
            --legend \
            --demangle-cpp {coverage_info_path} \
            -o {str(output_dir)}"
        )
    ):
        print("HTML generation failed.")
        return


def calculate_total_coverage(
    coverage_files: List[Path], output_path: Path, lcovrc: Path
):
    """ Calculate total coverage """
    if not output_path.parent.exists():
        output_path.parent.mkdir(parents=True)

    append_option = ""
    for coverage_file in coverage_files:
        append_option += f"-a {str(coverage_file)} "

    if not run_command(
        args=shlex.split(
            f"lcov \
            --config-file {str(lcovrc)} \
            {append_option} \
            -o {output_path}"
        )
    ):
        print("Filtering failed.")
        return
