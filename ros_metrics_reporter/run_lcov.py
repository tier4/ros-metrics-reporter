#! /usr/bin/env python3

from pathlib import Path
from typing import List
import shlex
import os

from ros_metrics_reporter.util import run_command


def get_file_size(file: Path) -> int:
    if file.exists():
        return os.stat(file).st_size
    return -1


def concat_build_dir(base_dir: Path, append_dir: str) -> str:
    if append_dir:
        build_dir = f'"{str(base_dir)}/build/{append_dir}"'
    else:
        build_dir = f'"{str(base_dir)}/build"'
    return build_dir


def concat_output_path(base_dir: Path, file_name: str) -> str:
    return f'"{str(base_dir / file_name)}"'


def initialize_lcov(
    base_dir: Path, output_dir: Path, lcovrc: Path, package_name: str = ""
) -> bool:
    # Get a zero-coverage baseline
    if not run_command(
        args=shlex.split(
            f"lcov \
            --config-file {str(lcovrc)} \
            --base-directory {str(base_dir)} \
            --capture \
            --directory {concat_build_dir(base_dir, package_name)} \
            -o {concat_output_path(output_dir, 'lcov.base')} \
            --initial"
        )
    ):
        print("Zero baseline coverage failed.")
        return False
    return True


def run_lcov(
    base_dir: Path,
    output_dir: Path,
    lcovrc: Path,
    package_name: str = "",
    exclude: List[str] = [],
):
    # Get coverage
    if not run_command(
        args=shlex.split(
            f"lcov \
            --config-file {str(lcovrc)} \
            --base-directory {str(base_dir)} \
            --capture \
            --directory {concat_build_dir(base_dir, package_name)} \
            --output-file {concat_output_path(output_dir, 'lcov.run')}"
        )
    ):
        print("Coverage generation failed.")
        return

    # Return if lcov.run is empty
    if get_file_size(output_dir / "lcov.run") <= 0:
        print("Skipped")
        return

    # Combine zero-coverage with coverage information.
    if not run_command(
        args=shlex.split(
            f"lcov \
            --config-file {str(lcovrc)} \
            -a {concat_output_path(output_dir, 'lcov.base')} \
            -a {concat_output_path(output_dir, 'lcov.run')} \
            -o {concat_output_path(output_dir, 'lcov.total')}"
        )
    ):
        print("Coverage combination failed.")
        return

    # Filter test, build, and install files and generate html
    exclude_list_str = " ".join([f'"{s}"' for s in exclude])

    if not run_command(
        args=shlex.split(
            f'lcov \
            --config-file {str(lcovrc)} \
            -r "{str(output_dir)}/lcov.total" \
            "{str(base_dir)}/build/*" \
            "{str(base_dir)}/install/*" \
            "*/test/*" \
            "*/CMakeCCompilerId.c" \
            "*/CMakeCXXCompilerId.cpp" \
            "*_msgs/*" \
            "*/usr/*" \
            "*/opt/*" \
            {exclude_list_str} \
            -o {concat_output_path(output_dir, "lcov.total.filtered")}'
        )
    ):
        print("Filtering failed.")
        return

    if not run_command(
        args=shlex.split(
            f"genhtml \
            --config-file {str(lcovrc)} \
            -p {str(base_dir)} \
            --legend \
            --demangle-cpp {concat_output_path(output_dir, 'lcov.total.filtered')} \
            -o {str(output_dir)}"
        )
    ):
        print("HTML generation failed.")
        return


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
