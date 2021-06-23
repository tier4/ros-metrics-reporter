#! /usr/bin/env python3

from pathlib import Path
from .util import run_command

def format_build_dir(base_dir: str, append_dir: str) -> str:
    if append_dir:
        build_dir = '"{0}/build/{1}"'.format(base_dir, append_dir)
    else:
        build_dir = '"{}/build"'.format(base_dir)
    return build_dir


def format_output_dir(base_dir: str, append_dir: str, file_name: str) -> str:
    return '"{}"'.format(Path(base_dir, append_dir, file_name))


def initialize_lcov(base_dir: Path, output_dir: Path, lcovrc: Path, package_name: str ="") -> bool:
    # Get a zero-coverage baseline
    if not run_command(
        args=[
            "lcov",
            "--config-file",
            str(lcovrc),
            "--base-directory",
            str(base_dir),
            "--capture",
            "--directory",
            format_build_dir(str(base_dir), package_name),
            "-o",
            format_output_dir(str(output_dir), package_name, "lcov.base"),
            "--initial",
        ]
    ):
        print("Zero baseline coverage failed.")
        return False
    return True


def run_lcov(base_dir: Path, output_dir: Path, lcovrc: Path, package_name: str =""):
    # Get coverage
    if not run_command(
        args=[
            "lcov",
            "--config-file",
            '"{}"'.format(str(lcovrc)),
            "--base-directory",
            str(base_dir),
            "--capture",
            "--directory",
            format_build_dir(str(base_dir), package_name),
            "--output-file",
            format_output_dir(str(output_dir), package_name, "lcov.run"),
        ]
    ):
        print("Coverage generation failed.")
        return

    # Return if lcov.run is empty
    if not Path(format_output_dir(str(output_dir), package_name, "lcov.run")).exists():
        print("Skipped")
        return

    # Combine zero-coverage with coverage information.
    if not run_command(
        args=[
            "lcov",
            "--config-file",
            str(lcovrc),
            "-a",
            format_output_dir(str(output_dir), package_name, "lcov.base"),
            "-a",
            format_output_dir(str(output_dir), package_name, "lcov.run"),
            "-o",
            format_output_dir(str(output_dir), package_name, "lcov.total"),
        ]
    ):
        print("Coverage combination failed.")
        return

    # Filter test, build, and install files and generate html
    if not run_command(
        args=[
            "lcov",
            "--config-file",
            str(lcovrc),
            "-r" '"{}/lcov.total"'.format(str(output_dir)),
            '"{}/build/*"'.format(str(base_dir)),
            '"{}/install/*"'.format(str(base_dir)),
            '"*/test/*"',
            '"*/CMakeCCompilerId.c"',
            '"*/CMakeCXXCompilerId.cpp"',
            '"*_msgs/*"',
            '"*/usr/*"',
            '"*/opt/*"',
            "-o",
            format_output_dir(str(output_dir), package_name, "lcov.total.filtered"),
        ]
    ):
        print("Filtering failed.")
        return

    if not run_command(
        args=[
            "genhtml",
            "--config-file",
            str(lcovrc),
            "-p",
            str(base_dir),
            "--legend",
            "--demangle-cpp",
            format_output_dir(str(output_dir), package_name, "lcov.total.filtered"),
            "-o",
            format_output_dir(str(output_dir), package_name, ""),
        ]
    ):
        print("HTML generation failed.")
        return
