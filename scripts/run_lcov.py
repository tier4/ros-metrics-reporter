#! /usr/bin/env python3

from pathlib import Path
from .util import run_command
import shlex


def format_build_dir(base_dir: str, append_dir: str) -> str:
    if append_dir:
        build_dir = '"{0}/build/{1}"'.format(base_dir, append_dir)
    else:
        build_dir = '"{}/build"'.format(base_dir)
    return build_dir


def format_output_dir(base_dir: str, append_dir: str, file_name: str) -> str:
    return '"{}"'.format(Path(base_dir, append_dir, file_name))


def initialize_lcov(
    base_dir: Path, output_dir: Path, lcovrc: Path, package_name: str = ""
) -> bool:
    # Get a zero-coverage baseline
    if not run_command(
        args=shlex.split(
            "lcov \
            --config-file {0} \
            --base-directory {1} \
            --capture \
            --directory {2} \
            -o {3} \
            --initial".format(
                str(lcovrc),
                str(base_dir),
                format_build_dir(str(base_dir), package_name),
                format_output_dir(str(output_dir), package_name, "lcov.base"),
            )
        )
    ):
        print("Zero baseline coverage failed.")
        return False
    return True


def run_lcov(base_dir: Path, output_dir: Path, lcovrc: Path, package_name: str = ""):
    # Get coverage
    if not run_command(
        args=shlex.split(
            "lcov \
            --config-file {0} \
            --base-directory {1} \
            --capture \
            --directory {2} \
            --output-file {3}".format(
                str(lcovrc),
                str(base_dir),
                format_build_dir(str(base_dir), package_name),
                format_output_dir(str(output_dir), package_name, "lcov.run"),
            )
        )
    ):
        print("Coverage generation failed.")
        return

    # Return if lcov.run is empty
    if not Path(format_output_dir(str(output_dir), package_name, "lcov.run")).exists():
        print("Skipped")
        return

    # Combine zero-coverage with coverage information.
    if not run_command(
        args=shlex.split(
            "lcov \
            --config-file {0} \
            -a {1} \
            -a {2} \
            -o {3}".format(
                str(lcovrc),
                format_output_dir(str(output_dir), package_name, "lcov.base"),
                format_output_dir(str(output_dir), package_name, "lcov.run"),
                format_output_dir(str(output_dir), package_name, "lcov.total"),
            )
        )
    ):
        print("Coverage combination failed.")
        return

    # Filter test, build, and install files and generate html
    if not run_command(
        args=shlex.split(
            'lcov \
            --config-file {0} \
            -r "{1}/lcov.total" \
            "{2}/build/*" \
            "{2}/install/*" \
            "*/test/*" \
            "*/CMakeCCompilerId.c" \
            "*/CMakeCXXCompilerId.cpp" \
            "*_msgs/*" \
            "*/usr/*" \
            "*/opt/*" \
            -o {3}'.format(
                str(lcovrc),
                str(output_dir),
                str(base_dir),
                format_output_dir(str(output_dir), package_name, "lcov.total.filtered"),
            )
        )
    ):
        print("Filtering failed.")
        return

    if not run_command(
        args=shlex.split(
            "genhtml \
            --config-file {0} \
            -p {1} \
            --legend \
            --demangle-cpp {2} \
            -o {3}".format(
                str(lcovrc),
                str(base_dir),
                format_output_dir(str(output_dir), package_name, "lcov.total.filtered"),
                format_output_dir(str(output_dir), package_name, ""),
            )
        )
    ):
        print("HTML generation failed.")
        return
