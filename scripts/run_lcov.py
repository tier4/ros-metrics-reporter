#! /usr/bin/env python3

from pathlib import Path
from util import run_command
import shlex
import os


def concat_build_dir(base_dir: Path, append_dir: str) -> str:
    if append_dir:
        build_dir = '"{0}/build/{1}"'.format(str(base_dir), append_dir)
    else:
        build_dir = '"{}/build"'.format(str(base_dir))
    return build_dir


def concat_output_path(base_dir: Path, file_name: str) -> str:
    return '"{}"'.format(str(base_dir / file_name))


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
                concat_build_dir(base_dir, package_name),
                concat_output_path(output_dir, "lcov.base"),
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
                concat_build_dir(base_dir, package_name),
                concat_output_path(output_dir, "lcov.run"),
            )
        )
    ):
        print("Coverage generation failed.")
        return

    # Return if lcov.run is empty
    if os.stat(output_dir / "lcov.run").st_size() == 0:
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
                concat_output_path(output_dir, "lcov.base"),
                concat_output_path(output_dir, "lcov.run"),
                concat_output_path(output_dir, "lcov.total"),
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
                concat_output_path(output_dir, "lcov.total.filtered"),
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
                concat_output_path(output_dir, "lcov.total.filtered"),
                str(output_dir),
            )
        )
    ):
        print("HTML generation failed.")
        return
