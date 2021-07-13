#! /usr/bin/env python3

from pathlib import Path
from util import run_command

import shlex


def clang_tidy(
    base_dir: Path,
    output_dir: Path,
    gh_action_dir: Path,
    config_path: Path,
    ignore_path: Path,
):

    # Build
    if not run_command(
        args=shlex.split(
            "bash -c 'source /opt/ros/foxy/setup.bash && colcon build \
            --event-handlers console_cohesion+ \
            --cmake-args -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'"
        ),
        cwd=base_dir,
    ):
        print("Build failed.")
        return

    codechecker_dir = gh_action_dir / "codechecker"
    if not codechecker_dir.exists():
        run_command(
            args=shlex.split(
                "git clone https://github.com/Ericsson/codechecker.git "
                + str(codechecker_dir)
            )
        )

    run_command(
        args=shlex.split("bash -c 'make venv'"),
        cwd=codechecker_dir,
    )

    run_command(
        args=shlex.split(
            "bash -c 'source venv/bin/activate && BUILD_UI_DIST=NO make package && deactivate'"
        ),
        cwd=codechecker_dir,
    )

    compile_command = base_dir / "build" / "compile_commands.json"
    run_command(
        args=shlex.split(
            "bash -c 'source {0}/venv/bin/activate && {0}/build/CodeChecker/bin/CodeChecker analyze {1} --config {2} --ignore {3} --output ./reports && deactivate'".format(
                codechecker_dir, compile_command, config_path, ignore_path
            )
        ),
    )

    run_command(
        args=shlex.split(
            "bash -c 'source {0}/venv/bin/activate && {0}/build/CodeChecker/bin/CodeChecker parse -e html ./reports -o {1} --trim-path-prefix {2} && deactivate'".format(
                codechecker_dir, output_dir, base_dir
            )
        ),
    )
