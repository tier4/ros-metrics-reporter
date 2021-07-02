#! /usr/bin/env python3

import os
from pathlib import Path
from util import run_command

import shlex


def clang_tidy(
    base_dir: Path, output_dir: Path, gh_action_dir: Path, config_path: Path
):

    # Build
    if not run_command(
        args=shlex.split(
            "colcon build \
            --event-handlers console_cohesion+ \
            --cmake-args -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
        )
    ):
        print("Build failed.")
        return

    codechecker_dir = gh_action_dir / "codechecker"
    # if not codechecker_dir.exists():
    #     run_command(args=shlex.split("git clone https://github.com/Ericsson/codechecker.git " + str(codechecker_dir)))

    # env = dict(os.environ)
    # env["BUILD_UI_DIST"] = "NO"
    # run_command(
    #     args=shlex.split("make package"),
    #     cwd=codechecker_dir,
    #     env=env,
    # )

    compile_command = base_dir / "build" / "compile_commands.json"
    ignore_path = gh_action_dir / "codechecker-skip-list.txt"
    # codechecker_exe = codechecker_dir / "build" / "CodeChecker" / "bin" / "CodeChecker"
    run_command(
        args=shlex.split(
            "build/CodeChecker/bin/CodeChecker analyze {0} --config {1} --ignore {2} --output ./reports".format(
                compile_command, config_path, ignore_path
            )
        ),
        cwd=codechecker_dir,
    )

    run_command(
        args=shlex.split(
            "build/CodeChecker/bin/CodeChecker parse -e html ./reports -o {0} --trim-path-prefix {1}".format(
                output_dir, base_dir
            )
        ),
        cwd=codechecker_dir,
    )
