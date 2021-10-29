#! /usr/bin/env python3

from pathlib import Path
import shlex

from ros_metrics_reporter.util import run_command


def clang_tidy(
    base_dir: Path,
    output_dir: Path,
    gh_action_dir: Path,
    config_path: Path,
    ignore_path: Path,
):
    codechecker_dir = gh_action_dir / "codechecker"
    if not codechecker_dir.exists():
        run_command(
            args=shlex.split(
                f"git clone https://github.com/Ericsson/codechecker.git {str(codechecker_dir)}"
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
            f"bash -c 'source {codechecker_dir}/venv/bin/activate && {codechecker_dir}/build/CodeChecker/bin/CodeChecker analyze {compile_command} --config {config_path} --ignore {ignore_path} --output ./reports && deactivate'"
        ),
    )

    run_command(
        args=shlex.split(
            f"bash -c 'source {codechecker_dir}/venv/bin/activate && {codechecker_dir}/build/CodeChecker/bin/CodeChecker parse -e html ./reports -o {output_dir} --trim-path-prefix {base_dir} && deactivate'"
        ),
    )
