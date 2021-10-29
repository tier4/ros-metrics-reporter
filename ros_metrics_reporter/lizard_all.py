#! /usr/bin/env python3

from pathlib import Path
from typing import List
import shlex

from ros_metrics_reporter.util import run_command, run_command_redirect


def lizard_all(
    base_dir: Path,
    output_dir: Path,
    gh_action_dir: Path,
    ccn: int,
    nloc: int,
    arguments: int,
    exclude: List[str],
):

    output_lizard_dir = output_dir / "all"
    if not output_lizard_dir.exists():
        output_lizard_dir.mkdir(parents=True)

    lizard_dir = gh_action_dir / "lizard"
    if not lizard_dir.exists():
        run_command(
            args=shlex.split(
                f"git clone https://github.com/terryyin/lizard.git {str(lizard_dir)}"
            )
        )

    # TODO: Consider call lizard script from python
    exclude_list_str = " ".join([f'-x "{s}"' for s in exclude])

    run_command_redirect(
        args=shlex.split(
            f'python3 {str(lizard_dir / "lizard.py")} \
            -l cpp \
            -l python \
            -x "*test*" \
            -x "*lizard*" \
            {exclude_list_str} \
            --CCN {ccn} \
            -T nloc={nloc} \
            --arguments {arguments} \
            --html {base_dir}'
        ),
        output_file=(output_lizard_dir / "index.html"),
    )
