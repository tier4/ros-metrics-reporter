#! /usr/bin/env python3

from pathlib import Path
from .util import run_command, run_command_redirect
import shlex


def lizard_all(
    base_dir: Path,
    output_dir: Path,
    gh_action_dir: Path,
    timestamp: str,
    ccn: int,
    nloc: int,
    arguments: int,
):

    output_lizard_dir = output_dir / "lizard_result" / timestamp / "all"
    if not output_lizard_dir.exists():
        output_lizard_dir.mkdir(parents=True)

    lizard_dir = gh_action_dir / "lizard"
    if not lizard_dir.exists():
        run_command(
            args=shlex.split(
                "git clone https://github.com/terryyin/lizard.git " + lizard_dir,
            )
        )

    # TODO: Consider call lizard script from python
    run_command_redirect(
        args=shlex.split(
            'python3 {0} \
            -l cpp \
            -l python \
            -x "*test*" \
            -x "*lizard*" \
            --CCN {1} \
            -T nloc={2} \
            --arguments {3} \
            --html {4}'.format(
                str(lizard_dir / "lizard.py"), ccn, nloc, arguments, base_dir
            )
        ),
        output_file=(output_lizard_dir / "index.html"),
    )
