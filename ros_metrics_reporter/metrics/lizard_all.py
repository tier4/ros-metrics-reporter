#! /usr/bin/env python3

from pathlib import Path
from typing import List
import shlex
from ros_metrics_reporter.metrics.metrics_data import MetricsValue

from ros_metrics_reporter.util import run_command_redirect


def lizard_all(
    lizard_executable: Path,
    base_dir: Path,
    output_dir: Path,
    threshold: MetricsValue,
    exclude: List[str],
):
    output_lizard_dir = output_dir / "all"
    if not output_lizard_dir.exists():
        output_lizard_dir.mkdir(parents=True)

    # TODO: Consider call lizard script from python
    exclude_list_str = " ".join([f'-x "{s}"' for s in exclude])

    run_command_redirect(
        args=shlex.split(
            f'python3 {str(lizard_executable)} \
            -l cpp \
            -l python \
            -x "*test*" \
            -x "*lizard*" \
            {exclude_list_str} \
            --CCN {threshold.ccn} \
            -T nloc={threshold.nloc} \
            --arguments {threshold.parameter} \
            --html {base_dir}'
        ),
        output_file=(output_lizard_dir / "index.html"),
    )
