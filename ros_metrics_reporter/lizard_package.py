#! /usr/bin/env python3

from pathlib import Path
from typing import List
import shlex

from ros_metrics_reporter.util import (
    run_command,
    run_command_redirect,
    path_match,
)
from ros_metrics_reporter.package_info import PackageInfo


def lizard_single_package(
    package_name: str,
    package_path: str,
    output_dir: Path,
    lizard_dir: Path,
    exclude: List[str],
    ccn: int,
    nloc: int,
    arguments: int,
):
    if "_msgs" in package_name:
        print(f"Skipped message package: {package_name}")
        return

    if path_match(package_path, exclude):
        print(f"Match exclude path. Skipped {package_name}")
        print(
            f"DEBUG: in lizard_package PATH={package_path} exclude={' '.join(exclude)}"
        )
        return

    output_package_dir = output_dir / package_name
    if not output_package_dir.exists():
        output_package_dir.mkdir(parents=True)

    # TODO: Consider call lizard script from python
    run_command_redirect(
        args=shlex.split(
            f'python3 {str(lizard_dir / "lizard.py")} \
            -l cpp \
            -l python \
            -x "*test*" \
            -x "*lizard*" \
            --CCN {ccn} \
            -T nloc={nloc} \
            --arguments {arguments} \
            --html {package_path}'
        ),
        output_file=(output_package_dir / "index.html"),
    )

    print(f"Generated package metrics: {package_name}")


def lizard_package(
    package_info: PackageInfo,
    output_dir: Path,
    gh_action_dir: Path,
    exclude: List[str],
    ccn: int,
    nloc: int,
    arguments: int,
):

    if not output_dir.exists():
        output_dir.mkdir(parents=True)

    lizard_dir = gh_action_dir / "lizard"
    if not lizard_dir.exists():
        run_command(
            args=shlex.split(
                f"git clone https://github.com/terryyin/lizard.git {str(lizard_dir)}"
            )
        )

    for package in package_info:
        package_full_path = str(package_info.ros_ws / package.path) + "/"
        lizard_single_package(
            package_name=package.name,
            package_path=package_full_path,
            output_dir=output_dir,
            lizard_dir=lizard_dir,
            exclude=exclude,
            ccn=ccn,
            nloc=nloc,
            arguments=arguments,
        )
