#! /usr/bin/env python3

from pathlib import Path
from util import run_command, run_command_pipe, run_command_redirect
from path_match import path_match
import shlex


def lizard_single_package(
    package_name: str,
    package_path: str,
    output_dir: Path,
    lizard_dir: Path,
    exclude: list,
    ccn: int,
    nloc: int,
    arguments: int,
):

    if path_match(package_path, exclude):
        print("Match exclude path. Skipped " + package_name)
        return

    output_package_dir = output_dir / package_name
    if not output_package_dir.exists():
        output_package_dir.mkdir(parents=True)

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
                str(lizard_dir / "lizard.py"), ccn, nloc, arguments, package_path
            )
        ),
        output_file=(output_package_dir / "index.html"),
    )


def lizard_package(
    output_dir: Path,
    gh_action_dir: Path,
    timestamp: str,
    exclude: list,
    ccn: int,
    nloc: int,
    arguments: int,
):

    output_lizard_dir = output_dir / "lizard_result" / timestamp
    if not output_lizard_dir.exists():
        output_lizard_dir.mkdir(parents=True)

    lizard_dir = gh_action_dir / "lizard"
    if not lizard_dir.exists():
        run_command(
            args=shlex.split(
                "git clone https://github.com/terryyin/lizard.git " + str(lizard_dir)
            )
        )

    package_list = run_command_pipe(["colcon", "list"]).splitlines()
    for line in package_list:
        package = line.split()
        lizard_single_package(
            package_name=package[0],
            package_path=package[1] + "/",
            output_dir=output_lizard_dir,
            lizard_dir=lizard_dir,
            exclude=exclude,
            ccn=ccn,
            nloc=nloc,
            arguments=arguments,
        )
