#! /usr/bin/env python3

from pathlib import Path
from typing import List
import shlex
from ros_metrics_reporter.metrics.metrics_data import MetricsValue

from ros_metrics_reporter.util import (
    run_command_redirect,
    path_match,
)
from ros_metrics_reporter.package_info import PackageInfo


def is_exclude(package_name: str, package_path: str, exclude: List[str]) -> bool:
    if "_msgs" in package_name:
        print(f"Skipped message package: {package_name}")
        return True

    if path_match(package_path, exclude):
        print(f"Match exclude path. Skipped {package_name}")
        print(
            f"DEBUG: in lizard_package PATH={package_path} exclude={' '.join(exclude)}"
        )
        return True
    return False


def lizard_single_package(
    lizard_executable: Path,
    package_name: str,
    package_path: str,
    output_dir: Path,
    exclude: List[str],
    threshold: MetricsValue,
):
    if is_exclude(package_name, package_path, exclude):
        return

    output_package_dir = output_dir / package_name
    if not output_package_dir.exists():
        output_package_dir.mkdir(parents=True)

    # TODO: Consider call lizard script from python
    run_command_redirect(
        args=shlex.split(
            f'python3 {str(lizard_executable)} \
            -l cpp \
            -l python \
            -x "*test*" \
            -x "*lizard*" \
            --CCN {threshold.ccn} \
            -T nloc={threshold.nloc} \
            --arguments {threshold.parameter} \
            --html {package_path}'
        ),
        output_file=(output_package_dir / "index.html"),
    )

    print(f"Generated package metrics: {output_package_dir}")


def lizard_package(
    lizard_executable: Path,
    package_info: PackageInfo,
    output_dir: Path,
    exclude: List[str],
    threshold: MetricsValue,
):
    if not output_dir.exists():
        output_dir.mkdir(parents=True)

    for package in package_info:
        package_full_path = str(package.git_ws / package.path) + "/"
        lizard_single_package(
            lizard_executable=lizard_executable,
            package_name=package.name,
            package_path=package_full_path,
            output_dir=output_dir,
            exclude=exclude,
            threshold=threshold,
        )
