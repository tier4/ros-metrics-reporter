#! /usr/bin/env python3

import argparse
from pathlib import Path
from typing import List

from ros_metrics_reporter.code_coverage_task_runner import CodeCoverageTaskRunner
from ros_metrics_reporter.metrics_task_runner import MetricsTaskRunner
from ros_metrics_reporter.create_link import create_link
from ros_metrics_reporter.create_static_page import create_static_page
from ros_metrics_reporter.clang_tidy import clang_tidy
from ros_metrics_reporter.colcon_directory import *
from ros_metrics_reporter.plot_timeseries import generate_metrics_graph
from ros_metrics_reporter.package_info import PackageInfo
from ros_metrics_reporter.code_activity.code_activity import code_activity
from ros_metrics_reporter.static_page_input import StaticPageInput


def ros_metrics_reporter(args):
    packages = PackageInfo(args.base_dir)
    metrics_dir = args.output_dir / "metrics" / args.timestamp
    metrics_dir.mkdir(parents=True, exist_ok=True)

    # Run code coverage task
    coverage_runner = CodeCoverageTaskRunner(args)
    coverage_runner.run(packages)
    coverage_runner.save_coverage_value(metrics_dir)

    # Measure code metrics
    metrics_runner = MetricsTaskRunner(args)
    metrics_runner.run(packages)
    metrics_runner.save_metrics_value(metrics_dir)

    # Run Clang-Tidy
    tidy_result_dir = args.base_dir / "tidy-reports" / args.timestamp
    clang_tidy(
        base_dir=args.base_dir,
        output_dir=tidy_result_dir,
        gh_action_dir=args.action_dir,
        config_path=args.tidy_config_path,
        ignore_path=args.tidy_ignore_path,
    )

    # Create symbolic link
    metrics_latest_dir = args.output_dir / "metrics" / "latest"
    create_link(target=Path(args.timestamp), link_from=metrics_latest_dir)

    # Create graph
    generate_metrics_graph(
        args.hugo_root_dir,
        metrics_dir.parent,
    )

    # generate code frequency graph
    code_frequency_output_dir = args.hugo_root_dir / "static" / "plotly"
    contributors = code_activity(
        github_target_repo=args.target_repo,
        package_info=packages,
        code_frequency_graph_output_dir=code_frequency_output_dir,
        github_access_token=args.github_access_token,
    )

    # Create static page
    hugo_template_dir = args.action_dir / "template" / "hugo"

    static_page_input = StaticPageInput(
        input_dir=metrics_dir.parent,
        packages=packages,
        hugo_root_dir=args.hugo_root_dir,
        hugo_template_dir=hugo_template_dir,
        tidy_result_path=tidy_result_dir,
        base_url=args.base_url,
        title=args.title,
        contributors=contributors,
        code_coverage=coverage_runner,
        metrics=metrics_runner,
    )

    create_static_page(
        input=static_page_input,
    )


def dir_path(input):
    if Path(input).is_dir():
        return Path(input)
    else:
        raise NotADirectoryError(input)


def space_separated_string(input: str) -> List[str]:
    return input.split()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--base-dir", help="Path to source file directory", type=dir_path, required=True
    )
    parser.add_argument(
        "--target-repo",
        help="Target repository. ex. tier4/ros-metrics-reporter",
        type=str,
        required=True,
    )
    parser.add_argument(
        "--action-dir",
        help="Path to ros-metrics-reporter directory",
        type=dir_path,
        required=True,
    )
    parser.add_argument(
        "--output-dir", help="Path to artifacts directory", type=dir_path, required=True
    )
    parser.add_argument(
        "--timestamp",
        help="Timestamp. Required format is %Y%m%d_%H%M%S",
        type=str,
        required=True,
    )
    parser.add_argument("--lcovrc", help="Path to .lcovrc", type=Path, required=True)
    parser.add_argument(
        "--hugo-root-dir", help="Hugo root directory", type=dir_path, required=True
    )
    parser.add_argument("--base-url", help="baseURL", type=str, required=True)
    parser.add_argument("--title", help="Title", type=str, required=True)
    parser.add_argument(
        "--extra-cmake-args", help="Extra cmake args", type=str, required=False
    )
    parser.add_argument(
        "--exclude", help="Exclude path", type=space_separated_string, required=False
    )
    parser.add_argument("--ccn", help="CCN", type=int, required=True)
    parser.add_argument(
        "--ccn-recommendation", help="CCN recommend value", type=int, required=True
    )
    parser.add_argument("--nloc", help="NLOC", type=int, required=True)
    parser.add_argument(
        "--nloc-recommendation", help="NLOC recommend value", type=int, required=True
    )
    parser.add_argument("--arguments", help="arguments", type=int, required=True)
    parser.add_argument(
        "--arguments-recommendation",
        help="arguments recommend value",
        type=int,
        required=True,
    )
    parser.add_argument(
        "--tidy-config-path",
        help="Path to codechecker-config.json",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--tidy-ignore-path",
        help="Path to codechecker-skip-list.txt",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--github-access-token",
        help="Github access token",
        type=str,
        default=None,
    )
    parser.add_argument(
        "--test-label",
        help="Test label",
        type=space_separated_string,
        required=False,
    )

    args = parser.parse_args()

    ros_metrics_reporter(args)
