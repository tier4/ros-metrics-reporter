#! /usr/bin/env python3

import argparse
from coverage_all import *
from coverage_package import *
from util import dir_path
from pathlib import Path
from lizard_all import lizard_all
from lizard_package import lizard_package
from scraping import scraping
from create_link import create_link
from create_static_page import create_static_page
from clang_tidy import clang_tidy
from save_metrics_threshold import save_threshold
from colcon import *


def ros_metrics_reporter(args):
    exclude = args.exclude.split()
    colcon = Colcon(target_path=args.base_dir)

    # Build packages
    colcon.build()

    # Initialize coverage
    lcov_dir = args.output_dir / "lcov_result" / args.timestamp
    coverage_all = CoverageAll(
        base_dir=args.base_dir, output_dir=lcov_dir, lcovrc=args.lcovrc
    )
    coverage_package = CoveragePackage(
        base_dir=args.base_dir, output_dir=lcov_dir, lcovrc=args.lcovrc
    )

    coverage_all.initialize()
    coverage_package.initialize(exclude=exclude)

    # Test packages
    colcon.test()

    # Measure code coverage
    coverage_all.measure_coverage(exclude=exclude)
    coverage_package.measure_coverage()

    # Measure code metrics for threshold value
    lizard_dir = args.output_dir / "lizard_result" / args.timestamp
    lizard_all(
        base_dir=args.base_dir,
        output_dir=lizard_dir,
        gh_action_dir=args.action_dir,
        ccn=args.ccn,
        nloc=args.nloc,
        arguments=args.arguments,
        exclude=exclude,
    )
    lizard_package(
        base_dir=args.base_dir,
        output_dir=lizard_dir,
        gh_action_dir=args.action_dir,
        exclude=exclude,
        ccn=args.ccn,
        nloc=args.nloc,
        arguments=args.arguments,
    )

    # Measure code metrics for recommend value
    lizard_recommendation_dir = (
        args.output_dir / "lizard_result_recommend" / args.timestamp
    )
    lizard_all(
        base_dir=args.base_dir,
        output_dir=lizard_recommendation_dir,
        gh_action_dir=args.action_dir,
        ccn=args.ccn_recommendation,
        nloc=args.nloc_recommendation,
        arguments=args.arguments_recommendation,
        exclude=exclude,
    )
    lizard_package(
        base_dir=args.base_dir,
        output_dir=lizard_recommendation_dir,
        gh_action_dir=args.action_dir,
        exclude=exclude,
        ccn=args.ccn_recommendation,
        nloc=args.nloc_recommendation,
        arguments=args.arguments_recommendation,
    )

    # Run Clang-Tidy
    tidy_result_dir = args.output_dir / "tidy-reports" / args.timestamp
    clang_tidy(
        base_dir=args.base_dir,
        output_dir=tidy_result_dir,
        gh_action_dir=args.action_dir,
        config_path=args.tidy_config_path,
        ignore_path=args.tidy_ignore_path,
    )

    # Scraping
    metrics_dir = args.output_dir / "metrics" / args.timestamp
    metrics_dir.mkdir(parents=True, exist_ok=True)
    scraping(
        lcov_dir=lcov_dir,
        lizard_dir=lizard_dir,
        lizard_recommendation_dir=lizard_recommendation_dir,
        output_dir=metrics_dir,
    )

    save_threshold(
        lcovrc=args.lcovrc,
        ccn=args.ccn,
        nloc=args.nloc,
        arguments=args.arguments,
        ccn_recommendation=args.ccn_recommendation,
        nloc_recommendation=args.nloc_recommendation,
        arguments_recommendation=args.arguments_recommendation,
        metrics_dir=metrics_dir,
    )

    # Create symbolic link
    lcov_latest_dir = args.output_dir / "lcov_result" / "latest"
    lizard_latest_dir = args.output_dir / "lizard_result" / "latest"
    metrics_latest_dir = args.output_dir / "metrics" / "latest"

    create_link(target=Path(args.timestamp), link_from=lcov_latest_dir)
    create_link(target=Path(args.timestamp), link_from=lizard_latest_dir)
    create_link(target=Path(args.timestamp), link_from=metrics_latest_dir)

    # Create static page
    hugo_template_dir = args.action_dir / "template" / "hugo"

    create_static_page(
        input_dir=metrics_dir.parent,
        hugo_root_dir=args.hugo_root_dir,
        hugo_template_dir=hugo_template_dir,
        lcov_result_path=lcov_latest_dir,
        lizard_result_path=lizard_latest_dir,
        tidy_result_path=tidy_result_dir,
        base_url=args.base_url,
        title=args.title,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--base-dir", help="Path to source file directory", type=dir_path, required=True
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
    parser.add_argument("--exclude", help="Exclude path", type=str, required=False)
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

    args = parser.parse_args()

    ros_metrics_reporter(args)
