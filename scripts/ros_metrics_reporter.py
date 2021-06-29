#! /usr/bin/env python3

import argparse
from .coverage_all import coverage_all
from .coverage_package import coverage_package
from .util import dir_path
from pathlib import Path
from .lizard_all import lizard_all
from .lizard_package import lizard_package
from .scraping import scraping
from .create_link import create_link
from .create_static_page import create_static_page


def ros_metrics_reporter(args):
    # Measure code coverage
    coverage_all(
        base_dir=args.base_dir,
        output_dir=args.output_dir,
        timestamp=args.timestamp,
        lcovrc=args.lcovrc,
    )
    coverage_package(
        base_dir=args.base_dir,
        output_dir=args.output_dir,
        timestamp=args.timestamp,
        lcovrc=args.lcovrc,
        exclude=args.exclude,
    )

    # Measure code metrics
    lizard_all(
        base_dir=args.base_dir,
        output_dir=args.output_dir,
        gh_action_dir=args.gh_action_dir,
        timestamp=args.timestamp,
        ccn=args.ccn,
        nloc=args.nloc,
        arguments=args.arguments,
    )
    lizard_package(
        base_dir=args.base_dir,
        output_dir=args.output_dir,
        gh_action_dir=args.gh_action_dir,
        timestamp=args.timestamp,
        exclude=args.exclude,
        ccn=args.ccn,
        nloc=args.nloc,
        arguments=args.arguments,
    )

    lcov_dir = args.output_dir / "lcov_result" / args.timestamp
    lizard_dir = args.output_dir / "lizard_result" / args.timestamp
    metrics_dir = args.output_dir / "metrics"
    metrics_output_dir = metrics_dir / args.timestamp

    # Scraping
    scraping(lcov_dir=lcov_dir, lizard_dir=lizard_dir, output_dir=metrics_output_dir)

    # Create symbolic link
    lcov_latest_dir = lcov_dir = args.output_dir / "lcov_result" / "latest"
    lizard_latest_dir = args.output_dir / "lizard_result" / "latest"
    metrics_latest_dir = metrics_dir / "latest"

    create_link(target=lcov_dir, link_from=lcov_latest_dir)
    create_link(target=lizard_dir, link_from=lizard_latest_dir)
    create_link(target=metrics_output_dir, link_from=metrics_latest_dir)

    # Create static page
    hugo_template_dir = args.action_dir / "template" / "hugo"
    create_static_page(
        input_dir=metrics_dir,
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
    parser.add_argument("--exclude", help="Exclude path", type=list, required=False)
    args = parser.parse_args()

    ros_metrics_reporter(args)
