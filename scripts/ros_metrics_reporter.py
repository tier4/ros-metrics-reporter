#! /usr/bin/env python3

import argparse
from .coverage_all import coverage_all
from .coverage_package import coverage_package
from .util import dir_path
from pathlib import Path
from .lizard_all import lizard_all
from .lizard_package import lizard_package


def ros_metrics_reporter(args):
    # Measure code coverage
    coverage_all(base_dir=args.base_dir, output_dir=args.output_dir, timestamp=args.timestamp, lcovrc=args.lcovrc)
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


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-dir", help="Path to source file directory", type=dir_path, required=True)
    parser.add_argument("--output-dir", help="Path to artifacts directory", type=dir_path, required=True)
    parser.add_argument("--timestamp", help="Timestamp. Required format is %Y%m%d_%H%M%S", type=str, required=True)
    parser.add_argument("--lcovrc", help="Path to .lcovrc", type=Path, required=True)
    parser.add_argument("--exclude", help="Exclude path", type=list, required=False)
    args = parser.parse_args()

    ros_metrics_reporter(args)
