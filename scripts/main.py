#! /usr/bin/env python3

import argparse
from pathlib import Path
from datetime import datetime
from distutils import dir_util
import pandas as pd
from pandas.core.frame import DataFrame
from pandas.io.parsers import read_csv

from util import dir_path
from plot_timeseries import plot_timeseries
from create_markdown import copy_template

cols = [
    "date",
    "package_name",
    "type",
    "value",
    "signal",
]


def copy_artifacts(src: Path, dest: Path):
    dest.mkdir(exist_ok=True)
    for package_dir in src.iterdir():
        if package_dir.is_dir():
            package_dest = dest / package_dir.name
            package_dest.mkdir(exist_ok=True)
            dir_util.copy_tree(package_dir, str(package_dest))


def get_trial_record(record_dir: Path) -> DataFrame:
    all_package_metrics = pd.DataFrame(index=[], columns=cols)

    for package_dir in record_dir.iterdir():
        date = datetime.strptime(record_dir.name, "%Y%m%d_%H%M%S")
        package_name = package_dir.name

        coverage_file = package_dir / "coverage.csv"
        if coverage_file.exists():
            coverage_data = pd.read_csv(coverage_file)
            coverage_data["package_name"] = package_name
            coverage_data["date"] = date
            all_package_metrics = pd.concat(
                [all_package_metrics, coverage_data], axis=0
            )

        lizard_file = package_dir / "lizard.csv"
        if lizard_file.exists():
            package_metrics = pd.read_csv(lizard_file)
            package_metrics["package_name"] = package_name
            package_metrics["date"] = date
            all_package_metrics = pd.concat(
                [all_package_metrics, package_metrics], axis=0
            )

    return all_package_metrics


def run(
    base_path: Path,
    hugo_root_dir: Path,
    hugo_template_dir: Path,
    lcov_result_path: Path,
    lizard_result_path: Path,
):
    data_source = pd.DataFrame(index=[], columns=cols)

    for timestamp_dir in base_path.iterdir():
        # Skip latest directory
        if "latest" in timestamp_dir.name:
            continue

        single_record = get_trial_record(timestamp_dir)
        data_source = pd.concat([data_source, single_record], axis=0)

    # Create graph
    plotly_output_dir = hugo_root_dir / "static" / "plotly"
    plotly_output_dir.mkdir(parents=True, exist_ok=True)

    packages = data_source["package_name"].unique()
    for package in packages:
        df = data_source[data_source["package_name"] == package]
        save_dir = plotly_output_dir / package
        save_dir.mkdir(exist_ok=True)
        plot_timeseries(df, save_dir)

    # Copy artifacts
    lcov_dest = hugo_root_dir / "static" / "__lcov"
    copy_artifacts(lcov_result_path, lcov_dest)

    lizard_dest = hugo_root_dir / "static" / "__lizard"
    copy_artifacts(lizard_result_path, lizard_dest)

    # Create markdown from template
    copy_template(hugo_template_dir, hugo_root_dir, base_path / "latest", packages)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input-dir", help="Path to coverage artifacts", type=dir_path, required=True
    )
    parser.add_argument(
        "--hugo-root-dir",
        help="Path to hugo directory to output files",
        type=dir_path,
        required=True,
    )
    parser.add_argument(
        "--hugo-template-dir",
        help="Path to template directory to generate markdown",
        type=dir_path,
        required=True,
    )
    parser.add_argument(
        "--lcov-result-path",
        help="Path to lcov result directory",
        type=dir_path,
        required=True,
    )
    parser.add_argument(
        "--lizard-result-path",
        help="Path to lizard result directory",
        type=dir_path,
        required=True,
    )

    args = parser.parse_args()

    run(
        args.input_dir,
        args.hugo_root_dir,
        args.hugo_template_dir,
        args.lcov_result_path,
        args.lizard_result_path,
    )
