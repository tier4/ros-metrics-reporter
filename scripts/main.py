#! /usr/bin/env python3

import argparse
from pathlib import Path
from datetime import datetime
from distutils import dir_util
import pandas as pd
from pandas.core import base
from pandas.core.frame import DataFrame
from pandas.io.parsers import read_csv
from jinja2 import Environment, FileSystemLoader

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


def read_data_source(base_path: Path) -> pd.DataFrame:
    data_source = pd.DataFrame(index=[], columns=cols)

    for timestamp_dir in base_path.iterdir():
        # Skip latest directory
        if "latest" in timestamp_dir.name:
            continue

        single_record = get_trial_record(timestamp_dir)
        data_source = pd.concat([data_source, single_record], axis=0)
    return data_source


def generate_graph(
    hugo_root_dir: Path,
    data_source: pd.DataFrame,
):
    # Create graph
    plotly_output_dir = hugo_root_dir / "static" / "plotly"
    plotly_output_dir.mkdir(parents=True, exist_ok=True)

    packages = data_source["package_name"].unique()
    for package in packages:
        df = data_source[data_source["package_name"] == package]
        save_dir = plotly_output_dir / package
        save_dir.mkdir(exist_ok=True)
        plot_timeseries(df, save_dir)


def copy_html(
    hugo_root_dir: Path,
    lcov_result_path: Path,
    lizard_result_path: Path,
    tidy_result_path: Path,
):
    # Copy artifacts
    lcov_dest = hugo_root_dir / "static" / "lcov"
    copy_artifacts(lcov_result_path, lcov_dest)

    lizard_dest = hugo_root_dir / "static" / "lizard"
    copy_artifacts(lizard_result_path, lizard_dest)

    tidy_dest = hugo_root_dir / "static" / "tidy"
    dir_util.copy_tree(tidy_result_path, str(tidy_dest))


def replace_hugo_config(
    hugo_root_dir: Path,
    base_url: str,
    title: str,
):
    config_file = hugo_root_dir / "config.toml"

    env = Environment(loader=FileSystemLoader(str(hugo_root_dir)))
    template = env.get_template(config_file.name)

    with open(config_file, "w") as f:
        f.write(
            template.render(
                {
                    "base_url": base_url,
                    "title": title,
                }
            )
        )


def generate_markdown(
    base_path: Path,
    hugo_root_dir: Path,
    hugo_template_dir: Path,
    packages: str,
):
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
    parser.add_argument(
        "--tidy-result-path",
        help="Path to clang-tidy result directory",
        type=dir_path,
        required=True,
    )
    parser.add_argument(
        "--base-url",
        help="baseURL",
        type=str,
    )

    parser.add_argument(
        "--title",
        help="Title",
        type=str,
    )

    args = parser.parse_args()

    df = read_data_source(args.input_dir)
    generate_graph(args.hugo_root_dir, df)
    copy_html(
        args.hugo_root_dir,
        args.lcov_result_path,
        args.lizard_result_path,
        args.tidy_result_path,
    )
    replace_hugo_config(
        args.hugo_root_dir,
        args.base_url,
        args.title,
    )
    generate_markdown(
        args.input_dir,
        args.hugo_root_dir,
        args.hugo_template_dir,
        df["package_name"].unique(),
    )
