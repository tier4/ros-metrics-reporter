#! /usr/bin/env python3

import pandas as pd
from pandas.core.frame import DataFrame
from pathlib import Path
from typing import List
from datetime import datetime

cols = [
    "date",
    "package_name",
    "type",
    "value",
    "signal",
]


def get_package_list(target_dir: Path) -> List[str]:
    return [path.name for path in sorted(target_dir.iterdir())]


def get_trial_record(record_dir: Path, allowed_packages: List[str]) -> DataFrame:
    all_package_metrics = pd.DataFrame(index=[], columns=cols)

    for package_dir in record_dir.iterdir():
        package_name = package_dir.name
        if not package_name in allowed_packages:
            continue

        date = datetime.strptime(record_dir.name, "%Y%m%d_%H%M%S")

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


def read_dataframe(base_path: Path) -> pd.DataFrame:
    df = pd.DataFrame(index=[], columns=cols)

    packages = get_package_list(base_path / "latest")

    for timestamp_dir in sorted(base_path.iterdir()):
        # Skip latest directory
        if "latest" in timestamp_dir.name:
            continue

        single_record = get_trial_record(timestamp_dir, packages)
        df = pd.concat([df, single_record], axis=0)
    return df
