from pathlib import Path
from typing import Dict
from ros_metrics_reporter.coverage.read_config import read_lcovrc


def save_data(
    data: Dict[str, int],
    file: Path,
):
    with open(file, "w") as f:
        for key, value in data.items():
            f.write(f"{key},{value}\n")


def save_metrics_threshold(
    output_dir: Path,
    ccn: int,
    nloc: int,
    arguments: int,
    ccn_recommendation: int,
    nloc_recommendation: int,
    arguments_recommendation: int,
):
    metrics = {
        "CCN(threshold)": ccn,
        "LOC(threshold)": nloc,
        "Parameter(threshold)": arguments,
        "CCN(recommendation)": ccn_recommendation,
        "LOC(recommendation)": nloc_recommendation,
        "Parameter(recommendation)": arguments_recommendation,
    }

    save_data(metrics, output_dir / "metrics_threshold.csv")


def save_lcov_threshold(
    lcovrc: Path,
    output_dir: Path,
):
    lcov_threshold = read_lcovrc(lcovrc).to_dict()
    save_data(lcov_threshold, output_dir / "lcov_threshold.csv")


def save_threshold(
    lcovrc: Path,
    output_dir: Path,
    ccn: int,
    nloc: int,
    arguments: int,
    ccn_recommendation: int,
    nloc_recommendation: int,
    arguments_recommendation: int,
):
    save_lcov_threshold(
        lcovrc,
        output_dir,
    )

    save_metrics_threshold(
        output_dir,
        ccn,
        nloc,
        arguments,
        ccn_recommendation,
        nloc_recommendation,
        arguments_recommendation,
    )
