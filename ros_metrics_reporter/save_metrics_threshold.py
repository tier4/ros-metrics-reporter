from pathlib import Path
from typing import Dict


def read_lcovrc(lcovrc: Path) -> Dict[str, int]:
    coverage_limits = {}
    with open(lcovrc) as f:
        for line in f:
            # Skip comment lines
            if "#" == line[0]:
                continue

            if "genhtml_hi_limit" in line:
                coverage_limits["Coverage(Hi)"] = int(line.split()[-1])
            elif "genhtml_med_limit" in line:
                coverage_limits["Coverage(Med)"] = int(line.split()[-1])
    return coverage_limits


def save_data(
    data: Dict[str, int],
    file: Path,
):
    with open(file, "w") as f:
        for key, value in data.items():
            f.write(f"{key},{value}\n")


def save_metrics_threshold(
    metrics_dir: Path,
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

    save_data(metrics, metrics_dir / "metrics_threshold.csv")


def save_lcov_threshold(
    lcovrc: Path,
    metrics_dir: Path,
):
    lcov_threshold = read_lcovrc(lcovrc)
    save_data(lcov_threshold, metrics_dir / "lcov_threshold.csv")


def save_threshold(
    lcovrc: Path,
    metrics_dir: Path,
    ccn: int,
    nloc: int,
    arguments: int,
    ccn_recommendation: int,
    nloc_recommendation: int,
    arguments_recommendation: int,
):
    save_lcov_threshold(
        lcovrc,
        metrics_dir,
    )

    save_metrics_threshold(
        metrics_dir,
        ccn,
        nloc,
        arguments,
        ccn_recommendation,
        nloc_recommendation,
        arguments_recommendation,
    )
