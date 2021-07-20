from pathlib import Path
import csv


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

    metrics_list = []
    for key, value in metrics.items():
        metrics_list.append({"type": key, "value": value})

    for html_dir in metrics_dir.iterdir():
        filename = html_dir / "lizard.csv"
        with open(filename, "a") as f:
            writer = csv.DictWriter(f, fieldnames=metrics[0].keys())
            writer.writeheader()
            for item in metrics:
                writer.writerow(item)
