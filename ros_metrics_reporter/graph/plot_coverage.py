from pathlib import Path

from pandas.core.frame import DataFrame
from ros_metrics_reporter.coverage.coverage_data import CoverageKeys, CoverageStamped
from typing import List
from datetime import datetime
import plotly.express as px
import pandas as pd


def plot_graph(df: DataFrame, output_path: Path, label: str):
    fig = px.line(
        df,
        x="date",
        y="value",
        title="Time Series with Range Slider and Selectors",
        color="type",
    )

    fig.update_traces(mode="markers+lines", hovertemplate=None)
    fig.update_layout(hovermode="x unified")

    fig.update_xaxes(
        rangeslider_visible=True,
        rangeselector=dict(
            buttons=list(
                [
                    dict(count=1, label="1M", step="month", stepmode="backward"),
                    dict(count=6, label="6M", step="month", stepmode="backward"),
                    dict(count=1, label="1Y", step="year", stepmode="backward"),
                    dict(step="all"),
                ]
            )
        ),
    )

    output_path.mkdir(exist_ok=True, parents=True)
    fig.write_json(str(output_path / ("coverage." + label + ".json")))
    # fig.write_html(str(output_path / ("coverage." + label + '.html')))


def generate_labeled_graph(coverage_list: List[CoverageStamped], output_dir: Path):
    labels = coverage_list[0].get_labels()
    for label in labels:
        df = pd.DataFrame(columns=["date", "value", "type"])
        for coverage in coverage_list:
            for key in CoverageKeys:
                df = pd.concat(
                    [
                        df,
                        pd.DataFrame(
                            {
                                "date": coverage.date,
                                "value": coverage.get_label_value(label).get(key),
                                "type": key.value,
                            },
                            index=[0],
                        ),
                    ],
                    ignore_index=True,
                )
        plot_graph(df, output_dir, label)


def get_trial_record(record_dir: Path) -> CoverageStamped:
    coverage_data = CoverageStamped()
    coverage_file = record_dir / "coverage.json"
    coverage_data.read(coverage_file)
    return coverage_data


def get_package_list(target_dir: Path) -> List[str]:
    return [path.name for path in sorted(target_dir.iterdir())]


def plot_coverage(base_path: Path, output_dir: Path):
    packages = get_package_list(base_path / "latest")

    for package in packages:
        record_list = []
        for timestamp_dir in sorted(base_path.iterdir()):
            # Skip latest directory
            if timestamp_dir.name == "latest":
                continue

            record_dir = timestamp_dir / package
            if not (record_dir / "coverage.json").exists():
                continue

            single_record = get_trial_record(record_dir)

            date = datetime.strptime(timestamp_dir.name, "%Y%m%d_%H%M%S")
            single_record.date = date

            record_list.append(single_record)

        if record_list:
            generate_labeled_graph(record_list, output_dir / package)
