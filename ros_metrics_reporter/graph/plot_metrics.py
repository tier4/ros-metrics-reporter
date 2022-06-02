from pathlib import Path
from datetime import datetime
from typing import List
import pandas as pd
import plotly.express as px

from ros_metrics_reporter.metrics.metrics_data import (
    MetricsDataKeys,
    MetricsDataStamped,
    MetricsValueKeys,
)


def plot_graph(df: pd.DataFrame, output_path: Path, metrics_type: str):
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
    fig.write_json(str(output_path / (metrics_type + ".json")))
    # fig.write_html(str(output_path / (metrics_type + '.html')))


def generate_metrics_graph(record_list: List[MetricsDataStamped], output_dir: Path):
    for metrics_type in MetricsValueKeys:
        df = pd.DataFrame(columns=["date", "value", "type"])
        for record in record_list:
            for key in MetricsDataKeys:
                df = pd.concat(
                    [
                        df,
                        pd.DataFrame(
                            {
                                "date": record.date,
                                "value": record.get_value(key).get(metrics_type),
                                "type": key.value,
                            },
                            index=[0],
                        ),
                    ],
                    ignore_index=True,
                )
        plot_graph(df, output_dir, metrics_type.value)


def get_trial_record(record_dir: Path) -> MetricsDataStamped:
    trial_record = MetricsDataStamped()
    metrics_file = record_dir / "lizard.json"
    trial_record.read(metrics_file)
    return trial_record


def get_package_list(target_dir: Path) -> List[str]:
    return [path.name for path in sorted(target_dir.iterdir())]


def plot_metrics(base_path: Path, output_dir: Path):
    packages = get_package_list(base_path / "latest")

    for package in packages:
        record_list = []
        for timestamp_dir in sorted(base_path.iterdir()):
            # Skip latest directory
            if timestamp_dir.name == "latest":
                continue

            record_dir = timestamp_dir / package
            if not (record_dir / "lizard.json").exists():
                continue

            single_record = get_trial_record(record_dir)

            date = datetime.strptime(timestamp_dir.name, "%Y%m%d_%H%M%S")
            single_record.date = date

            record_list.append(single_record)

        if record_list:
            generate_metrics_graph(record_list, output_dir / package)
