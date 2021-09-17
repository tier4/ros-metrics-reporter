#! /usr/bin/env python3

import pandas as pd
from pandas.core.frame import DataFrame
import plotly.express as px
from pathlib import Path
from read_dataframe import read_dataframe


plot_name_list = [
    {
        "coverage": {
            "Lines": "Line coverage",
            "Functions": "Function coverage",
            "Branches": "Branch coverage",
        }
    },
    {
        "ccn": {
            "CCN(worst)": "CCN (Worst case)",
            "CCN(average)": "CCN (Average)",
            "CCN(violate)": "CCN (Violation count)",
        }
    },
    {
        "loc": {
            "LOC(worst)": "LOC (Worst case)",
            "LOC(average)": "LOC (Average)",
            "LOC(violate)": "LOC (Violation count)",
        }
    },
    {
        "parameter": {
            "Parameter(worst)": "Parameter count (Worst case)",
            "Parameter(average)": "Parameter count (average)",
            "Parameter(violate)": "Parameter count (Violation count)",
        }
    },
    {
        "token": {
            "Token(worst)": "Token count (Worst case)",
            "Token(average)": "Token count (average)",
        }
    },
]


def plot_timeseries(df: DataFrame, output_path: Path):
    """Plot time-series and write to json"""
    for plot_group in plot_name_list:
        for plot_type, plot_items in plot_group.items():
            key_list = list(plot_items.keys())
            df_filtered = df.query("type in @key_list")
            if df_filtered.empty:
                continue

            fig = px.line(
                df_filtered,
                x="date",
                y="value",
                title="Time Series with Range Slider and Selectors",
                color="type",
            )

            fig.update_traces(mode="markers+lines", hovertemplate=None)
            fig.update_layout(hovermode="x unified")

            for data in fig.data:
                data.update(name=plot_items[data.name])

            fig.update_xaxes(
                rangeslider_visible=True,
                rangeselector=dict(
                    buttons=list(
                        [
                            dict(
                                count=1, label="1M", step="month", stepmode="backward"
                            ),
                            dict(
                                count=6, label="6M", step="month", stepmode="backward"
                            ),
                            dict(count=1, label="1Y", step="year", stepmode="backward"),
                            dict(step="all"),
                        ]
                    )
                ),
            )

            fig.write_json(str(output_path / (plot_type + ".json")))
            # fig.write_html(str(output_path / (plot_type + '.html')))


def generate_metrics_graph(
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


def generate_metrics_graph(
    hugo_root_dir: Path,
    data_source_dir: Path,
):
    generate_metrics_graph(hugo_root_dir, read_dataframe(data_source_dir))
