#! /usr/bin/env python3

from os import name
from pandas.core.frame import DataFrame
import plotly.express as px
from pathlib import Path


plot_name_list = [
    {
        "coverage": {
            "Lines": "Line coverage",
            "Functions": "Function coverage",
            "Branches": "Branch coverage",
        }
    },
    {
        "metrics": {
            "CCN(worst)": "CCN (Worst case)",
            "CCN(violate)": "CCN (Violation count)",
            "LOC(worst)": "LOC (Worst case)",
            "LOC(violate)": "LOC (Violation count)",
            "Parameter(worst)": "Parameter count (Worst case)",
            "Parameter(violate)": "Parameter count (Violation count)",
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
