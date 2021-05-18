#! /usr/bin/env python3

from pandas.core.frame import DataFrame
import plotly.express as px
from pathlib import Path


plot_name_list = [
    {
        "coverage": [
            "Lines",
            "Functions",
            "Branches",
        ]
    },
    {
        "metrics": [
            "CCN(worst)",
            "CCN(violate)",
            "LOC(worst)",
            "LOC(violate)",
            "Parameter(worst)",
            "Parameter(violate)",
        ]
    },
]


def plot_timeseries(df: DataFrame, output_path: Path):
    """Plot time-series and write to json"""
    for plot_group in plot_name_list:
        for plot_type, plot_items in plot_group.items():

            df_filtered = df.query("type in @plot_items")
            if df_filtered.empty:
                continue

            fig = px.line(
                df_filtered,
                x="date",
                y="value",
                title="Time Series with Range Slider and Selectors",
                color="type",
            )

            fig.update_xaxes(
                rangeslider_visible=True,
                rangeselector=dict(
                    buttons=list(
                        [
                            dict(
                                count=1, label="1m", step="month", stepmode="backward"
                            ),
                            dict(
                                count=6, label="6m", step="month", stepmode="backward"
                            ),
                            dict(count=1, label="YTD", step="year", stepmode="todate"),
                            dict(count=1, label="1y", step="year", stepmode="backward"),
                            dict(step="all"),
                        ]
                    )
                ),
            )

            fig.write_json(str(output_path / (plot_type + ".json")))
            # fig.write_html(str(output_path / (plot_type + '.html')))
