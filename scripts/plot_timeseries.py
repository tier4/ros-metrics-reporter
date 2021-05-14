#! /usr/bin/env python3

from pandas.core.frame import DataFrame
import plotly.express as px
from pathlib import Path


plot_name_list = [
    "Lines Functions",
    "Branches CCN(worst)",
    "CCN(violate)",
    "LOC(worst)",
    "LOC(violate)",
    "Parameter(worst)",
    "Parameter(violate)",
]


def column_exists(df: DataFrame, names: list) -> bool:
    """Return True when all names are found in columns"""
    for name in names:
        if not name in df.columns.values:
            return False
    return True


def plot_timeseries(df: DataFrame, output_path: Path):
    """Plot time-series and write to json"""
    for plot_name in plot_name_list:
        if not column_exists(df, [plot_name]):
            continue

        fig = px.line(
            df,
            x="date",
            y=plot_name,
            title="Time Series with Range Slider and Selectors",
        )

        fig.update_xaxes(
            rangeslider_visible=True,
            rangeselector=dict(
                buttons=list(
                    [
                        dict(count=1, label="1m", step="month", stepmode="backward"),
                        dict(count=6, label="6m", step="month", stepmode="backward"),
                        dict(count=1, label="YTD", step="year", stepmode="todate"),
                        dict(count=1, label="1y", step="year", stepmode="backward"),
                        dict(step="all"),
                    ]
                )
            ),
        )

        file_name = plot_name.replace("(", "_").replace(")", "")
        fig.write_json(str(output_path / (file_name + ".json")))

        # fig.write_html(str(output_path / (file_name + '.html')))
