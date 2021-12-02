from pathlib import Path
from typing import List, Dict
import shlex
import collections
import plotly.express as px
import pandas as pd

from ros_metrics_reporter.util import run_command_pipe


def generate_code_frequency_graph(git_ws: Path, package_path: Path, dest: Path) -> None:
    git_log = run_command_pipe(
        args=shlex.split(f'git log --follow --pretty=format:"%as" {package_path}'),
        cwd=git_ws,
    ).splitlines()

    print(git_log)

    df = pd.DataFrame({"date": git_log})
    df["date"] = pd.to_datetime(df["date"])
    df["week"] = df["date"].dt.to_period("W").dt.to_timestamp()
    weekly_commits = df["week"].value_counts()

    fig = px.area(
        data_frame=weekly_commits,
        labels={"x": "Date", "y": "Commits"},
        title="Commits per Week",
    )

    if dest.is_dir():
        dest = dest / "code_frequency_graph.json"

    fig.write_json(str(dest))


def get_top3_contributor(git_ws: Path, package_path: Path) -> List[Dict]:
    git_log = run_command_pipe(
        args=shlex.split(f'git log --follow --pretty=format:"%an" {package_path}'),
        cwd=git_ws,
    ).splitlines()
    top3_contributor = collections.Counter(git_log).most_common()[:3]

    print(git_log)

    return [
        {
            "name": x[0],
            "total": x[1],
        }
        for x in top3_contributor
    ]
