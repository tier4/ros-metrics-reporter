from github import Github
import plotly.express as px
from pathlib import Path
from typing import List, Dict


def generate_code_frequency_graph(repo: str, dest: Path, token: str = None) -> None:
    gh = Github(token)
    repo = gh.get_repo(repo)

    stats = repo.get_stats_commit_activity()
    weeks = [x.week for x in stats]
    total = [x.total for x in stats]

    fig = px.area(
        x=weeks, y=total, labels={"x": "Date", "y": "Commits"}, title="Commits per Week"
    )
    fig.write_json(dest)


def get_top3_contributor(repo: str, token: str = None) -> List[Dict]:
    gh = Github(token)
    repo = gh.get_repo(repo)

    stats = repo.get_stats_contributors()
    top3_contributor = [
        x for x in sorted(stats, key=lambda x: x.total, reverse=True)[:3]
    ]
    return [
        {
            "name": x.author.login,
            "avatar": x.author.avatar_url,
            "total": x.total,
        }
        for x in top3_contributor
    ]
