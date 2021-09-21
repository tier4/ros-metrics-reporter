from github import Github
import plotly.express as px
from pathlib import Path
from typing import List, Dict


def check_token(token):
    if token and len(token) > 0:
        return token
    else:
        return None


def generate_code_frequency_graph(repo: str, dest: Path, token: str = None) -> None:
    gh = Github(check_token(token))
    gh_repo = gh.get_repo(repo)

    stats = gh_repo.get_stats_contributors()
    total = {}
    for stat in stats:
        for week in stat.weeks:
            if week.w not in total:
                total[week.w] = 0
            else:
                total[week.w] += week.c
    weeks = [str(week.w) for week in stats[0].weeks]

    fig = px.area(
        x=weeks, y=total, labels={"x": "Date", "y": "Commits"}, title="Commits per Week"
    )

    if dest.is_dir():
        dest = dest / "code_frequency_graph.json"

    fig.write_json(str(dest))


def get_top3_contributor(repo: str, token: str = None) -> List[Dict]:
    gh = Github(check_token(token))
    gh_repo = gh.get_repo(repo)

    stats = gh_repo.get_stats_contributors()
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
