from github import Github
import plotly.express as px
from pathlib import Path
from typing import List, Dict


def generate_code_frequency_graph(repo: str, token: str, dest: Path):
    gh = Github(token)
    repo = gh.get_repo(repo)

    stats = repo.get_stats_commit_activity()
    weeks = [item.week for item in stats]
    total = [x.total for x in stats]

    fig = px.area(x=weeks, y=total, labels={"x":"Date", "y":"Commits"}, title="Commits per Week")
    fig.write_json(dest)


def get_top3_contributor(repo: str, token: str) -> List[Dict]:
    gh = Github(token)
    repo = gh.get_repo(repo)

    ret = []
    stats = repo.get_stats_contributors()
    for item in sorted(stats, key=lambda x: x.total, reverse=True)[:3]:
        ret.append({
            "name": item.author.login,
            "avatar": item.author.avatar_url,
            "total": item.total
        })
    return ret
