from pathlib import Path
from typing import List, Dict

from ros_metrics_reporter.package_info import PackageInfo
import ros_metrics_reporter.code_activity.github_statistics as github_statistics
import ros_metrics_reporter.code_activity.git_statistics as git_statistics


def code_activity(
    github_target_repo: str,
    package_info: PackageInfo,
    code_frequency_graph_output_dir: Path,
    github_access_token: str,
) -> Dict[str, List]:
    # generate code statistics for whole repo
    repo_graph_output_dir = code_frequency_graph_output_dir / "all"
    repo_graph_output_dir.mkdir(parents=True, exist_ok=True)
    github_statistics.generate_code_frequency_graph(
        github_target_repo, repo_graph_output_dir, github_access_token
    )
    contributors = {}
    contributors["all"] = github_statistics.get_top3_contributor(
        github_target_repo, github_access_token
    )

    # generate code statistics for each package
    for package in package_info:
        graph_output_dir = code_frequency_graph_output_dir / package.name
        graph_output_dir.mkdir(parents=True, exist_ok=True)
        git_statistics.generate_code_frequency_graph(
            package_info.ros_ws, package.path, graph_output_dir
        )
        contributors[package.name] = git_statistics.get_top3_contributor(
            package_info.ros_ws, package.path
        )
        print(f"Top3 contributor of {package.name}")
        print(str(contributors[package.name]))
    return contributors
