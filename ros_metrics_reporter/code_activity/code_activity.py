from pathlib import Path
from typing import List, Dict

from ros_metrics_reporter.package_info import PackageInfo
import ros_metrics_reporter.code_activity.github_statistics as github_statistics
import ros_metrics_reporter.code_activity.git_statistics as git_statistics


def _find_git_ws(path: Path) -> Path:
    dot_git = path / ".git"
    if dot_git.is_dir():
        print(f"Detected git workspace: {path}")
        return path
    else:
        if path.parent == path:
            raise FileNotFoundError(f"Cannot find .git directory in {path}")
        else:
            return _find_git_ws(path.parent)


def _get_package_relative_path(base_path: Path, package_path: Path) -> Path:
    """
    Get the relative path from base_path to package_path
    """
    return package_path.relative_to(base_path)


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
        git_ws = _find_git_ws(package_info.ros_ws / package.path)
        relative_package_path = _get_package_relative_path(git_ws, package_path)
        print(
            f"[DEBUG] package {package.name}, git workspace: {git_ws}, path: {relative_package_path}"
        )
        graph_output_dir = code_frequency_graph_output_dir / package.name
        graph_output_dir.mkdir(parents=True, exist_ok=True)
        git_statistics.generate_code_frequency_graph(
            git_ws, relative_package_path, graph_output_dir
        )
        contributors[package.name] = git_statistics.get_top3_contributor(
            git_ws, relative_package_path
        )
    return contributors
