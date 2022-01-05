from dataclasses import dataclass
from pathlib import Path
from typing import List, Dict

from ros_metrics_reporter.code_coverage_task_runner import CodeCoverageTaskRunner
from ros_metrics_reporter.metrics_task_runner import MetricsTaskRunner
from ros_metrics_reporter.package_info import PackageInfo


@dataclass
class StaticPageInput:
    input_dir: Path
    hugo_root_dir: Path
    hugo_template_dir: Path
    tidy_result_path: Path
    packages: PackageInfo
    base_url: str
    title: str
    contributors: Dict[str, List]
    code_coverage: CodeCoverageTaskRunner
    metrics: MetricsTaskRunner
