import shlex
from pathlib import Path
from codechecker.analyzer.codechecker_analyzer import arg

from ros_metrics_reporter.metrics.lizard_all import lizard_all
from ros_metrics_reporter.metrics.lizard_package import lizard_package
from ros_metrics_reporter.metrics.metrics_data import (
    MetricsDataList,
    MetricsValue,
    Threshold,
)
from ros_metrics_reporter.package_info import PackageInfo
from ros_metrics_reporter.scraping.metrics import scraping
from ros_metrics_reporter.util import run_command
from uuid import uuid4


def clone_lizard(lizard_dir: Path):
    run_command(
        args=shlex.split(
            f"git clone https://github.com/terryyin/lizard.git {str(lizard_dir)}"
        )
    )


class MetricsTaskRunner:
    def __init__(self, args):
        self.base_dir = args.base_dir
        self.html_dir = args.base_dir / str(uuid4())
        self.output_html_dir = self.html_dir / "lizard_result"
        self.output_recommend_html_dir = self.html_dir / "lizard_result_recommend"
        self.exclude = args.exclude
        self.gh_action_dir = args.action_dir
        self.metrics_data = MetricsDataList()
        self.metrics_data.threshold = Threshold(
            threshold_value=MetricsValue(
                ccn=args.ccn,
                nloc=args.nloc,
                parameter=args.arguments,
            ),
            recommendation_value=MetricsValue(
                ccn=args.ccn_recommendation,
                nloc=args.nloc_recommendation,
                parameter=args.arguments_recommendation,
            ),
        )

    def run(self, packages: PackageInfo):

        lizard_dir = self.gh_action_dir / "lizard"
        if not lizard_dir.exists():
            clone_lizard(lizard_dir)

        # Measure code metrics for threshold value
        lizard_all(
            lizard_executable=lizard_dir / "lizard.py",
            base_dir=self.base_dir,
            output_dir=self.output_html_dir,
            threshold=self.metrics_data.threshold.threshold_value,
            exclude=self.exclude,
        )
        lizard_package(
            lizard_executable=lizard_dir / "lizard.py",
            package_info=packages,
            output_dir=self.output_html_dir,
            threshold=self.metrics_data.threshold.threshold_value,
            exclude=self.exclude,
        )

        # Measure code metrics for recommend value
        lizard_all(
            lizard_executable=lizard_dir / "lizard.py",
            base_dir=self.base_dir,
            output_dir=self.output_recommend_html_dir,
            threshold=self.metrics_data.threshold.recommendation_value,
            exclude=self.exclude,
        )
        lizard_package(
            lizard_executable=lizard_dir / "lizard.py",
            package_info=packages,
            output_dir=self.output_recommend_html_dir,
            threshold=self.metrics_data.threshold.recommendation_value,
            exclude=self.exclude,
        )

    def save_metrics_value(self, output_dir: Path):
        self.metrics_data.add_metrics_data_list(
            scraping(
                lizard_dir=self.output_html_dir,
                lizard_recommendation_dir=self.output_recommend_html_dir,
            )
        )

        self.metrics_data.save_metrics_value(output_dir)

        # Save threshold value
        self.metrics_data.save_threshold_value(output_dir / "metrics_threshold.json")
