from dataclasses import Field, dataclass, field
from typing import List, Dict
from pathlib import Path
import json


@dataclass
class MetricsValue:
    ccn: int = 0
    nloc: int = 0
    parameter: int = 0
    token: int = 0


@dataclass
class FloatMetricsValue:
    ccn: float = 0.0
    nloc: float = 0.0
    parameter: float = 0.0
    token: float = 0.0


@dataclass
class MetricsData:
    package: str = ""
    worst_value: MetricsValue = field(default_factory=MetricsValue)
    average_value: FloatMetricsValue = field(default_factory=FloatMetricsValue)
    over_threshold_count: MetricsValue = field(default_factory=MetricsValue)
    over_recommendation_count: MetricsValue = field(default_factory=MetricsValue)

    def to_dict(self) -> dict:
        return {
            "worst": {
                "CCN": self.worst_value.ccn,
                "NLOC": self.worst_value.nloc,
                "Parameter": self.worst_value.parameter,
                "Token": self.worst_value.token,
            },
            "average": {
                "CCN": self.average_value.ccn,
                "NLOC": self.average_value.nloc,
                "Parameter": self.average_value.parameter,
                "Token": self.average_value.token,
            },
            "over_threshold": {
                "CCN": self.over_threshold_count.ccn,
                "NLOC": self.over_threshold_count.nloc,
                "Parameter": self.over_threshold_count.parameter,
                "Token": self.over_threshold_count.token,
            },
            "over_recommendation": {
                "CCN": self.over_recommendation_count.ccn,
                "NLOC": self.over_recommendation_count.nloc,
                "Parameter": self.over_recommendation_count.parameter,
                "Token": self.over_recommendation_count.token,
            },
        }

    def save_metrics(self, file):
        with open(file, "w") as f:
            f.write(json.dumps(self.to_dict(), indent=4))


@dataclass
class Threshold:
    threshold_value: MetricsValue = field(default_factory=MetricsValue)
    recommendation_value: MetricsValue = field(default_factory=MetricsValue)

    def to_dict(self) -> dict:
        return {
            "threshold": {
                "CCN": self.threshold_value.ccn,
                "NLOC": self.threshold_value.nloc,
                "Parameter": self.threshold_value.parameter,
            },
            "recommendation": {
                "CCN": self.recommendation_value.ccn,
                "NLOC": self.recommendation_value.nloc,
                "Parameter": self.recommendation_value.parameter,
            },
        }

    def save_threshold(self, file):
        with open(file, "w") as f:
            f.write(json.dumps(self.to_dict(), indent=4))


@dataclass
class MetricsDataList:
    metrics_data: List[MetricsData] = field(default_factory=list)
    threshold: Threshold = field(default_factory=Threshold)

    def add_metrics_data(self, metrics_data: MetricsData):
        self.metrics_data.append(metrics_data)

    def add_metrics_data_list(self, metrics_data_list: List[MetricsData]):
        self.metrics_data.extend(metrics_data_list)

    def get_metrics_data(self, package: str) -> MetricsData:
        for metrics in self.metrics_data:
            if metrics.package == package:
                return metrics

        # If not found, create new one
        new_data = MetricsData(package=package)
        self.add_metrics_data(new_data)
        return new_data

    def save_metrics_value(self, output_dir: Path):
        for metrics in self.metrics_data:
            output_json_dir = output_dir / metrics.package
            output_json_dir.mkdir(exist_ok=True, parents=True)
            metrics.save_metrics(output_json_dir / "lizard.json")

    def save_threshold_value(self, output_path: Path):
        self.threshold.save_threshold(output_path)
