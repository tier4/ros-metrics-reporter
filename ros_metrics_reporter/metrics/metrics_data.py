from __future__ import annotations
from dataclasses import dataclass, field, asdict
from typing import List
from pathlib import Path
import json
import enum
from ros_metrics_reporter.color import Color
from datetime import datetime


class MetricsValueKeys(enum.Enum):
    CCN = "ccn"
    NLOC = "nloc"
    PARAMETER = "parameter"
    TOKEN = "token"


@dataclass
class MetricsValue:
    ccn: int = 0
    nloc: int = 0
    parameter: int = 0
    token: int = 0

    def get(self, metrics_value_key: MetricsValueKeys) -> int:
        if metrics_value_key == MetricsValueKeys.CCN:
            return self.ccn
        elif metrics_value_key == MetricsValueKeys.NLOC:
            return self.nloc
        elif metrics_value_key == MetricsValueKeys.PARAMETER:
            return self.parameter
        elif metrics_value_key == MetricsValueKeys.TOKEN:
            return self.token
        else:
            raise Exception("Unknown metrics value key")


@dataclass
class FloatMetricsValue:
    ccn: float = 0.0
    nloc: float = 0.0
    parameter: float = 0.0
    token: float = 0.0

    def get(self, metrics_value_key: MetricsValueKeys) -> float:
        if metrics_value_key == MetricsValueKeys.CCN:
            return self.ccn
        elif metrics_value_key == MetricsValueKeys.NLOC:
            return self.nloc
        elif metrics_value_key == MetricsValueKeys.PARAMETER:
            return self.parameter
        elif metrics_value_key == MetricsValueKeys.TOKEN:
            return self.token
        else:
            raise Exception("Unknown metrics value key")


class MetricsDataKeys(enum.Enum):
    Worst = "worst_value"
    Average = "average_value"
    OverThresholdCount = "over_threshold_count"
    OverRecommendationCount = "over_recommendation_count"


@dataclass
class MetricsData:
    package: str = ""
    worst_value: MetricsValue = field(default_factory=MetricsValue)
    average_value: FloatMetricsValue = field(default_factory=FloatMetricsValue)
    over_threshold_count: MetricsValue = field(default_factory=MetricsValue)
    over_recommendation_count: MetricsValue = field(default_factory=MetricsValue)

    def write(self, file):
        with open(file, "w") as f:
            json.dump(asdict(self), f, indent=2)

    def read(self, file) -> MetricsData:
        with open(file, "r") as f:
            data = json.load(f)
            self.package = data["package"]
            self.worst_value = MetricsValue(**data["worst_value"])
            self.average_value = FloatMetricsValue(**data["average_value"])
            self.over_threshold_count = MetricsValue(**data["over_threshold_count"])
            self.over_recommendation_count = MetricsValue(
                **data["over_recommendation_count"]
            )
        return self

    def get_value(self, metrics_key: MetricsDataKeys):
        if metrics_key == MetricsDataKeys.Worst:
            return self.worst_value
        elif metrics_key == MetricsDataKeys.Average:
            return self.average_value
        elif metrics_key == MetricsDataKeys.OverThresholdCount:
            return self.over_threshold_count
        elif metrics_key == MetricsDataKeys.OverRecommendationCount:
            return self.over_recommendation_count
        else:
            raise Exception("Unknown metrics key")


@dataclass
class MetricsDataStamped(MetricsData):
    date: datetime = field(default_factory=datetime.now)


@dataclass
class Threshold:
    threshold_value: MetricsValue = field(default_factory=MetricsValue)
    recommendation_value: MetricsValue = field(default_factory=MetricsValue)

    def save_threshold(self, file):
        with open(file, "w") as f:
            json.dump(asdict(self), f, indent=2)

    def get_color(
        self,
        value: float,
        value_type: MetricsDataKeys,
        metrics_value_key: MetricsValueKeys,
    ) -> Color:
        threshold = self.threshold_value.get(metrics_value_key)
        recommendation = self.recommendation_value.get(metrics_value_key)

        if value_type == MetricsDataKeys.Worst:
            if value > threshold:
                return Color.RED
            elif value > recommendation:
                return Color.YELLOW
            else:
                return Color.GREEN
        else:
            if value == 0:
                return Color.GREEN
            else:
                return Color.RED


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
            metrics.write(output_json_dir / "lizard.json")

    def save_threshold_value(self, output_path: Path):
        self.threshold.save_threshold(output_path)
