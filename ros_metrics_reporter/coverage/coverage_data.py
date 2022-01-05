from __future__ import annotations
from typing import List, Set
from dataclasses import dataclass, field, asdict
import enum
from pathlib import Path
import json
from ros_metrics_reporter.color import Color
import pandas as pd
from datetime import datetime


class CoverageKeys(enum.Enum):
    Lines = "Lines"
    Functions = "Functions"
    Branches = "Branches"


@dataclass
class CoverageValue:
    label: str = ""
    line: float = 0.0
    function: float = 0.0
    branch: float = 0.0

    def get(self, coverage_key: CoverageKeys):
        if coverage_key == CoverageKeys.Lines:
            return self.line
        elif coverage_key == CoverageKeys.Functions:
            return self.function
        elif coverage_key == CoverageKeys.Branches:
            return self.branch
        else:
            raise Exception("Unknown coverage key")


@dataclass
class Coverage:
    package: str = ""
    value: List[CoverageValue] = field(default_factory=list)

    def write(self, file: Path):
        with open(file, "w") as f:
            json.dump(asdict(self), f, indent=2)

    def read(self, file: Path) -> Coverage:
        with open(file, "r") as f:
            data = json.load(f)
            self.package = data["package"]
            self.value = [CoverageValue(**value) for value in data["value"]]
        return self

    def get_label_value(self, label: str) -> CoverageValue:
        for value in self.value:
            if value.label == label:
                return value
        return CoverageValue(label=label)

    def get_labels(self) -> Set[str]:
        return {value.label for value in self.value}


@dataclass
class CoverageStamped(Coverage):
    date: datetime = field(default_factory=datetime.now)


@dataclass
class Threshold:
    high: float = 0.0
    med: float = 0.0

    def write(self, file: Path):
        with open(file, "w") as f:
            json.dump(asdict(self), f, indent=2)


@dataclass
class CoverageData:
    coverage: List[Coverage] = field(default_factory=list)
    threshold: Threshold = field(default_factory=Threshold)

    def add_threshold(self, high, med):
        self.threshold.high = high
        self.threshold.med = med

    def add_coverage(self, coverage: Coverage):
        for i, item in enumerate(self.coverage):
            if item.package == coverage.package:
                self.coverage[i].value.extend(coverage.value)
                return
        self.coverage.append(coverage)

    def add_coverages(self, coverages: List[Coverage]):
        for coverage in coverages:
            self.add_coverage(coverage)

    def save_coverage(self, output_dir: Path):
        for item in self.coverage:
            output_json_dir = output_dir / item.package
            output_json_dir.mkdir(parents=True, exist_ok=True)
            item.write(output_json_dir / "coverage.json")

    def save_threshold_value(self, output_path: Path):
        self.threshold.write(output_path)

    def get_coverage(self, package: str) -> Coverage:
        for item in self.coverage:
            if item.package == package:
                return item
        return Coverage(package=package)

    def get_color(self, value: str, coverage_key: CoverageKeys) -> Color:
        if coverage_key == CoverageKeys.Lines:
            if value >= self.threshold.high:
                return Color.GREEN
            elif value >= self.threshold.med:
                return Color.YELLOW
            else:
                return Color.RED
        elif coverage_key == CoverageKeys.Functions:
            if value >= self.threshold.high:
                return Color.GREEN
            elif value >= self.threshold.med:
                return Color.YELLOW
            else:
                return Color.RED
        elif coverage_key == CoverageKeys.Branches:
            if value >= self.threshold.high:
                return Color.GREEN
            elif value >= self.threshold.med:
                return Color.YELLOW
            else:
                return Color.RED
        else:
            raise Exception("Unknown coverage key")
