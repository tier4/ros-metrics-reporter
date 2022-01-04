from typing import List, Dict
from dataclasses import dataclass, field, asdict
import enum
from pathlib import Path
import json


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


@dataclass
class Coverage:
    package: str = ""
    value: List[CoverageValue] = field(default_factory=list)

    def write(self, file: Path):
        with open(file, "w") as f:
            json.dump(asdict(self), f, indent=4)


@dataclass
class Threshold:
    high: float = 0.0
    med: float = 0.0

    def write(self, file: Path):
        with open(file, "w") as f:
            json.dump(asdict(self), f, indent=4)


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
