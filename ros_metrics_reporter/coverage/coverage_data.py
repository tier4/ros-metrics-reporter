from typing import List, Dict
from dataclasses import dataclass, field
import enum
from pathlib import Path
import json


class CoverageKeys(enum.Enum):
    Lines = "Lines"
    Functions = "Functions"
    Branches = "Branches"


@dataclass
class Coverage:
    package: str = ""
    label: str = ""
    line: float = 0.0
    function: float = 0.0
    branch: float = 0.0


@dataclass
class Threshold:
    high: float = 0.0
    med: float = 0.0

    def to_dict(self) -> Dict[str, float]:
        return {
            "Coverage(Hi)": self.high,
            "Coverage(Med)": self.med,
        }

    def write(self, file: Path):
        with open(file, "w") as f:
            f.write(json.dumps(self.to_dict(), indent=4))


@dataclass
class CoverageData:
    coverage: List[Coverage] = field(default_factory=list)
    threshold: Threshold = field(default_factory=Threshold)

    def add_threshold(self, high, med):
        self.threshold.high = high
        self.threshold.med = med

    def add_coverage(self, coverage: Coverage):
        self.coverage.append(coverage)

    def add_coverages(self, coverages: List[Coverage]):
        self.coverage.extend(coverages)
