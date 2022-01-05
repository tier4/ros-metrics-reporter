from typing import List, Dict
import dataclasses
import enum


class CoverageKeys(enum.Enum):
    Lines = "Lines"
    Functions = "Functions"
    Branches = "Branches"


@dataclasses.dataclass
class Coverage:
    package: str = ""
    label: str = ""
    line: float = 0.0
    function: float = 0.0
    branch: float = 0.0


@dataclasses.dataclass
class Threshold:
    high: float = 0.0
    med: float = 0.0

    def to_dict(self) -> Dict[str, float]:
        return {
            "Coverage(Hi)": self.high,
            "Coverage(Med)": self.med,
        }


@dataclasses.dataclass
class CoverageData:
    coverage: List[Coverage] = dataclasses.field(default_factory=list)
    threshold: Threshold = dataclasses.field(default_factory=Threshold)

    def add_threshold(self, high, med):
        self.threshold.high = high
        self.threshold.med = med

    def add_coverage(self, coverage: Coverage):
        self.coverage.append(coverage)

    def add_coverages(self, coverages: List[Coverage]):
        self.coverage.extend(coverages)
