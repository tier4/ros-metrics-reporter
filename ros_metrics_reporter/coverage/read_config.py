from ros_metrics_reporter.coverage.coverage_data import Threshold
from pathlib import Path


def read_lcovrc(lcovrc: Path) -> Threshold:
    lcov_threshold = Threshold()
    with open(lcovrc) as f:
        for line in f:
            # Skip comment lines
            if "#" == line[0]:
                continue

            if "genhtml_hi_limit" in line:
                lcov_threshold.high = float(line.split()[-1])
            elif "genhtml_med_limit" in line:
                lcov_threshold.med = float(line.split()[-1])
    return lcov_threshold
