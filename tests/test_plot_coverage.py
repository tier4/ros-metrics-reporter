import os
from pathlib import Path
from ros_metrics_reporter.graph.plot_coverage import plot_coverage


def test_plot_coverage(tmp_path):
    coverage_path = Path(os.path.dirname(__file__), "resource", "metrics")
    plot_coverage(coverage_path, tmp_path / "plots")
    assert (tmp_path / "plots").exists()
