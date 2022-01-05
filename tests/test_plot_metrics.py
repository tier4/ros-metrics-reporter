import os
from pathlib import Path
from ros_metrics_reporter.graph.plot_metrics import plot_metrics


def test_plot_metrics(tmp_path):
    metrics_path = Path(os.path.dirname(__file__), "resource", "metrics")
    plot_metrics(metrics_path, tmp_path / "plots")
    assert (tmp_path / "plots").exists()
