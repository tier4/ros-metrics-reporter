from pathlib import Path

from ros_metrics_reporter.graph.plot_coverage import plot_coverage
from ros_metrics_reporter.graph.plot_metrics import plot_metrics


def generate_metrics_graph(
    hugo_root_dir: Path,
    data_source_dir: Path,
):
    # Create graph
    plotly_output_dir = hugo_root_dir / "static" / "plotly"
    plotly_output_dir.mkdir(parents=True, exist_ok=True)

    plot_coverage(data_source_dir, plotly_output_dir)
    plot_metrics(data_source_dir, plotly_output_dir)
