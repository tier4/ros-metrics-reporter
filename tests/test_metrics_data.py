from ros_metrics_reporter.metrics.metrics_data import *


def test_metrics_data_io(tmp_path):
    data = MetricsData(package="foo")
    data_path = tmp_path / "data.json"
    data.write(data_path)
    data_loaded = MetricsData().read(data_path)
    assert data == data_loaded
