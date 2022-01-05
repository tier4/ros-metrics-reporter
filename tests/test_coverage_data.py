from ros_metrics_reporter.coverage.coverage_data import *


def test_coverage_data_io(tmp_path):
    data = Coverage(package="foo")
    data_path = tmp_path / "data.json"
    data.write(data_path)
    data_loaded = Coverage().read(data_path)
    assert data == data_loaded
