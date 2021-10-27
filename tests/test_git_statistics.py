from pathlib import Path

from ros_metrics_reporter.git_statistics import *


def test_generate_code_frequency_graph(tmpdir):
    generate_code_frequency_graph(
        git_ws="example/src/demos",
        package_path=Path("demo_nodes_cpp"),
        dest=Path(tmpdir),
    )
    assert Path(tmpdir, "code_frequency_graph.json").exists()


def test_get_top3_contributor():
    result = get_top3_contributor(
        git_ws="example/src/demos",
        package_path=Path("demo_nodes_cpp"),
    )

    assert len(result) == 3
    for item in result:
        assert item["name"] is not None
        assert item["total"] is not None
