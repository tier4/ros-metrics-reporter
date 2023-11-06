from pathlib import Path

from ros_metrics_reporter.code_activity.git_statistics import *

GIT_WS = "src/example/geometry2"


def test_generate_code_frequency_graph(tmpdir):
    generate_code_frequency_graph(
        git_ws=GIT_WS,
        package_path=Path("tf2"),
        dest=Path(tmpdir),
    )
    assert Path(tmpdir, "code_frequency_graph.json").exists()


def test_get_top3_contributor():
    result = get_top3_contributor(
        git_ws="src/example/geometry2",
        package_path=Path("tf2"),
    )

    assert len(result) == 3
    for item in result:
        assert item["name"] is not None
        assert item["total"] is not None
