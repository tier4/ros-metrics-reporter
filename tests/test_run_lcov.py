from pathlib import Path

from ros_metrics_reporter.coverage.run_lcov import *

GIT_WS = "src/example/geometry2"


def test_concat_output_path():
    dir = Path("foo", "bar")
    filename = "baz"
    foo_bar_baz = concat_output_path(dir, filename)
    assert '"foo/bar/baz"' == foo_bar_baz
