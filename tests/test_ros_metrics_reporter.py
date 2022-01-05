from ros_metrics_reporter.ros_metrics_reporter import *
import pytest


def test_dir_path(tmpdir):
    dir = Path(tmpdir, "foo")
    dir.mkdir(parents=True)
    assert dir == dir_path(str(dir))

    not_a_dir = Path(tmpdir, "bar")
    with pytest.raises(NotADirectoryError):
        dir_path(str(not_a_dir))


def test_space_separated_string():
    test_input = "foo bar baz"
    assert ["foo", "bar", "baz"] == space_separated_string(test_input)
