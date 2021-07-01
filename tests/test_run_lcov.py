from pathlib import Path
from scripts.run_lcov import concat_build_dir, concat_output_path


def test_concat_build_dir():
    foo_bar_str = concat_build_dir(Path("foo", "bar"), "")
    assert foo_bar_str == '"foo/bar/build"'

    foo_bar_baz_str = concat_build_dir(Path("foo", "bar"), "baz")
    assert foo_bar_baz_str == '"foo/bar/build/baz"'


def test_concat_output_path():
    dir = Path("foo", "bar")
    filename = "baz"
    foo_bar_baz = concat_output_path(dir, filename)
    assert '"foo/bar/baz"' == foo_bar_baz
