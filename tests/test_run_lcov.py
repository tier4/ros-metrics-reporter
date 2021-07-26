from pathlib import Path
from scripts.run_lcov import concat_build_dir, concat_output_path, get_file_size


def test_get_file_size(tmp_path):
    test_file = Path(tmp_path, "foo.txt")
    test_file.touch()
    assert get_file_size(test_file) == 0

    test_file.write_text("foo")
    assert get_file_size(test_file) == 3

    assert get_file_size(Path(tmp_path, "bar.txt")) == -1


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
