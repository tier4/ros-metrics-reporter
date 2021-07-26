import pytest
import tempfile
from util import *


def test_dir_path_dir_exists():
    tmpdir = tempfile.TemporaryDirectory()
    assert Path(tmpdir.name) == dir_path(tmpdir.name)


def test_dir_path_dir_not_exists():
    with pytest.raises(NotADirectoryError):
        dir_path("not/a/dir/")
