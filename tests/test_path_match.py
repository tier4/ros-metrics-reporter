from pathlib import Path
from scripts.path_match import path_match


def test_path_match_true():
    pattern = [
        "**/foo/*",
        "**/bar/*",
    ]
    assert True == path_match("/home/user/project/foo/", pattern)


def test_path_match_false():
    pattern = [
        "**/foo/*",
        "**/bar/*",
    ]
    assert False == path_match("/home/user/project/foolish/", pattern)
