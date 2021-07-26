from pathlib import Path
from path_match import path_match


def test_path_match_true():
    pattern = [
        "**/foo/*",
        "**/bar/*",
    ]
    assert True == path_match("/home/user/project/foo/", pattern)
    assert True == path_match("/home/user/project/bar/", pattern)
    assert True == path_match("/home/user/project/foo/bar/", pattern)


def test_path_match_false():
    pattern = [
        "**/foo/*",
        "**/bar/*",
    ]
    assert False == path_match("/home/user/project/foolish/", pattern)
