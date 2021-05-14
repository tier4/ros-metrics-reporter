#! /usr/bin/env python3

from pathlib import Path


def dir_path(input):
    if Path(input).is_dir():
        return Path(input)
    else:
        raise NotADirectoryError(input)
