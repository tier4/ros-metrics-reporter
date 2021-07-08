#! /usr/bin/env python3

import fnmatch
from typing import List


def path_match(target_path: str, pattern_list: List[str]) -> bool:
    for pattern in pattern_list:
        if fnmatch.fnmatch(target_path, pattern):
            return True
    return False
