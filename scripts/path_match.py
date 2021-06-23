#! /usr/bin/env python3

import fnmatch


def path_match(target_path: str, pattern_list: list) -> bool:
    for pattern in pattern_list:
        if fnmatch.fnmatch(target_path, pattern):
            return True
    return False
