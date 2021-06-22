#! /usr/bin/env python3

import sys
import re
import fnmatch

target_path = sys.argv[1]

for i in range(2, len(sys.argv)):
    if fnmatch.fnmatch(target_path, sys.argv[i]):
        print("0")
        sys.exit(0)
print("1")
