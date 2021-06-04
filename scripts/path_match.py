#! /usr/bin/env python3

import sys
import re
import fnmatch

print(len(sys.argv))

target_path = sys.argv[1]

for i in range(2, len(sys.argv)):
    if fnmatch.fnmatch(target_path, sys.argv[i]):
        sys.exit(0)
sys.exit(1)
