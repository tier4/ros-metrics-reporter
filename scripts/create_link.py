#! /usr/bin/env python3

from os import path
from pathlib import Path


def create_link(target: Path, link_from: Path):
    # Unlink old symbolic link
    link_from.unlink(missing_ok=True)

    # Create symbolic link
    link_from.symlink_to(target=target, target_is_directory=True)
