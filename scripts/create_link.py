#! /usr/bin/env python3

from os import path
from pathlib import Path
import shutil


def create_link(target: Path, link_from: Path):
    # Unlink old symbolic link
    if link_from.is_dir():
        shutil.rmtree(link_from)
    if link_from.exists():
        link_from.unlink(missing_ok=True)

    # Create symbolic link
    link_from.symlink_to(target=target, target_is_directory=True)
