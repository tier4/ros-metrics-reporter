#! /usr/bin/env python3

from os import path
from pathlib import Path
import shutil


def create_link(target: Path, link_from: Path):
    # Unlink old symbolic link
    if link_from.exists():
        if link_from.is_dir():
            if path.islink(link_from):
                link_from.unlink()
            else:
                shutil.rmtree(link_from)
        else:
            if path.islink(link_from):
                link_from.unlink()
            else:
                shutil.rmtree(link_from)

    # Create symbolic link
    link_from.symlink_to(target=target, target_is_directory=True)
