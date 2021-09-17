#! /usr/bin/env python3

from pathlib import Path
import subprocess
from subprocess import CalledProcessError
import sys
from typing import List


def dir_path(input):
    if Path(input).is_dir():
        return Path(input)
    else:
        raise NotADirectoryError(input)


def run_command(args: List[str], cwd: Path = None) -> bool:
    try:
        subprocess.run(
            args=args, check=True, stdout=sys.stdout, stderr=sys.stderr, cwd=cwd
        )
    except CalledProcessError:
        return False
    return True


def run_command_pipe(args: List[str], cwd: Path = None) -> str:
    try:
        proc = subprocess.run(
            args=args, check=True, capture_output=True, text=True, cwd=cwd
        )
        return proc.stdout
    except CalledProcessError:
        return ""


def run_command_redirect(args: list, output_file: Path, cwd: Path = None) -> bool:
    try:
        with open(output_file, "w") as out:
            subprocess.run(args=args, check=True, stdout=out, cwd=cwd)
    except CalledProcessError:
        return False
    return True
