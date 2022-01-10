#! /usr/bin/env python3

from pathlib import Path
import subprocess
from subprocess import CalledProcessError
import sys
from typing import List
import fnmatch
import shutil
import uuid

from jinja2 import Environment, FileSystemLoader
from jinja2.environment import Template


class DirectoryBackup:
    def __init__(self, dir_path: Path):
        self.orig_path = dir_path
        self.backup_path = dir_path.with_suffix("." + str(uuid.uuid4()))

    def __del__(self):
        if self.backup_path.exists():
            shutil.rmtree(self.backup_path)

    def backup(self):
        shutil.copytree(self.orig_path, self.backup_path)

    def restore(self):
        shutil.rmtree(self.orig_path, ignore_errors=True)
        self.backup_path.rename(self.orig_path)


def path_match(target_path: str, pattern_list: List[str]) -> bool:
    matched = [
        pattern for pattern in pattern_list if fnmatch.fnmatch(target_path, pattern)
    ]
    return True if matched else False


def run_command(args: List[str], cwd: Path = None) -> bool:
    try:
        subprocess.run(
            args=args, check=True, stdout=sys.stdout, stderr=sys.stderr, cwd=cwd
        )
    except CalledProcessError:
        return False
    return True


def run_command_pipe(args: List[str], cwd: Path = None) -> str:
    proc = subprocess.run(
        args=args, check=True, stdout=subprocess.PIPE, encoding="utf-8", cwd=cwd
    )
    return proc.stdout


def run_command_redirect(args: list, output_file: Path, cwd: Path = None) -> bool:
    try:
        with open(output_file, "w") as out:
            subprocess.run(args=args, check=True, stdout=out, cwd=cwd)
    except CalledProcessError:
        return False
    return True


def read_jinja2_template(
    file: Path, variable_start_string: str = "[[", variable_end_string: str = "]]"
) -> Template:
    """Read jinja2 Template"""
    env = Environment(
        loader=FileSystemLoader(str(file.parent)),
        variable_start_string=variable_start_string,
        variable_end_string=variable_end_string,
    )
    return env.get_template(file.name)
