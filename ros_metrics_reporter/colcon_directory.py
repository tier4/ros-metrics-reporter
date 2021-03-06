#! /usr/bin/env python3

import shlex
from pathlib import Path
import shutil

from ros_metrics_reporter.util import run_command


def backup_build_artifacts(target_path: Path):
    # Move directory to build_base, install_base
    build_base = target_path / "build_base"
    install_base = target_path / "install_base"

    if build_base.exists():
        shutil.rmtree(build_base)
    build_base.mkdir()
    for item in Path(target_path, "build").glob("**/*"):
        shutil.move(str(item), build_base)

    if install_base.exists():
        shutil.rmtree(install_base)
    install_base.mkdir()
    for item in Path(target_path, "install").glob("**/*"):
        shutil.move(str(item), install_base)


def clear_build_directory(target_path: Path):
    if Path(target_path / "build").exists():
        shutil.rmtree(target_path / "build")

    if Path(target_path / "install").exists():
        shutil.rmtree(target_path / "install")


def colcon_get_package(target_path: Path, package_name: str) -> bool:
    try:
        clear_build_directory(target_path)
        shutil.copytree(
            target_path / "build_base" / package_name,
            target_path / "build" / package_name,
        )
        shutil.copytree(
            target_path / "install_base" / package_name,
            target_path / "install" / package_name,
        )
        shutil.copy2(
            target_path / "build" / package_name / "compile_commands.json",
            target_path / "build" / "compile_commands.json",
        )
    except (FileNotFoundError, OSError):
        return False
    return True


def colcon_get_all_packages(target_path: Path):
    clear_build_directory(target_path)
    shutil.copytree(target_path / "build_base", target_path / "build")
    shutil.copytree(target_path / "install_base", target_path / "install")


class Colcon:
    def __init__(self, target_path: Path):
        self.target_path = target_path
        self.__build_success = False
        self.__test_success = False

    def is_build_success(self) -> bool:
        return self.__build_success

    def is_test_success(self) -> bool:
        return self.__test_success

    def build(self, extra_cmake_args: str = ""):
        COVERAGE_FLAGS = "-fprofile-arcs -ftest-coverage -DCOVERAGE_RUN=1"

        if run_command(
            args=shlex.split(
                f'colcon build \
                --event-handlers console_cohesion+ \
                --cmake-args {extra_cmake_args} -DCMAKE_BUILD_TYPE=Debug \
                -DCMAKE_CXX_FLAGS="{COVERAGE_FLAGS}" -DCMAKE_C_FLAGS="{COVERAGE_FLAGS}" \
                -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
            ),
            cwd=self.target_path,
        ):
            self.__build_success = True
        backup_build_artifacts(self.target_path)

    def test(self):
        if not self.__build_success:
            self.__test_success = False
            return

        colcon_get_all_packages(self.target_path)

        if run_command(
            args=shlex.split(
                "colcon test \
                --event-handlers console_cohesion+ \
                --return-code-on-test-failure"
            ),
            cwd=self.target_path,
        ):
            self.__test_success = True

        backup_build_artifacts(self.target_path)
