#! /usr/bin/env python3

from util import run_command
import shlex
from pathlib import Path
import shutil


def backup_build_artifacts():
    # Move directory to build_base, install_base
    build_base = Path("build_base")
    install_base = Path("install_base")

    if build_base.exists():
        shutil.rmtree(build_base)
    build_base.mkdir()
    for item in Path("build").glob("**/*"):
        shutil.move(str(item), build_base)

    if install_base.exists():
        shutil.rmtree(install_base)
    install_base.mkdir()
    for item in Path("install").glob("**/*"):
        shutil.move(str(item), install_base)


def clear_build_directory():
    if Path("build").exists():
        shutil.rmtree("build")

    if Path("install").exists():
        shutil.rmtree("install")


def colcon_build():
    COVERAGE_FLAGS = "-fprofile-arcs -ftest-coverage -DCOVERAGE_RUN=1 -O0"

    run_command(
        args=shlex.split(
            'colcon build \
            --event-handlers console_cohesion+ \
            --cmake-args -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_CXX_FLAGS="{0}" -DCMAKE_C_FLAGS="{0}" \
            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'.format(
                COVERAGE_FLAGS
            )
        ),
    )

    run_command(
        args=shlex.split(
            "colcon test \
            --event-handlers console_cohesion+ \
            --return-code-on-test-failure"
        )
    )

    backup_build_artifacts()


def colcon_get_packages(package_name: str):
    clear_build_directory()
    shutil.copytree("build_base/" + package_name, "build/" + package_name)
    shutil.copytree("install_base/" + package_name, "install/" + package_name)
    shutil.copy2(
        "build/" + package_name + "/compile_commands.json",
        "build/compile_commands.json",
    )


def colcon_get_all_packages():
    clear_build_directory()
    shutil.copytree("build_base/", "build/")
    shutil.copytree("install_base/", "install/")
