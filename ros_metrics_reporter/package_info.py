from pathlib import Path
from typing import List

from ros_metrics_reporter.util import run_command_pipe


class Package:
    """
    Returns the package info
    """

    name: str
    path: Path
    type: str

    def __init__(self, name, path, package_type):
        self.name = name
        self.path = path
        self.type = package_type


class PackageInfo:
    """
    Provide a list of packages
    """

    ros_ws: Path
    package_list: List[Package]

    def __init__(self, ros_ws: Path):
        self.ros_ws = ros_ws.absolute()
        package_list = run_command_pipe(["colcon", "list"], cwd=ros_ws).splitlines()
        self.package_list = []
        for line in package_list:
            package_name, package_path, package_type = line.split()
            self.__check_package_full_path(ros_ws, package_path)
            self.package_list.append(
                Package(package_name, Path(package_path), package_type)
            )

    def get_package_info(self, package_name: str) -> Package:
        for package in self.package_list:
            if package.name == package_name:
                return package
        return None

    def __iter__(self):
        return iter(self.package_list)

    def __check_package_full_path(self, ros_ws: Path, package_path: Path):
        if (ros_ws / package_path).exists():
            return
        raise ValueError(
            f"Package path {package_path} does not exist in workspace {ros_ws}"
        )
