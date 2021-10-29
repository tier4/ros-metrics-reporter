from pathlib import Path
from ros_metrics_reporter.package_info import *


def test_package_info():
    package_info = PackageInfo(ros_ws=Path("example/src/demos"))
    assert package_info.ros_ws == Path("example/src/demos").absolute()

    package = package_info.get_package_info("demo_nodes_cpp")
    assert package.name == "demo_nodes_cpp"
    assert package.path == Path("demo_nodes_cpp")
    assert package.type == "(ros.ament_cmake)"

    package_count = 0
    for package in package_info:
        package_count += 1
        assert package.name is not None

    assert len(package_info.package_list) == package_count
