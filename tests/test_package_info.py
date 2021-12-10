from pathlib import Path
from ros_metrics_reporter.package_info import *
import shutil


def test_package_info_root(tmp_path):
    dummy_pkg = Path(os.path.dirname(__file__), "resource", "dummy_package")
    ros_ws = Path(tmp_path)
    git_ws = Path(tmp_path)
    (git_ws / ".git").mkdir(parents=True)
    shutil.copytree(dummy_pkg, git_ws / "dummy_package")

    package_info = PackageInfo(ros_ws)
    assert package_info.ros_ws == Path(tmp_path)

    package = package_info.get_package_info("dummy_package")
    assert package.name == "dummy_package"
    assert package.git_ws == git_ws
    assert package.path == "dummy_package"
    assert package.type == "(ros.ament_cmake)"


def test_package_info_subdir(tmp_path):
    dummy_pkg = Path(os.path.dirname(__file__), "resource", "dummy_package")
    ros_ws = Path(tmp_path)
    git_ws = Path(tmp_path / "subdir")
    (git_ws / ".git").mkdir(parents=True)
    shutil.copytree(dummy_pkg, git_ws / "dummy_package")

    package_info = PackageInfo(ros_ws)
    assert package_info.ros_ws == Path(tmp_path)

    package = package_info.get_package_info("dummy_package")
    assert package.name == "dummy_package"
    assert package.git_ws == git_ws
    assert package.path == "dummy_package"
    assert package.type == "(ros.ament_cmake)"
