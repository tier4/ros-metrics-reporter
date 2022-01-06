from datetime import datetime
from pathlib import Path
import shutil
import subprocess


root_path = Path(__file__).parent.parent.resolve()
ros_ws = root_path / "example" / "src" / "geometry2"


def setup():
    res = subprocess.run(
        "vcs import . < ros-metrics-reporter-galactic.repos",
        shell=True,
        cwd=root_path,
    )
    if res.returncode != 0:
        print("Failed to build and test")
        exit(1)
    res = subprocess.run(
        "rosdep update && rosdep install --from-paths . --ignore-src --rosdistro galactic -y",
        shell=True,
        cwd=ros_ws,
    )
    if res.returncode != 0:
        print("Failed to build and test")
        exit(1)


def build():
    res = subprocess.run(
        '. /opt/ros/galactic/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS="-fprofile-arcs -ftest-coverage --coverage -DCOVERAGE_RUN=1" -DCMAKE_C_FLAGS="-fprofile-arcs --coverage -ftest-coverage -DCOVERAGE_RUN=1" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON',
        shell=True,
        cwd=ros_ws,
    )
    if res.returncode != 0:
        print("Failed to build")
        exit(1)


class Container:
    pass


from ros_metrics_reporter import ros_metrics_reporter


def run_ros_metrics_reporter():
    # copy template dir to output dir
    template_dir = root_path.joinpath("example", "hugo-site")
    hugo_test_dir = root_path.joinpath("hugo-test")
    if hugo_test_dir.exists():
        shutil.rmtree(hugo_test_dir)
    shutil.copytree(template_dir, hugo_test_dir)

    args = Container()
    args.output_dir = root_path.joinpath("output")
    args.base_dir = ros_ws
    args.action_dir = root_path
    args.hugo_root_dir = hugo_test_dir
    args.base_url = "http://localhost:1313/"
    args.title = "test"
    args.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    args.lcovrc = root_path.joinpath(".lcovrc")
    args.exclude = [""]
    args.ccn = 20
    args.nloc = 200
    args.arguments = 6
    args.ccn_recommendation = 6
    args.nloc_recommendation = 100
    args.arguments_recommendation = 10
    args.tidy_config_path = root_path.joinpath("codechecker-config.json")
    args.tidy_ignore_path = root_path.joinpath("codechecker-skip-list.txt")
    args.target_repo = "ros2/geometry2"
    args.github_access_token = None
    args.test_label = ["gtest", "component_test"]

    ros_metrics_reporter.ros_metrics_reporter(args)
    return args


def test_ros_metrics_reporter():
    if not ros_ws.exists():
        setup()
    if not (ros_ws / "build").exists():
        build()
    args = run_ros_metrics_reporter()

    # Check result files
    assert (ros_ws / "lcov").is_dir()
    assert (ros_ws / "lcov" / "total_coverage.info").is_file()
    for label in args.test_label:
        assert (ros_ws / ("lcov." + label)).is_dir()
        assert (ros_ws / ("lcov." + label) / "total_coverage.info").is_file()

    # Check data files
    assert (
        args.output_dir / "metrics" / args.timestamp / "all" / "coverage.json"
    ).stat().st_size > 0
    assert (
        args.output_dir / "metrics" / args.timestamp / "all" / "lizard.json"
    ).stat().st_size > 0

    # Check gh-pages files
    assert (args.hugo_root_dir / "static" / "lcov" / "all" / "index.html").is_file()
    assert (args.hugo_root_dir / "static" / "lizard" / "all" / "index.html").is_file()
    assert (args.hugo_root_dir / "static" / "tidy" / "index.html").is_file()
    assert (args.hugo_root_dir / "static" / "plotly" / "all" / "ccn.json").is_file()
    assert (
        args.hugo_root_dir / "static" / "plotly" / "all" / "code_frequency_graph.json"
    ).is_file()
    assert (args.hugo_root_dir / "static" / "plotly" / "all" / "nloc.json").is_file()
    assert (
        args.hugo_root_dir / "static" / "plotly" / "all" / "parameter.json"
    ).is_file()
    assert (args.hugo_root_dir / "static" / "plotly" / "all" / "token.json").is_file()
    for label in args.test_label:
        assert (
            args.hugo_root_dir / "static" / "plotly" / "all" / f"coverage.{label}.json"
        ).is_file()
        assert (
            args.hugo_root_dir / "static" / "lcov" / "all" / label / "index.html"
        ).is_file()


if __name__ == "__main__":
    test_ros_metrics_reporter()
