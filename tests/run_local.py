from datetime import datetime
from pathlib import Path
import sys
import os
import shutil


class Container:
    pass


sys.path.append(os.path.abspath(".."))
sys.path.append(os.path.abspath("../scripts"))
from scripts.ros_metrics_reporter import ros_metrics_reporter

root_path = Path(__file__).parent.parent.resolve()

# copy template dir to output dir
template_dir = root_path.joinpath("example", "hugo-site")
hugo_test_dir = root_path.joinpath("hugo-test")
if hugo_test_dir.exists():
    shutil.rmtree(hugo_test_dir)
shutil.copytree(template_dir, hugo_test_dir)

args = Container()
args.output_dir = root_path.joinpath("output")
args.base_dir = root_path.joinpath("example", "src", "demos")
args.action_dir = root_path
args.hugo_root_dir = hugo_test_dir
args.base_url = "http://localhost:1313/"
args.title = "test"
args.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
args.lcovrc = root_path.joinpath(".lcovrc")
args.exclude = ""
args.ccn = 20
args.nloc = 200
args.arguments = 6
args.ccn_recommendation = 6
args.nloc_recommendation = 100
args.arguments_recommendation = 10
args.tidy_config_path = root_path.joinpath("codechecker-config.json")
args.tidy_ignore_path = root_path.joinpath("codechecker-skip-list.txt")

ros_metrics_reporter(args)
