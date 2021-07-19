#! /usr/bin/env python3

from distutils import dir_util
import shutil
from pathlib import Path
from typing import List
from jinja2 import Environment, FileSystemLoader
import csv
from enum import Enum


class Color(Enum):
    RED = "D9634C"
    YELLOW = "D6AF22"
    GREEN = "4FC921"
    GREY = "828282"


def add_package_link(package_name: str) -> str:
    return f'<a href="{{{{< relref "/packages/{package_name}" >}}}}">{package_name}</a>'


def convert_color_cell(message: str, color_code: str) -> str:
    template = "<td bgcolor=COLOR>MESSAGE"
    return template.replace("MESSAGE", message).replace("COLOR", color_code)


def read_lcov_result(file: Path, type: str) -> tuple:
    label_color = {
        "None": Color.GREY.value,
        "Lo": Color.RED.value,
        "Med": Color.YELLOW.value,
        "Hi": Color.GREEN.value,
    }

    if not file.exists():
        return "N/A", label_color["None"]

    with open(file) as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row["type"] == type:
                return row["value"], label_color[row["signal"]]
    return "N/A", label_color["None"]


def lizard_color(type: str, value: float) -> str:
    if "worst" in type:
        if value < 10:
            return Color.GREEN.value
        else:
            return Color.RED.value
    else:
        if value == 0:
            return Color.GREEN.value
        else:
            return Color.RED.value


def read_lizard_result(file: Path, type: str) -> tuple:
    if not file.exists():
        return "N/A", lizard_color(type, 0)

    with open(file) as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row["type"] == type:
                return row["value"], lizard_color(type, float(row["value"]))
    return "N/A", lizard_color(0)


def replace_summary_page(file: Path, metrics_dir: Path, packages: List[str]):
    # Read file, replace token and overwrite file
    env = Environment(
        loader=FileSystemLoader(str(file.parent)),
        variable_start_string="[[",
        variable_end_string="]]",
    )
    template = env.get_template(file.name)

    # Get badge
    param_list = []
    for package in packages:
        if package == "all":
            continue
        param = {}
        param["package"] = add_package_link(package)
        lcov_csv = metrics_dir / package / "coverage.csv"
        badge_names = {
            "line_badge": "Lines",
            "functions_badge": "Functions",
            "branches_badge": "Branches",
        }
        for badge_name, type_name in badge_names.items():
            lcov_cov, lcov_color = read_lcov_result(lcov_csv, type_name)
            if badge_name == "branches_badge":
                # Set background of branches coverage to gray
                lcov_color = Color.GREY.value
            param[badge_name] = convert_color_cell(str(lcov_cov), lcov_color)

        lizard_csv = metrics_dir / package / "lizard.csv"
        badge_names = {
            "ccn_worst_badge": "CCN(worst)",
            "ccn_violation_badge": "CCN(violate)",
            "ccn_warning_badge": "CCN(warning)",
            "loc_worst_badge": "LOC(worst)",
            "loc_violation_badge": "LOC(violate)",
            "loc_warning_badge": "LOC(warning)",
            "parameter_worst_badge": "Parameter(worst)",
            "parameter_violation_badge": "Parameter(violate)",
            "parameter_warning_badge": "Parameter(warning)",
        }
        for badge_name, type_name in badge_names.items():
            lizard_count, lizard_color = read_lizard_result(lizard_csv, type_name)
            param[badge_name] = convert_color_cell(str(lizard_count), lizard_color)

        param_list.append(param)

    param_list = sorted(param_list, key=lambda x: x["package"])

    render_dict = replace_token("all")

    render_dict["param_list"] = param_list

    with open(file, "w") as f:
        f.write(template.render(render_dict))


def replace_token(package: str) -> dict:
    lcov_html = "/lcov/" + package
    lizard_html = "/lizard/" + package
    tidy_html = "/tidy"
    coverage_json = package + "/coverage.json"
    ccn_json = package + "/ccn.json"
    loc_json = package + "/loc.json"
    parameter_json = package + "/parameter.json"
    token_json = package + "/token.json"

    return {
        "package_name": package,
        "lcov_result_html_link": lcov_html,
        "lizard_result_html_link": lizard_html,
        "tidy_result_html_link": tidy_html,
        "plotly_lcov_figure_name": coverage_json,
        "plotly_metrics_ccn_figure_name": ccn_json,
        "plotly_metrics_loc_figure_name": loc_json,
        "plotly_metrics_parameter_figure_name": parameter_json,
        "plotly_metrics_token_figure_name": token_json,
    }


def replace_contents(file: Path, package: str):
    # Read file, replace token and overwrite file
    env = Environment(
        loader=FileSystemLoader(str(file.parent)),
        variable_start_string="[[",
        variable_end_string="]]",
    )
    template = env.get_template(file.name)

    with open(file, "w") as f:
        f.write(template.render(replace_token(package)))


def copy_template(src: Path, dest: Path, metrics_dir: Path, packages: List[str]):
    # Copy all files from template/hugo/content/ to hugo content directory
    markdown_dir_src = src / "content"
    markdown_dir_dest = dest / "content"
    dir_util.copy_tree(markdown_dir_src, str(markdown_dir_dest))

    # Create summary page
    summary_page = markdown_dir_dest / "_index.md"
    replace_summary_page(summary_page, metrics_dir, packages)

    # Create package detail page
    template = dest / "content" / "packages" / "TEMPLATE.md"
    for package in packages:
        if package == "all":
            continue
        filename = dest / "content" / "packages" / (package + ".md")
        shutil.copy(template, filename)
        # Peplace token
        replace_contents(filename, package)

    template.unlink()

    # Copy layouts to hugo layouts directory
    layout_dir_src = src / "layouts" / "shortcodes"
    layout_dir_dest = dest / "layouts" / "shortcodes"
    dir_util.copy_tree(layout_dir_src, str(layout_dir_dest))
