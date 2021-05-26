#! /usr/bin/env python3

from distutils import dir_util
import shutil
from pathlib import Path
from jinja2 import Environment, FileSystemLoader
import csv


def add_package_link(package_name: str) -> str:
    return f'<a href="{{{{< relref "/packages/{package_name}" >}}}}">{package_name}</a>'


def convert_color_cell(message: str, color_code: str) -> str:
    template = "<td bgcolor=COLOR>MESSAGE"
    return template.replace("MESSAGE", message).replace("COLOR", color_code)


def read_lcov_result(file: Path, type: str) -> tuple:
    label_color = {
        "Lo": "D9634C",
        "Med": "D6AF22",
        "Hi": "4FC921",
    }

    if not file.exists():
        return 0, label_color["Lo"]

    with open(file) as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row["type"] == type:
                return row["value"], label_color[row["signal"]]
    return 0, label_color["Lo"]


def lizard_color(value: int) -> str:
    if value == 0:
        return "4FC921"
    else:
        return "D9634C"


def read_lizard_result(file: Path, type: str) -> tuple:
    if not file.exists():
        return 0, lizard_color(0)

    with open(file) as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row["type"] == type:
                return int(row["value"]), lizard_color(int(row["value"]))
    return 0, lizard_color(0)


def replace_summary_page(file: Path, metrics_dir: Path, packages: list):
    # Read file, replace token and overwrite file
    env = Environment(loader=FileSystemLoader(str(file.parent)))
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
            param[badge_name] = convert_color_cell(str(lcov_cov), lcov_color)

        lizard_csv = metrics_dir / package / "lizard.csv"
        badge_names = {
            "ccn_badge": "CCN(violate)",
            "loc_badge": "LOC(violate)",
            "parameter_badge": "Parameter(violate)",
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
    lcov_html = "/lcov/" + package + "/index.html"
    lizard_html = "/lizard/" + package + "/index.html"
    coverage_json = package + "/coverage.json"
    ccn_json = package + "/ccn.json"
    loc_json = package + "/loc.json"
    parameter_json = package + "/parameter.json"
    token_json = package + "/token.json"

    return {
        "package_name": package,
        "lcov_result_html_link": lcov_html,
        "lizard_result_html_link": lizard_html,
        "plotly_lcov_figure_name": coverage_json,
        "plotly_metrics_ccn_figure_name": ccn_json,
        "plotly_metrics_loc_figure_name": loc_json,
        "plotly_metrics_parameter_figure_name": parameter_json,
        "plotly_metrics_token_figure_name": token_json,
    }


def replace_contents(file: Path, package: str):
    # Read file, replace token and overwrite file
    env = Environment(loader=FileSystemLoader(str(file.parent)))
    template = env.get_template(file.name)

    with open(file, "w") as f:
        f.write(template.render(replace_token(package)))


def copy_template(src: Path, dest: Path, metrics_dir: Path, packages: list):
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
