#! /usr/bin/env python3

from distutils import dir_util
import shutil
from pathlib import Path
from jinja2 import Environment, FileSystemLoader
import csv
import urllib.parse


def convert_hugo_style_img(url: str) -> str:
    return "![" + url + "](" + url + ")"


def convert_badge_url(label: str, message: str, color: str) -> str:
    url = "https://img.shields.io/badge/<LABEL>-<MESSAGE>-<COLOR>"
    replaced = (
        url.replace("<LABEL>", label)
        .replace("<MESSAGE>", message)
        .replace("<COLOR>", color)
    )
    return urllib.parse.quote(replaced, safe="/:")


def read_lcov_result(file: Path) -> tuple:
    label_color = {
        "Lo": "red",
        "Med": "yellow",
        "Hi": "brightgreen",
    }

    if not file.exists():
        return 0, label_color["Lo"]

    with open(file) as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row["type"] == "Lines":
                return row["value"], label_color[row["signal"]]
    return 0, label_color["Lo"]


def lizard_color(value: int) -> str:
    if value == 0:
        return "brightgreen"
    else:
        return "red"


def read_lizard_result(file: Path) -> tuple:
    if not file.exists():
        return 0, lizard_color(0)

    with open(file) as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row["type"] == "CCN(violate)":
                return row["value"], lizard_color(row["value"])
    return 0, lizard_color(0)


def replace_summary_page(file: Path, metrics_dir: Path, packages: list):
    # Read file, replace token and overwrite file
    env = Environment(loader=FileSystemLoader(str(file.parent)))
    template = env.get_template(file.name)

    # Get badge
    param_list = []
    for package in packages:
        param = {}
        param["package"] = package
        lcov_csv = metrics_dir / package / "coverage.csv"
        lcov_cov, lcov_color = read_lcov_result(lcov_csv)
        param["coverage_badge"] = convert_hugo_style_img(
            convert_badge_url("coverage", str(lcov_cov), lcov_color)
        )

        lizard_csv = Path("metrics", "latest", package, "lizard.csv")
        lizard_count, lizard_color = read_lizard_result(lizard_csv)
        param["metrics_badge"] = convert_hugo_style_img(
            convert_badge_url("violations", str(lizard_count), lizard_color)
        )
        param_list.append(param)

    with open(file, "w") as f:
        f.write(template.render(param_list=param_list))


def replace_token(package: str) -> dict:
    lcov_html = "/__lcov/" + package + "/index.html"
    lizard_html = "/__lizard/" + package + "/index.html"
    coverage_json = package + "/coverage.json"
    metrics_json = package + "/metrics.json"

    return {
        "package_name": package,
        "lcov_result_html_link": lcov_html,
        "lizard_result_html_link": lizard_html,
        "plotly_lcov_figure_name": coverage_json,
        "plotly_metrics_figure_name": metrics_json,
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
    template = Path(dest, "content", "packages", "TEMPLATE.md")
    for package in packages:
        filename = dest / "content" / "packages" / (package + ".md")
        shutil.copy(template, filename)
        # Peplace token
        replace_contents(filename, package)

    template.unlink()

    # Copy layouts to hugo layouts directory
    layout_dir_src = Path(src, "layouts", "shortcodes")
    layout_dir_dest = Path(dest, "layouts", "shortcodes")
    dir_util.copy_tree(layout_dir_src, str(layout_dir_dest))
