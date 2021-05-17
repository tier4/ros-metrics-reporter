#! /usr/bin/env python3

from distutils import dir_util
import shutil
from pathlib import Path
from jinja2 import Environment, FileSystemLoader


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


def replace_summary_page(file: Path, packages: list):
    # Read file, replace token and overwrite file
    env = Environment(loader=FileSystemLoader(str(file.parent)))
    template = env.get_template(file.name)

    with open(file, "w") as f:
        f.write(template.render({
            'packages': packages,
        }))


def replace_contents(file: Path, package: str):
    # Read file, replace token and overwrite file
    env = Environment(loader=FileSystemLoader(str(file.parent)))
    template = env.get_template(file.name)

    with open(file, "w") as f:
        f.write(template.render(
            replace_token(package)
        ))


def copy_template(src: str, dest: str, packages: list):
    # Copy all files from template/hugo/content/ to hugo content directory
    markdown_dir_src = Path(src, "content")
    markdown_dir_dest = Path(dest, "content")
    dir_util.copy_tree(markdown_dir_src, str(markdown_dir_dest))

    # Create summary page
    summary_page = markdown_dir_dest / '_index.md'
    replace_summary_page(summary_page, packages)

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
