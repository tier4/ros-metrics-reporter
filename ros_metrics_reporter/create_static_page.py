#! /usr/bin/env python3

from pathlib import Path
from distutils import dir_util

from ros_metrics_reporter.util import read_jinja2_template
from ros_metrics_reporter.create_markdown import run_markdown_generator
from ros_metrics_reporter.static_page_input import StaticPageInput


def copy_artifacts(src: Path, dest: Path):
    dest.mkdir(exist_ok=True)
    package_dirs = [x for x in src.iterdir() if x.is_dir()]
    for package_dir in package_dirs:
        package_dest = dest / package_dir.name
        package_dest.mkdir(exist_ok=True)
        dir_util.copy_tree(package_dir, str(package_dest))


def copy_html(
    input: StaticPageInput,
):
    # Copy artifacts
    lcov_dest = input.hugo_root_dir / "static" / "lcov"
    copy_artifacts(input.code_coverage.html_dir, lcov_dest)

    lizard_dest = input.hugo_root_dir / "static" / "lizard"
    copy_artifacts(input.metrics.output_html_dir, lizard_dest)

    tidy_dest = input.hugo_root_dir / "static" / "tidy"
    dir_util.copy_tree(input.tidy_result_path, str(tidy_dest))


def replace_hugo_config(
    input: StaticPageInput,
):
    config_file = input.hugo_root_dir / "config.toml"
    template = read_jinja2_template(config_file)

    with open(config_file, "w") as f:
        f.write(
            template.render(
                {
                    "base_url": input.base_url,
                    "title": input.title,
                }
            )
        )


def create_static_page(input: StaticPageInput):
    copy_html(
        input=input,
    )
    replace_hugo_config(
        input=input,
    )

    # Create markdown from template
    run_markdown_generator(
        input.hugo_template_dir,
        input.hugo_root_dir,
        input.input_dir / "latest",
        input,
    )
