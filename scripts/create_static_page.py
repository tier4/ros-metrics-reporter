#! /usr/bin/env python3

from pathlib import Path
from distutils import dir_util
from jinja2 import Environment, FileSystemLoader
from typing import List, Dict

from create_markdown import run_markdown_generator
from read_dataframe import read_dataframe


def copy_artifacts(src: Path, dest: Path):
    dest.mkdir(exist_ok=True)
    package_dirs = [x for x in src.iterdir() if x.is_dir()]
    for package_dir in package_dirs:
        package_dest = dest / package_dir.name
        package_dest.mkdir(exist_ok=True)
        dir_util.copy_tree(package_dir, str(package_dest))


def copy_html(
    hugo_root_dir: Path,
    lcov_result_path: Path,
    lizard_result_path: Path,
    tidy_result_path: Path,
):
    # Copy artifacts
    lcov_dest = hugo_root_dir / "static" / "lcov"
    copy_artifacts(lcov_result_path, lcov_dest)

    lizard_dest = hugo_root_dir / "static" / "lizard"
    copy_artifacts(lizard_result_path, lizard_dest)

    tidy_dest = hugo_root_dir / "static" / "tidy"
    dir_util.copy_tree(tidy_result_path, str(tidy_dest))


def replace_hugo_config(
    hugo_root_dir: Path,
    base_url: str,
    title: str,
):
    config_file = hugo_root_dir / "config.toml"

    env = Environment(
        loader=FileSystemLoader(str(hugo_root_dir)),
        variable_start_string="[[",
        variable_end_string="]]",
    )
    template = env.get_template(config_file.name)

    with open(config_file, "w") as f:
        f.write(
            template.render(
                {
                    "base_url": base_url,
                    "title": title,
                }
            )
        )


def generate_markdown(
    base_path: Path,
    hugo_root_dir: Path,
    hugo_template_dir: Path,
    packages: str,
    contributors: List[Dict],
):
    # Create markdown from template
    run_markdown_generator(
        hugo_template_dir, hugo_root_dir, base_path / "latest", packages, contributors
    )


def create_static_page(
    input_dir: Path,
    hugo_root_dir: Path,
    hugo_template_dir: Path,
    lcov_result_path: Path,
    lizard_result_path: Path,
    tidy_result_path: Path,
    base_url: str,
    title: str,
    contributors: List[Dict],
):
    copy_html(
        hugo_root_dir,
        lcov_result_path,
        lizard_result_path,
        tidy_result_path,
    )
    replace_hugo_config(
        hugo_root_dir,
        base_url,
        title,
    )

    df = read_dataframe(input_dir)
    generate_markdown(
        input_dir,
        hugo_root_dir,
        hugo_template_dir,
        df["package_name"].unique(),
        contributors,
    )
