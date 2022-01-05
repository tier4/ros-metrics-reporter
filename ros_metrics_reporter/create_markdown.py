#! /usr/bin/env python3

from distutils import dir_util
import shutil
from pathlib import Path
from typing import Dict, List
from datetime import datetime
from ros_metrics_reporter.metrics.metrics_data import MetricsDataKeys, MetricsValueKeys
from ros_metrics_reporter.package_info import PackageInfo

from ros_metrics_reporter.util import read_jinja2_template
from ros_metrics_reporter.coverage.coverage_data import CoverageKeys
from ros_metrics_reporter.static_page_input import StaticPageInput
import ros_metrics_reporter.coverage.coverage_data as coverage_data
import ros_metrics_reporter.metrics.metrics_data as metrics_data
from ros_metrics_reporter.color import Color


def add_package_link(package_name: str) -> str:
    """Replace a tag with a hugo-style link to the package page.

    Args:
        package_name: The name of the package.

    Returns:
        A string with the hugo-style link to the package page.

    >>> add_package_link("foo")
    '<a href="{{< relref "/packages/foo" >}}">foo</a>'
    """
    return f'<a href="{{{{< relref "/packages/{package_name}" >}}}}">{package_name}</a>'


def convert_color_cell(message: str, color_code: Color) -> str:
    """Replace a tag with a color cell.

    Args:
        message: The message to be displayed in the cell.
        color_code: background color of the cell.

    Returns:
        A string with the legend class.

    >>> convert_color_cell("foo", Color.RED)
    '<td class="LegendLo">foo'
    """
    template = '<td class="COLOR">MESSAGE'
    if color_code == Color.RED:
        return template.replace("MESSAGE", message).replace("COLOR", "LegendLo")
    elif color_code == Color.YELLOW:
        return template.replace("MESSAGE", message).replace("COLOR", "LegendMed")
    elif color_code == Color.GREEN:
        return template.replace("MESSAGE", message).replace("COLOR", "LegendHi")
    else:
        return template.replace("MESSAGE", message).replace("COLOR", "LegendNA")


def convert_legend(
    coverage_thresold: coverage_data.Threshold, metrics_thresold: metrics_data.Threshold
) -> Dict[str, str]:
    return {
        "coverage_hi": coverage_thresold.high,
        "coverage_med": coverage_thresold.med,
        "ccn_recommendation": metrics_thresold.recommendation_value.ccn,
        "loc_recommendation": metrics_thresold.recommendation_value.nloc,
        "parameter_recommendation": metrics_thresold.recommendation_value.parameter,
        "ccn_threshold": metrics_thresold.threshold_value.ccn,
        "loc_threshold": metrics_thresold.threshold_value.nloc,
        "parameter_threshold": metrics_thresold.threshold_value.parameter,
    }


def get_timestamp_from_lizard(file: Path, format: str) -> datetime:
    return datetime.fromtimestamp(file.stat().st_mtime).strftime(format)


def replace_summary_page(
    file: Path,
    metrics_dir: Path,
    input_data: StaticPageInput,
):
    template = read_jinja2_template(file)

    # Replace table
    param_list = []
    for package in input_data.packages:
        if package.name == "all":
            continue
        param = {}
        param["package"] = add_package_link(package.name)

        # Coverage
        package_coverage_value = input_data.code_coverage.coverage_data.get_coverage(
            package.name
        )
        coverage_names = {
            "line_badge": CoverageKeys.Lines,
            "functions_badge": CoverageKeys.Functions,
            "branches_badge": CoverageKeys.Branches,
        }
        for badge_name, type_name in coverage_names.items():
            lcov_value = package_coverage_value.get_label_value("all").get(type_name)
            lcov_color = input_data.code_coverage.coverage_data.get_color(
                lcov_value, type_name
            )
            if badge_name == "branches_badge":
                # Set background of branches coverage to gray
                lcov_color = Color.GREY
            param[badge_name] = convert_color_cell(str(lcov_value), lcov_color)

        # Metrics
        lizard_result = input_data.metrics.metrics_data.get_metrics_data(package.name)

        metrics_names = {
            "worst_badge": MetricsDataKeys.Worst,
            "violation_badge": MetricsDataKeys.OverThresholdCount,
            "warning_badge": MetricsDataKeys.OverRecommendationCount,
        }

        for badge_name_suffix, metrics_data_key in metrics_names.items():
            metrics_value_names = {
                "ccn_": MetricsValueKeys.CCN,
                "loc_": MetricsValueKeys.NLOC,
                "parameter_": MetricsValueKeys.PARAMETER,
            }
            for badge_name_prefix, value_name_keys in metrics_value_names.items():
                value = lizard_result.get_value(metrics_data_key).get(value_name_keys)
                color = input_data.metrics.metrics_data.threshold.get_color(
                    value, metrics_data_key, value_name_keys
                )
                badge_name = f"{badge_name_prefix}{badge_name_suffix}"
                param[badge_name] = convert_color_cell(str(value), color)

        param_list.append(param)

    param_list = sorted(param_list, key=lambda x: x["package"])

    render_dict = replace_token("all", input_data.code_coverage.test_label)

    render_dict["param_list"] = param_list

    # Read datetime
    render_dict["last_updated"] = get_timestamp_from_lizard(
        metrics_dir / "all" / "lizard.json", "%Y-%m-%d %H:%M:%S UTC"
    )

    # get repository statistics information
    for i, contributor in enumerate(input_data.contributors["all"], 1):
        render_dict["contributor_name_" + str(i)] = contributor["name"]
        render_dict["contributor_avatar_" + str(i)] = contributor["avatar"]
        render_dict["contribute_count_" + str(i)] = contributor["total"]

    # Replace legend
    legend_dict = convert_legend(
        input_data.code_coverage.coverage_data.threshold,
        input_data.metrics.metrics_data.threshold,
    )
    render_dict.update(legend_dict)

    with open(file, "w") as f:
        f.write(template.render(render_dict))


def replace_token(package: str, test_label_list: List[str]) -> Dict[str, str]:
    lizard_html = "/lizard/" + package
    tidy_html = "/tidy"
    ccn_json = package + "/ccn.json"
    loc_json = package + "/nloc.json"
    parameter_json = package + "/parameter.json"
    token_json = package + "/token.json"

    coverage_graph_list = []
    # Code coverage for all tests
    coverage_graph_list.append(
        {
            "name": "all",
            "plotly_lcov_figure_name": package + "/coverage.all.json",
            "lcov_result_html_link": "/lcov/" + package,
        }
    )

    for label in test_label_list:
        coverage_graph_list.append(
            {
                "name": label,
                "plotly_lcov_figure_name": package + f"/coverage.{label}.json",
                "lcov_result_html_link": "/lcov/" + package + f"/{label}",
            }
        )

    return {
        "package_name": package,
        "lizard_result_html_link": lizard_html,
        "tidy_result_html_link": tidy_html,
        "plotly_metrics_ccn_figure_name": ccn_json,
        "plotly_metrics_loc_figure_name": loc_json,
        "plotly_metrics_parameter_figure_name": parameter_json,
        "plotly_metrics_token_figure_name": token_json,
        "test_label_list": coverage_graph_list,
    }


def replace_contents(file: Path, package: str, input_data: StaticPageInput):
    template = read_jinja2_template(file)
    render_dict = replace_token(package, input_data.code_coverage.test_label)

    # get repository statistics information
    for i, contributor in enumerate(input_data.contributors[package], 1):
        render_dict["contributor_name_" + str(i)] = contributor["name"]
        render_dict["contribute_count_" + str(i)] = contributor["total"]

    with open(file, "w") as f:
        f.write(template.render(render_dict))


def run_markdown_generator(
    src: Path,
    dest: Path,
    metrics_dir: Path,
    input_data: StaticPageInput,
):
    # Copy all files from template/hugo/content/ to hugo content directory
    markdown_dir_src = src / "content"
    markdown_dir_dest = dest / "content"
    dir_util.copy_tree(markdown_dir_src, str(markdown_dir_dest))

    # Create summary page
    summary_page = markdown_dir_dest / "_index.md"
    replace_summary_page(summary_page, metrics_dir, input_data)

    # Create package detail page
    template = dest / "content" / "packages" / "TEMPLATE.md"
    for package in input_data.packages:
        if package.name == "all":
            continue
        filename = dest / "content" / "packages" / (package.name + ".md")
        shutil.copy(template, filename)
        # Replace token
        replace_contents(filename, package.name, input_data)

    template.unlink()

    # Copy layouts to hugo layouts directory
    layout_dir_src = src / "layouts" / "shortcodes"
    layout_dir_dest = dest / "layouts" / "shortcodes"
    dir_util.copy_tree(layout_dir_src, str(layout_dir_dest))


if __name__ == "__main__":
    import doctest

    doctest.testmod()
