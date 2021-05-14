#! /usr/bin/env python3

import argparse
import bs4
import re

from util import dir_path


def get_lcov_coverage(html_path: str) -> dict:
    """ Get Lines, Functions, Branches coverage rate """
    soup = bs4.BeautifulSoup(open(html_path), "html.parser")

    rate = {}
    for tr in soup.select("body > table:nth-of-type(1) > tr > td > table > tr"):
        try:
            coverage_type = tr.select("td:nth-of-type(4)")[0]
            percentage = tr.select("td:nth-of-type(7)")[0]
            if "%" in percentage.text:
                rate[coverage_type.text] = re.sub(" %", "", percentage.text)
        except:
            continue

    return rate


def get_worst_case(metrics: list) -> int:
    """ Get worst case """
    if metrics:
        return max(int(item.get_text()) for item in metrics)
    else:
        return 0


def get_violate_count(metrics: list) -> int:
    """ Get number of functions that violated the criteria """
    if metrics:
        return sum(item["class"][0] == "greater-value" for item in metrics)
    else:
        return 0


def get_lizard_metrics(html_path: str) -> list:
    """
    Get following information
      Cyclomatic complexity
        Worst case
        Number of functions that violated the criteria (described "greater-value" in html)
      LOC
        Worst case
        Number of functions that violated the criteria (described "greater-value" in html)
      Parameter count
        Worst case
        Number of functions that violated the criteria (described "greater-value" in html)
    """
    soup = bs4.BeautifulSoup(open(html_path), "html.parser")
    table = soup.select("body > center > table:nth-of-type(1)")[0]

    ccn = []
    loc = []
    parameter = []

    for tr in table.find_all("tr"):
        tds = tr.select("td")
        if len(tds) == 6:
            ccn.append(tds[2])
            loc.append(tds[3])
            parameter.append(tds[5])

    return [
        get_worst_case(ccn),
        get_violate_count(ccn),
        get_worst_case(loc),
        get_violate_count(loc),
        get_worst_case(parameter),
        get_violate_count(parameter),
    ]


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--lcov_dir", help="Path to lcov result", type=dir_path, required=True
    )
    parser.add_argument(
        "--lizard_dir", help="Path to lizard result", type=dir_path, required=True
    )
    parser.add_argument(
        "--output_dir", help="Path to output directory", type=dir_path, required=True
    )

    args = parser.parse_args()

    lcov_index_list = list(args.lcov_dir.glob("*/index.html"))
    lizard_index_list = list(args.lizard_dir.glob("*/index.html"))

    # Create output directory
    for html in lizard_index_list:
        dirname = html.parent.name
        (args.output_dir / dirname).mkdir()

    for html in lcov_index_list:
        coverage = get_lcov_coverage(html)

        filename = args.output_dir / html.parent.name / "coverage.txt"
        with open(filename, "w") as f:
            for key, val in coverage.items():
                f.write(f"{key} {val}\n")

    for html in lizard_index_list:
        metrics = get_lizard_metrics(html)
        filename = args.output_dir / html.parent.name / "lizard.txt"
        with open(filename, "w") as f:
            f.write(f"CCN(worst): {metrics[0]}\n")
            f.write(f"CCN(violate): {metrics[1]}\n")
            f.write(f"LOC(worst): {metrics[2]}\n")
            f.write(f"LOC(violate): {metrics[3]}\n")
            f.write(f"Parameter(worst): {metrics[4]}\n")
            f.write(f"Parameter(violate): {metrics[5]}\n")
