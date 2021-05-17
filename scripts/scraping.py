#! /usr/bin/env python3

import argparse
import bs4
import re
import csv

from util import dir_path

def classname_to_signal(classname: str) -> str:
    return re.sub("headerCovTableEntry", "", classname)

def get_lcov_coverage(html_path: str) -> list:
    """ Get Lines, Functions, Branches coverage rate """
    soup = bs4.BeautifulSoup(open(html_path), "html.parser")
    rate_list = []
    for tr in soup.select("body > table:nth-of-type(1) > tr > td > table > tr"):
        try:
            coverage_type = tr.select("td:nth-of-type(4)")[0]
            percentage = tr.select("td:nth-of-type(7)")[0]
            if "%" in percentage.text:
                rate = {}
                rate['type'] = re.sub(":", "", coverage_type.text)
                rate['value'] = re.sub(" %", "", percentage.text)
                rate['signal'] = classname_to_signal(percentage['class'][0])
                rate_list.append(rate)
        except:
            continue

    return rate_list


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
        {"type": "CCN(worst)", "value": get_worst_case(ccn)},
        {"type": "CCN(violate)", "value": get_violate_count(ccn)},
        {"type": "LOC(worst)", "value": get_worst_case(loc)},
        {"type": "LOC(violate)", "value": get_violate_count(loc)},
        {"type": "Parameter(worst)", "value": get_worst_case(parameter)},
        {"type": "Parameter(violate)", "value": get_violate_count(parameter)},
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
        (args.output_dir / dirname).mkdir(exist_ok=True)

    for html in lcov_index_list:
        coverages = get_lcov_coverage(html)

        filename = args.output_dir / html.parent.name / "coverage.csv"
        with open(filename, "w") as f:
            writer = csv.DictWriter(f, fieldnames=coverages[0].keys())
            writer.writeheader()
            for coverage in coverages:
                writer.writerow(coverage)

    for html in lizard_index_list:
        metrics = get_lizard_metrics(html)
        filename = args.output_dir / html.parent.name / "lizard.csv"
        with open(filename, "w") as f:
            writer = csv.DictWriter(f, fieldnames=metrics[0].keys())
            writer.writeheader()
            for item in metrics:
                writer.writerow(item)
