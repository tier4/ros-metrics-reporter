#! /usr/bin/env python3

import bs4
from pathlib import Path
from numpy.core.fromnumeric import mean
from decimal import Decimal, ROUND_HALF_UP
from ros_metrics_reporter.metrics.metrics_data import *


class ScrapingData:
    package_name: str
    threshold_data: dict
    recommendation_data: dict


def get_worst_case(metrics: list) -> int:
    """Get worst case"""
    if metrics:
        return max(int(item.get_text()) for item in metrics)
    else:
        return 0


def get_average(metrics: list) -> float:
    """Get average"""
    if metrics:
        value = mean([int(item.get_text()) for item in metrics])
        return float(
            Decimal(str(value)).quantize(Decimal(".1"), rounding=ROUND_HALF_UP)
        )
    else:
        return 0


def get_violate_count(metrics: list) -> int:
    """Get number of functions that violated the criteria"""
    if metrics:
        return sum(item["class"][0] == "greater-value" for item in metrics)
    else:
        return 0


def scraping_lizard_result(html_path: Path) -> dict:
    soup = bs4.BeautifulSoup(open(html_path), "html.parser")
    table = soup.select("body > center > table:nth-of-type(1)")[0]

    ccn = []
    loc = []
    parameter = []
    token = []

    for tr in table.find_all("tr"):
        tds = tr.select("td")
        if len(tds) == 6:
            ccn.append(tds[2])
            loc.append(tds[3])
            token.append(tds[4])
            parameter.append(tds[5])
    return {
        "ccn": ccn,
        "loc": loc,
        "token": token,
        "parameter": parameter,
    }


def get_lizard_metrics(scraping_data: ScrapingData) -> MetricsData:
    """
    Get following information
      Cyclomatic complexity
        Worst case
        Average
        Number of functions that violated the criteria (described "greater-value" in html)
      LOC
        Worst case
        Average
        Number of functions that violated the criteria (described "greater-value" in html)
      Parameter count
        Worst case
        Average
        Number of functions that violated the criteria (described "greater-value" in html)
    Token count
        Worst case
        Average
    """
    metrics = MetricsData(
        package=scraping_data.package_name,
        worst_value=MetricsValue(
            ccn=get_worst_case(scraping_data.threshold_data["ccn"]),
            nloc=get_worst_case(scraping_data.threshold_data["loc"]),
            parameter=get_worst_case(scraping_data.threshold_data["parameter"]),
            token=get_worst_case(scraping_data.threshold_data["token"]),
        ),
        average_value=FloatMetricsValue(
            ccn=get_average(scraping_data.threshold_data["ccn"]),
            nloc=get_average(scraping_data.threshold_data["loc"]),
            parameter=get_average(scraping_data.threshold_data["parameter"]),
            token=get_average(scraping_data.threshold_data["token"]),
        ),
        over_threshold_count=MetricsValue(
            ccn=get_violate_count(scraping_data.threshold_data["ccn"]),
            nloc=get_violate_count(scraping_data.threshold_data["loc"]),
            parameter=get_violate_count(scraping_data.threshold_data["parameter"]),
        ),
        over_recommendation_count=MetricsValue(
            ccn=get_violate_count(scraping_data.recommendation_data["ccn"]),
            nloc=get_violate_count(scraping_data.recommendation_data["loc"]),
            parameter=get_violate_count(scraping_data.recommendation_data["parameter"]),
        ),
    )
    return metrics


def scraping(lizard_dir: Path, lizard_recommendation_dir: Path) -> List[MetricsData]:
    scraping_data = []
    for html_path in lizard_dir.glob("*/index.html"):
        package_name = html_path.parent.name
        item = ScrapingData()
        item.package_name = package_name
        item.threshold_data = scraping_lizard_result(html_path)
        scraping_data.append(item)

    for html_path in lizard_recommendation_dir.glob("*/index.html"):
        package_name = html_path.parent.name
        for item in scraping_data:
            if item.package_name == package_name:
                item.recommendation_data = scraping_lizard_result(html_path)
                break

    metrics_data_list = []
    for item in scraping_data:
        metrics_data_list.append(get_lizard_metrics(item))

    return metrics_data_list
