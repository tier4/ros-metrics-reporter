from typing import List
import re
import bs4
from pathlib import Path
from ros_metrics_reporter.coverage.coverage_data import Coverage, CoverageKeys
from ros_metrics_reporter.scraping.common import save_to_csv, save_to_json


class LcovScraping:
    @staticmethod
    def __classname_to_signal(classname: str) -> str:
        return re.sub("headerCovTableEntry", "", classname)

    @staticmethod
    def __get_lcov_coverage(html_path: str) -> List[dict]:
        """Get Lines, Functions, Branches coverage rate"""
        soup = bs4.BeautifulSoup(open(html_path), "html.parser")
        rate_list = []
        for tr in soup.select("body > table:nth-of-type(1) > tr > td > table > tr"):
            try:
                coverage_type = tr.select("td:nth-of-type(4)")[0]
                percentage = tr.select("td:nth-of-type(7)")[0]
                if "%" in percentage.text:
                    rate = {}
                    rate["type"] = re.sub(":", "", coverage_type.text)
                    rate["value"] = re.sub(" %", "", percentage.text)
                    rate["signal"] = LcovScraping.__classname_to_signal(
                        percentage["class"][0]
                    )
                    rate_list.append(rate)
            except:
                continue

        return rate_list

    def __list_to_coverage(
        self, package_name: str, coverage_list: List[dict]
    ) -> Coverage:
        """List to Coverage"""
        coverage = Coverage(label=coverage_list[0]["label"])
        coverage.package = package_name
        for item in coverage_list:
            if item["type"] == CoverageKeys.Lines:
                coverage.lines = item["value"]
            elif item["type"] == CoverageKeys.Functions:
                coverage.functions = item["value"]
            elif item["type"] == CoverageKeys.Branches:
                coverage.branches = item["value"]
        return coverage

    def scraping(
        self, lcov_dir: Path, output_dir: Path, test_label: str = ""
    ) -> List[Coverage]:
        """Scraping lcov result"""
        if test_label:
            search_pattern = f"**/{test_label}/index.html"
            test_label_str = test_label
        else:
            search_pattern = "*/index.html"
            test_label_str = "all"
        lcov_index_list = list(lcov_dir.glob(search_pattern))

        coverage_list = []

        for html in lcov_index_list:
            if test_label:
                package_name = html.parent.parent.name
            else:
                package_name = html.parent.name
            output_csv_dir = output_dir / package_name
            output_csv_dir.mkdir(exist_ok=True, parents=True)

            coverages = LcovScraping.__get_lcov_coverage(html)

            # Add coverage type
            for coverage in coverages:
                coverage["label"] = test_label_str

            filename = output_csv_dir / "coverage.csv"
            if filename.exists():
                write_mode = "a"
            else:
                write_mode = "w"

            save_to_csv(filename, coverages[0].keys(), coverages, write_mode)

            filename = output_csv_dir / "coverage.json"
            save_to_json(filename, coverages)

            coverage_list.append(self.__list_to_coverage(package_name, coverages))

        return coverage_list
