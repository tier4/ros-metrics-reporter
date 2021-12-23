from typing import List
import re
import bs4
from pathlib import Path
from ros_metrics_reporter.scraping.common import save_to_csv


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

    def scraping(self, lcov_dir: Path, output_dir: Path):
        lcov_index_list = list(lcov_dir.glob("*/index.html"))

        for html in lcov_index_list:
            dirname = html.parent.name
            (output_dir / dirname).mkdir(exist_ok=True, parents=True)

            coverages = LcovScraping.__get_lcov_coverage(html)
            filename = output_dir / html.parent.name / "coverage.csv"
            save_to_csv(filename, coverages[0].keys(), coverages)
