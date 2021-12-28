from dataclasses import dataclass
from pathlib import Path
from typing import List, Dict


@dataclass
class StaticPageInput:
    input_dir: Path
    hugo_root_dir: Path
    hugo_template_dir: Path
    lcov_result_path: Path
    lizard_result_path: Path
    tidy_result_path: Path
    base_url: str
    title: str
    contributors: Dict[str, List]
    test_label: List[str]
