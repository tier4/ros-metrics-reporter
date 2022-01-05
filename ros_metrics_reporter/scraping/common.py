import csv
import json
from pathlib import Path


def save_to_csv(filename, field_names, data, mode="w"):
    """Save to csv"""
    with open(filename, mode) as f:
        writer = csv.DictWriter(f, fieldnames=field_names)
        if mode == "w":
            writer.writeheader()
        writer.writerows(data)


def save_to_json(filename: Path, data: list):
    """Save to json"""
    if filename.exists():
        with open(filename, "r") as f:
            read_data = json.load(f)
            data.extend(read_data)

    with open(filename, "w") as f:
        f.write(json.dumps(data, indent=4))
