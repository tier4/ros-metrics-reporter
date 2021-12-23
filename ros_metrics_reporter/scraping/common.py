import csv


def save_to_csv(filename, field_names, data, mode="w"):
    """Save to csv"""
    with open(filename, mode) as f:
        writer = csv.DictWriter(f, fieldnames=field_names)
        if mode == "w":
            writer.writeheader()
        writer.writerows(data)
