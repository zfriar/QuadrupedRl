#!/usr/bin/env python3
"""Create a Shields.io-compatible JSON badge from coverage.xml.

Writes badges/coverage.json which can be published on the gh-pages branch
and referenced via shields.io endpoint:

  https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/OWNER/REPO/gh-pages/badges/coverage.json

This script expects `coverage.xml` (coverage.py/pytest-cov) in the repo root.
"""
import json
import pathlib
import sys
import xml.etree.ElementTree


def main():
    cov_path = pathlib.Path("coverage.xml")
    if not cov_path.exists():
        print("coverage.xml not found; run tests with coverage first", file=sys.stderr)
        sys.exit(2)

    tree = xml.etree.ElementTree.parse(str(cov_path))
    root = tree.getroot()
    # coverage.py XML may store line-rate on the root element or the packages element.
    line_rate = root.attrib.get("line-rate")
    if line_rate is None:
        # fallback: try to find any element that has a 'line-rate' attribute
        line_rate = None
        for el in root.iter():
            if "line-rate" in el.attrib:
                line_rate = el.attrib.get("line-rate")
                break

    try:
        pct = int(round(float(line_rate) * 100))
    except Exception:
        pct = 0

    if pct < 80:
        color = "red"
    elif pct < 90:
        color = "yellow"
    elif pct < 95:
        color = "yellowgreen"
    else:
        color = "brightgreen"

    payload = {
        "schemaVersion": 1,
        "label": "coverage",
        "message": f"{pct}%",
        "color": color,
    }

    out_dir = pathlib.Path("badges")
    out_dir.mkdir(exist_ok=True)
    out_file = out_dir / "coverage.json"
    out_file.write_text(json.dumps(payload))
    print(f"Wrote {out_file} -> {payload}")


if __name__ == "__main__":
    main()
