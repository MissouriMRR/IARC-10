from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any


def coerce_to_row_value(value: Any) -> Any:
    if isinstance(value, (dict, list)):
        return json.dumps(value, ensure_ascii=False)
    return value


def json_to_rows(json_path: Path) -> tuple[list[str], list[dict[str, Any]]]:
    with json_path.open("r", encoding="utf-8") as f:
        payload = json.load(f)

    title = payload.get("title", "")
    data = payload.get("data", {})

    if not isinstance(data, dict):
        raise ValueError(f"Expected 'data' to be an object in {json_path.name}")

    keys = list(data.keys())
    lengths = [len(v) for v in data.values() if isinstance(v, list)]
    row_count = max(lengths, default=0)

    headers = ["sourceFile", "title", "rowIndex"] + keys
    rows: list[dict[str, Any]] = []

    for idx in range(row_count):
        row: dict[str, Any] = {
            "sourceFile": json_path.name,
            "title": title,
            "rowIndex": idx,
        }

        for key in keys:
            value = data.get(key, "")
            if isinstance(value, list):
                row[key] = coerce_to_row_value(value[idx]) if idx < len(value) else ""
            else:
                # Non-list fields are repeated on first row only.
                row[key] = coerce_to_row_value(value) if idx == 0 else ""

        rows.append(row)

    return headers, rows


def write_csv(path: Path, headers: list[str], rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Create CSV spreadsheets from each JSON in Logs/Range_Test."
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=Path("Logs/Range_Test"),
        help="Directory containing range-test JSON files.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("Logs/Range_Test/Spreadsheets"),
        help="Directory where CSV files will be written.",
    )
    parser.add_argument(
        "--combined-name",
        type=str,
        default="all_range_tests.csv",
        help="Filename for combined CSV output.",
    )
    args = parser.parse_args()

    if not args.input_dir.exists():
        raise FileNotFoundError(f"Input directory not found: {args.input_dir}")

    json_files = sorted(args.input_dir.glob("*.json"))
    if not json_files:
        print(f"No JSON files found in {args.input_dir}")
        return

    all_rows: list[dict[str, Any]] = []
    combined_headers: list[str] | None = None
    written = 0

    for json_file in json_files:
        try:
            headers, rows = json_to_rows(json_file)
        except Exception as exc:
            print(f"Skipping {json_file.name}: {exc}")
            continue

        out_csv = args.output_dir / f"{json_file.stem}.csv"
        write_csv(out_csv, headers, rows)
        written += 1
        print(f"Wrote {out_csv} ({len(rows)} rows)")

        if combined_headers is None:
            combined_headers = headers
        if headers == combined_headers:
            all_rows.extend(rows)
        else:
            print(
                f"Skipping {json_file.name} from combined CSV due to header mismatch."
            )

    if combined_headers and all_rows:
        combined_csv = args.output_dir / args.combined_name
        write_csv(combined_csv, combined_headers, all_rows)
        print(f"Wrote combined CSV: {combined_csv} ({len(all_rows)} rows)")

    print(f"Done. Generated {written} per-file CSV spreadsheet(s).")


if __name__ == "__main__":
    main()
