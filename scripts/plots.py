#!/usr/bin/env python3
"""Plot left/right force-torque axes from FT log CSV files."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Sequence

import matplotlib.pyplot as plt


VALID_AXES = ("fx", "fy", "fz", "tx", "ty", "tz")
AXIS_UNITS = {
    "fx": "N",
    "fy": "N",
    "fz": "N",
    "tx": "N*m",
    "ty": "N*m",
    "tz": "N*m",
}


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot left/right FT axis values over time from a log CSV.",
    )
    parser.add_argument(
        "file",
        type=Path,
        help="Path to FT log CSV file (for example: log/ft_log_20260422T233811Z.csv).",
    )
    parser.add_argument(
        "axis",
        nargs="+",
        choices=VALID_AXES,
        help="Axis name(s) to plot. Choices: fx fy fz tx ty tz.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Optional path to save the figure (e.g., plot.png).",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open an interactive window; useful with --output on headless systems.",
    )
    return parser.parse_args()


def _parse_time_seconds(row: Dict[str, str]) -> float | None:
    unix_val = row.get("timestamp_unix_s", "")
    if unix_val:
        try:
            return float(unix_val)
        except ValueError:
            pass

    iso_val = row.get("timestamp_utc", "")
    if not iso_val:
        return None
    try:
        return datetime.fromisoformat(iso_val).timestamp()
    except ValueError:
        return None


def _parse_float(row: Dict[str, str], key: str) -> float | None:
    raw = row.get(key, "")
    if raw == "":
        return None
    try:
        return float(raw)
    except ValueError:
        return None


def _load_series(csv_path: Path, axes: Sequence[str]) -> Dict[str, List[float]]:
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    data: Dict[str, List[float]] = {"time": []}
    for axis in axes:
        data[f"left_{axis}"] = []
        data[f"right_{axis}"] = []

    with csv_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        expected_columns = {"timestamp_unix_s", "timestamp_utc"}
        for axis in axes:
            expected_columns.add(f"left_{axis}")
            expected_columns.add(f"right_{axis}")

        missing_columns = [c for c in expected_columns if c not in (reader.fieldnames or [])]
        if missing_columns:
            missing_str = ", ".join(sorted(missing_columns))
            raise ValueError(f"CSV is missing expected columns: {missing_str}")

        for row in reader:
            t = _parse_time_seconds(row)
            if t is None:
                continue

            row_values: Dict[str, float] = {}
            valid = True
            for axis in axes:
                left_key = f"left_{axis}"
                right_key = f"right_{axis}"
                left_val = _parse_float(row, left_key)
                right_val = _parse_float(row, right_key)
                if left_val is None or right_val is None:
                    valid = False
                    break
                row_values[left_key] = left_val
                row_values[right_key] = right_val

            if not valid:
                continue

            data["time"].append(t)
            for key, value in row_values.items():
                data[key].append(value)

    if not data["time"]:
        raise ValueError("No valid rows found in CSV after parsing.")

    t0 = data["time"][0]
    data["time"] = [t - t0 for t in data["time"]]
    return data


def _plot_axes(csv_path: Path, axes: Sequence[str], data: Dict[str, List[float]], output: Path | None, no_show: bool) -> None:
    num_axes = len(axes)
    fig, axs = plt.subplots(num_axes, 1, figsize=(10, 3.2 * num_axes), sharex=True)
    if num_axes == 1:
        axs = [axs]

    t = data["time"]
    for ax_obj, axis_name in zip(axs, axes):
        left_key = f"left_{axis_name}"
        right_key = f"right_{axis_name}"
        unit = AXIS_UNITS[axis_name]

        ax_obj.plot(t, data[left_key], label=left_key, linewidth=1.3)
        ax_obj.plot(t, data[right_key], label=right_key, linewidth=1.3)
        ax_obj.set_ylabel(f"{axis_name} [{unit}]")
        ax_obj.grid(True, alpha=0.3)
        ax_obj.legend(loc="best")

    axs[-1].set_xlabel("time [s] from start")
    fig.suptitle(f"FT log: {csv_path.name}", fontsize=12)
    fig.tight_layout()

    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, dpi=150, bbox_inches="tight")
        print(f"Saved plot to: {output}")

    if not no_show:
        plt.show()
    else:
        plt.close(fig)


def main() -> int:
    args = _parse_args()
    axes = list(dict.fromkeys(args.axis))
    data = _load_series(args.file, axes)
    _plot_axes(args.file, axes, data, args.output, args.no_show)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
