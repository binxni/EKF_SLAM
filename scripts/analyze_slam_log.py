#!/usr/bin/env python3
"""Utility for analyzing and visualizing SLAM logs stored in a CSV file."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt
import pandas as pd


def load_log(path: Path) -> pd.DataFrame:
    """Load a SLAM log CSV file."""
    if not path.exists():
        raise FileNotFoundError(f"Could not find log file: {path}")
    return pd.read_csv(path)


def summarize(df: pd.DataFrame) -> pd.DataFrame:
    """Return basic statistics for numeric columns."""
    return df.describe()


def plot_trajectory(df: pd.DataFrame, out_dir: Path) -> None:
    """Plot the x-y trajectory if available."""
    if {"x", "y"} <= set(df.columns):
        out_dir.mkdir(parents=True, exist_ok=True)
        plt.figure()
        plt.plot(df["x"].to_numpy(), df["y"].to_numpy(), label="trajectory")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.title("SLAM Trajectory")
        plt.axis("equal")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(out_dir / "trajectory.png")
        plt.close()


def plot_time_series(df: pd.DataFrame, columns: Iterable[str], out_dir: Path) -> None:
    """Plot selected columns over time."""
    out_dir.mkdir(parents=True, exist_ok=True)
    for col in columns:
        if col not in df.columns:
            continue
        plt.figure()
        plt.plot(df.index.to_numpy(), df[col].to_numpy())
        plt.xlabel("index")
        plt.ylabel(col)
        plt.title(f"{col} over time")
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(out_dir / f"{col}.png")
        plt.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "csv",
        type=Path,
        nargs="?",
        default=Path("slam_log.csv"),
        help="Path to the SLAM log CSV file",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("plots"),
        help="Directory to save generated plots",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    df = load_log(args.csv)
    stats = summarize(df)
    print(stats)
    plot_trajectory(df, args.out)
    numeric_cols = df.select_dtypes(include="number").columns
    plot_time_series(df, numeric_cols, args.out)


if __name__ == "__main__":
    main()
