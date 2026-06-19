#!/usr/bin/env python3

#usage:
    #python3 viz/plot_suboptimality.py
    #python3 viz/plot_suboptimality.py --csv viz/data/suboptimality.csv \
     #                                 --out-dir viz/out


import argparse
import csv
import os
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_csv(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            row["makespan_ratio"] = float(row["makespan_ratio"])
            row["soc_ratio"] = float(row["soc_ratio"])
            row["astar_time_ms"] = float(row["astar_time_ms"])
            row["pibt_time_ms"] = float(row["pibt_time_ms"])
            rows.append(row)
    return rows


def group_by_zone(rows, key):
    out = defaultdict(list)
    for r in rows:
        out[r["zone"]].append(r[key])
    return out


def plot_ratio(rows, out_path):
    """Side-by-side makespan-ratio and SOC-ratio box-plots, one box per zone."""
    zones = sorted({r["zone"] for r in rows})
    mksp = [group_by_zone(rows, "makespan_ratio")[z] for z in zones]
    soc = [group_by_zone(rows, "soc_ratio")[z] for z in zones]

    fig, axes = plt.subplots(1, 2, figsize=(11, 5), sharey=True)
    for ax, data, title in [
        (axes[0], mksp, "Makespan ratio  (PIBT / A*)"),
        (axes[1], soc, "Sum-of-costs ratio  (PIBT / A*)"),
    ]:
        ax.boxplot(
            data,
            labels=zones,
            showfliers=True,
            patch_artist=True,
            boxprops=dict(facecolor="#cfe0fc", color="#2b6cf0"),
            medianprops=dict(color="#0b3d91", linewidth=2),
            whiskerprops=dict(color="#2b6cf0"),
            capprops=dict(color="#2b6cf0"),
            flierprops=dict(marker="o", markerfacecolor="#d1495b",
                            markeredgecolor="#d1495b", markersize=4),
        )
        ax.axhline(1.0, color="#888", linestyle="--", linewidth=1)
        ax.set_title(title)
        ax.set_ylabel("ratio  (1.0 = optimal)")
        ax.grid(True, axis="y", alpha=0.3)
        ax.set_ylim(bottom=0.95)
    fig.suptitle(
        "PIBT sub-optimality versus optimal joint-state A* (50 random instances / zone)",
        fontsize=12,
    )
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_time(rows, out_path):
    """Paired A*-time vs PIBT-time box-plots per zone, log y."""
    zones = sorted({r["zone"] for r in rows})
    astar = [group_by_zone(rows, "astar_time_ms")[z] for z in zones]
    pibt = [group_by_zone(rows, "pibt_time_ms")[z] for z in zones]

    fig, ax = plt.subplots(figsize=(10, 5))
    positions_astar = [i * 3 + 0 for i in range(len(zones))]
    positions_pibt = [i * 3 + 1 for i in range(len(zones))]

    bp_a = ax.boxplot(astar, positions=positions_astar, widths=0.7, patch_artist=True,
                      boxprops=dict(facecolor="#f5c0c6", color="#d1495b"),
                      medianprops=dict(color="#7a1a25", linewidth=2),
                      whiskerprops=dict(color="#d1495b"),
                      capprops=dict(color="#d1495b"),
                      flierprops=dict(marker="o", markerfacecolor="#d1495b",
                                      markeredgecolor="#d1495b", markersize=3))
    bp_b = ax.boxplot(pibt, positions=positions_pibt, widths=0.7, patch_artist=True,
                      boxprops=dict(facecolor="#cfe0fc", color="#2b6cf0"),
                      medianprops=dict(color="#0b3d91", linewidth=2),
                      whiskerprops=dict(color="#2b6cf0"),
                      capprops=dict(color="#2b6cf0"),
                      flierprops=dict(marker="o", markerfacecolor="#2b6cf0",
                                      markeredgecolor="#2b6cf0", markersize=3))
    ax.set_yscale("log")
    ax.set_xticks([i * 3 + 0.5 for i in range(len(zones))])
    ax.set_xticklabels(zones)
    ax.set_ylabel("runtime (ms, log scale)")
    ax.set_title("Solver runtime distribution per zone  (50 random instances / zone)")
    ax.grid(True, axis="y", which="both", alpha=0.3)
    ax.legend([bp_a["boxes"][0], bp_b["boxes"][0]], ["A*", "PIBT"], loc="upper left")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_hist(rows, out_path):
    """Histogram of pooled makespan ratio across all instances."""
    ratios = [r["makespan_ratio"] for r in rows]
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.hist(ratios, bins=30, color="#cfe0fc", edgecolor="#2b6cf0")
    ax.axvline(1.0, color="#888", linestyle="--", linewidth=1, label="optimum (1.0)")
    ax.set_xlabel("makespan ratio  (PIBT / A*)")
    ax.set_ylabel("count of instances")
    ax.set_title(f"Distribution of PIBT makespan ratio over {len(ratios)} random instances")
    ax.legend()
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="viz/data/suboptimality.csv")
    ap.add_argument("--out-dir", default="viz/out")
    args = ap.parse_args()

    rows = read_csv(args.csv)
    if not rows:
        raise SystemExit(f"no rows read from {args.csv}")
    os.makedirs(args.out_dir, exist_ok=True)

    plot_ratio(rows, os.path.join(args.out_dir, "suboptimality_ratio.png"))
    plot_time(rows, os.path.join(args.out_dir, "suboptimality_time.png"))
    plot_hist(rows, os.path.join(args.out_dir, "suboptimality_hist.png"))
    print(f"wrote 3 PNGs to {args.out_dir}/ from {len(rows)} CSV rows")


if __name__ == "__main__":
    main()
