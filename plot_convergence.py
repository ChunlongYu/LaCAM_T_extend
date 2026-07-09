#!/usr/bin/env python3
"""Plot UB/LB convergence from a LaCAM* convergence log.

The log is produced by `build/main ... --conv_log <path>` and has columns:
    time_ms, ub, lb, gap
`ub` and `gap` are empty on rows recorded before the first feasible solution.

Usage:
    python plot_convergence.py <conv_log.csv> [-o out.png] [--title "..."]
    python plot_convergence.py a.csv b.csv -o cmp.png   # overlay several runs
"""
import argparse
import csv
import os
import sys

import matplotlib

matplotlib.use("Agg")  # headless / file output
import matplotlib.pyplot as plt


def read_log(path):
    """Return dict of parallel lists: t, ub, lb, gap (ub/gap may contain None)."""
    t, ub, lb, gap = [], [], [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["time_ms"]))
            ub.append(float(row["ub"]) if row.get("ub") else None)
            lb.append(float(row["lb"]) if row.get("lb") else None)
            gap.append(float(row["gap"]) if row.get("gap") else None)
    return {"t": t, "ub": ub, "lb": lb, "gap": gap}


def first_solution_time(d):
    for ti, ui in zip(d["t"], d["ub"]):
        if ui is not None:
            return ti
    return None


def plot(logs, labels, out_path, title):
    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(9, 7), sharex=True, gridspec_kw={"height_ratios": [3, 1]}
    )
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    for i, (d, label) in enumerate(zip(logs, labels)):
        c = colors[i % len(colors)]
        # upper bound (incumbent) — only defined after first solution
        ub_t = [ti for ti, ui in zip(d["t"], d["ub"]) if ui is not None]
        ub_v = [ui for ui in d["ub"] if ui is not None]
        # lower bound — defined throughout
        lb_t, lb_v = d["t"], d["lb"]

        single = len(logs) == 1
        ub_label = "UB (incumbent)" if single else f"{label} UB"
        lb_label = "LB (cheap global)" if single else f"{label} LB"

        if ub_v:
            ax1.step(ub_t, ub_v, where="post", color=c,
                     linewidth=2, label=ub_label)
        ax1.step(lb_t, lb_v, where="post", color=c, linewidth=2,
                 linestyle="--", label=lb_label)

        # shade the optimality gap for a single run
        if single and ub_v:
            # align lb onto ub timeline for fill
            ax1.fill_between(lb_t, lb_v,
                             [ub_v[-1]] * len(lb_v),
                             step="post", color=c, alpha=0.08)

        # mark first solution
        fst = first_solution_time(d)
        if fst is not None:
            ax1.axvline(fst, color=c, alpha=0.3, linestyle=":")

        # gap panel
        g_t = [ti for ti, gi in zip(d["t"], d["gap"]) if gi is not None]
        g_v = [gi * 100 for gi in d["gap"] if gi is not None]
        if g_v:
            ax2.step(g_t, g_v, where="post", color=c, linewidth=2,
                     label=label if len(logs) > 1 else None)

    ax1.set_ylabel("cost")
    ax1.set_title(title)
    ax1.legend(loc="best", fontsize=9)
    ax1.grid(True, alpha=0.3)

    ax2.set_ylabel("gap (%)")
    ax2.set_xlabel("time (ms)")
    ax2.grid(True, alpha=0.3)
    ax2.axhline(0, color="gray", linewidth=0.8)
    if len(logs) > 1:
        ax2.legend(loc="best", fontsize=9)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"saved: {out_path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", help="one or more conv_log CSV files")
    ap.add_argument("-o", "--out", default=None, help="output image path (.png)")
    ap.add_argument("--title", default=None, help="plot title")
    args = ap.parse_args()

    for p in args.logs:
        if not os.path.exists(p):
            sys.exit(f"file not found: {p}")

    logs = [read_log(p) for p in args.logs]
    labels = [os.path.splitext(os.path.basename(p))[0] for p in args.logs]
    out = args.out or (os.path.splitext(args.logs[0])[0] + ".png")
    title = args.title or ("UB/LB convergence" if len(logs) > 1
                           else f"UB/LB convergence — {labels[0]}")
    plot(logs, labels, out, title)


if __name__ == "__main__":
    main()
