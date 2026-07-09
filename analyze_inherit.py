#!/usr/bin/env python3
"""Analyze the per-expansion inheritance/reward log produced by
`build/main ... --inherit_log <path>`.

CSV columns: loop_cnt, region, inherit, reward, outcome, h, g, f
  inherit  : priority-inheritance count during this expansion (X)
  reward   : reward for this expansion  (0/0.05/0.1/0.5/1.0)
  outcome  : 0=fail 1=known 2=new 3=f_improve 4=better
  h,g,f    : heuristic/cost of the expanded node (units follow --objective)

It prints Pearson + Spearman correlations among {inherit, reward, h, g, f} and
saves a multi-panel figure. Reward is discrete/near-categorical, so beyond the
raw correlation it also shows: mean reward by inherit bucket, the outcome mix by
inherit bucket, inherit distribution per outcome, and reward vs h/g/f quantiles.

Usage:
    python analyze_inherit.py inherit.csv [more.csv ...] [-o out.png]
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUTCOME_NAMES = {0: "fail", 1: "known", 2: "new", 3: "f_improve", 4: "better"}
NUMCOLS = ["inherit", "reward", "h", "g", "f"]


def load(paths):
    frames = []
    for p in paths:
        if not os.path.exists(p):
            sys.exit(f"file not found: {p}")
        df = pd.read_csv(p)
        df["__src"] = os.path.basename(p)
        frames.append(df)
    df = pd.concat(frames, ignore_index=True)
    missing = [c for c in ["inherit", "reward", "outcome", "h", "g", "f"]
               if c not in df.columns]
    if missing:
        sys.exit(f"missing columns in CSV: {missing}")
    return df


def bucket_inherit(s, cap=6):
    b = s.clip(upper=cap).astype(int)
    labels = [str(i) for i in range(cap)] + [f">={cap}"]
    return b, labels


def print_report(df):
    n = len(df)
    print(f"\n=== samples: {n} ===")
    print("\n-- column stats --")
    print(df[NUMCOLS].describe().T[["mean", "std", "min", "max"]].to_string())

    print("\n-- outcome breakdown --")
    for o, cnt in df["outcome"].value_counts().sort_index().items():
        sub = df[df.outcome == o]
        print(f"  {o} {OUTCOME_NAMES.get(o,o):9s}: n={cnt:8d} "
              f"({100*cnt/n:5.1f}%)  mean_inherit={sub['inherit'].mean():.3f}  "
              f"mean_reward={sub['reward'].mean():.3f}")

    print("\n-- Pearson correlation --")
    print(df[NUMCOLS].corr(method="pearson").round(3).to_string())
    print("\n-- Spearman correlation --")
    print(df[NUMCOLS].corr(method="spearman").round(3).to_string())

    print("\n-- mean reward by inherit bucket --")
    b, labels = bucket_inherit(df["inherit"])
    grp = df.groupby(b)["reward"]
    for k in sorted(grp.groups):
        r = grp.get_group(k)
        lab = labels[k] if k < len(labels) else str(k)
        print(f"  inherit={lab:>4}: n={len(r):8d}  "
              f"mean_reward={r.mean():.4f}  sem={r.sem():.4f}")


def _heatmap(ax, corr, title):
    im = ax.imshow(corr.values, vmin=-1, vmax=1, cmap="coolwarm")
    ax.set_xticks(range(len(corr)))
    ax.set_yticks(range(len(corr)))
    ax.set_xticklabels(corr.columns, rotation=45, ha="right")
    ax.set_yticklabels(corr.index)
    for i in range(len(corr)):
        for j in range(len(corr)):
            ax.text(j, i, f"{corr.values[i,j]:.2f}", ha="center", va="center",
                    fontsize=8,
                    color="white" if abs(corr.values[i, j]) > 0.5 else "black")
    ax.set_title(title)
    plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)


def make_figure(df, out):
    fig, axes = plt.subplots(2, 3, figsize=(16, 9))

    # 1) Pearson heatmap
    _heatmap(axes[0, 0], df[NUMCOLS].corr(method="pearson"),
             "Pearson correlation")

    # 2) mean reward by inherit bucket
    b, labels = bucket_inherit(df["inherit"])
    grp = df.groupby(b)["reward"]
    ks = sorted(grp.groups)
    means = [grp.get_group(k).mean() for k in ks]
    sems = [grp.get_group(k).sem() for k in ks]
    xlab = [labels[k] for k in ks]
    ax = axes[0, 1]
    ax.bar(range(len(ks)), means, yerr=sems, capsize=3, color="#4C72B0")
    ax.set_xticks(range(len(ks)))
    ax.set_xticklabels(xlab)
    ax.set_xlabel("inherit count")
    ax.set_ylabel("mean reward")
    ax.set_title("mean reward by inherit bucket")
    ax.grid(True, axis="y", alpha=0.3)

    # 3) outcome share by inherit bucket (stacked)
    ax = axes[0, 2]
    ct = pd.crosstab(b, df["outcome"], normalize="index")
    ct = ct.reindex(sorted(ct.index))
    bottom = np.zeros(len(ct))
    colors = {0: "#999999", 1: "#DD8452", 2: "#4C72B0", 3: "#55A868",
              4: "#C44E52"}
    for o in sorted(ct.columns):
        ax.bar(range(len(ct)), ct[o].values, bottom=bottom,
               label=f"{o}:{OUTCOME_NAMES.get(o,o)}", color=colors.get(o))
        bottom += ct[o].values
    ax.set_xticks(range(len(ct)))
    ax.set_xticklabels([labels[k] for k in ct.index])
    ax.set_xlabel("inherit count")
    ax.set_ylabel("outcome share")
    ax.set_title("outcome mix by inherit bucket")
    ax.legend(fontsize=7, loc="lower right")

    # 4) inherit distribution by outcome (boxplot)
    ax = axes[1, 0]
    outs = sorted(df["outcome"].unique())
    data = [df[df.outcome == o]["inherit"].values for o in outs]
    ax.boxplot(data, showfliers=False)
    ax.set_xticks(range(1, len(outs) + 1))
    ax.set_xticklabels([OUTCOME_NAMES.get(o, o) for o in outs])
    ax.set_ylabel("inherit count")
    ax.set_title("inherit by outcome")
    ax.grid(True, axis="y", alpha=0.3)

    # 5) mean reward vs h/g/f quantile
    ax = axes[1, 1]
    for col, c in zip(["h", "g", "f"], ["#4C72B0", "#55A868", "#C44E52"]):
        try:
            q = pd.qcut(df[col], 10, labels=False, duplicates="drop")
        except ValueError:
            continue
        m = df.groupby(q)["reward"].mean()
        ax.plot(np.linspace(0, 1, len(m)), m.values, marker="o", label=col)
    ax.set_xlabel("quantile of h/g/f (0=low,1=high)")
    ax.set_ylabel("mean reward")
    ax.set_title("mean reward vs h/g/f quantile")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 6) mean inherit vs f quantile (interaction / confounder view)
    ax = axes[1, 2]
    for col, c in zip(["h", "g", "f"], ["#4C72B0", "#55A868", "#C44E52"]):
        try:
            q = pd.qcut(df[col], 10, labels=False, duplicates="drop")
        except ValueError:
            continue
        m = df.groupby(q)["inherit"].mean()
        ax.plot(np.linspace(0, 1, len(m)), m.values, marker="o", label=col)
    ax.set_xlabel("quantile of h/g/f (0=low,1=high)")
    ax.set_ylabel("mean inherit")
    ax.set_title("mean inherit vs h/g/f quantile")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.suptitle(f"inherit / reward analysis  (n={len(df)})", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out, dpi=140)
    print(f"\nsaved figure: {out}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+", help="inherit_log CSV file(s)")
    ap.add_argument("-o", "--out", default=None, help="output PNG path")
    ap.add_argument("--drop-fail", action="store_true",
                    help="exclude outcome=0 (failed expansions)")
    args = ap.parse_args()

    df = load(args.csv)
    if args.drop_fail:
        df = df[df.outcome != 0].reset_index(drop=True)
        print("(dropped outcome=0 fail rows)")

    print_report(df)
    out = args.out or (os.path.splitext(args.csv[0])[0] + "_analysis.png")
    make_figure(df, out)


if __name__ == "__main__":
    main()
