import os
import glob
import re
from collections import defaultdict

import numpy as np
import pandas as pd
from pandas.errors import EmptyDataError
import matplotlib.pyplot as plt

phase = 0
task_set = "_0_"
col_to_plot = "fitness_task_0"
min_gen_cutoff = 50

COLORS = {
    "stateless": "#1f77b4",
    "stateful": "#ff7f0e",
}

HERE = os.path.dirname(os.path.abspath(__file__))

# If this script is stored inside paper_data:
#base_path = HERE

# Otherwise use:
base_path = "/home/dhilan/tpg_codebase/tpg/experiments/paper_data"


def get_experiment_type(exp_name):
    name = exp_name.lower()
    if "stateless" in name:
        return "stateless"
    if "stateful" in name:
        return "stateful"
    return "unknown"


def discover_experiments(base_dir):
    pattern = os.path.join(base_dir, "gazebo_turtlebot4_*")
    return [p for p in sorted(glob.glob(pattern)) if os.path.isdir(p)]


def parse_seed_rep(path):
    """
    selection.<seed>.<rep>.csv -> (seed:int, rep:int)
    """
    m = re.search(r"selection\.(\d+)\.(\d+)\.csv$", os.path.basename(path))
    if not m:
        return None, None
    return int(m.group(1)), int(m.group(2))


def load_curves_by_seed(exp_dir):
    """
    Returns dict: seed -> (gens, mean_curve, max_curve, std_curve)
    If a seed has multiple reps, aggregate those reps.
    """
    pattern = os.path.join(exp_dir, "selection.*.*.csv")
    files = sorted(glob.glob(pattern))

    curves = defaultdict(list)

    for f in files:
        seed, rep = parse_seed_rep(f)
        if seed is None:
            continue

        try:
            df = pd.read_csv(f)
        except (EmptyDataError, pd.errors.ParserError, UnicodeDecodeError):
            continue

        if df.shape[0] == 0:
            continue

        df.columns = df.columns.astype(str).str.strip()

        required = {"phase", "task_set", col_to_plot}
        if not required.issubset(df.columns):
            print(f"[WARN] Missing required columns in {f}")
            continue

        dff = df[(df["phase"] == phase) & (df["task_set"] == task_set)]
        if dff.empty:
            continue

        if "generation" in dff.columns:
            dff = dff.sort_values("generation")

        y = pd.to_numeric(dff[col_to_plot], errors="coerce").dropna().to_numpy()
        if len(y) < min_gen_cutoff:
            continue

        curves[seed].append(y)

    out = {}
    for seed, ys in curves.items():
        if not ys:
            continue

        min_len = min(len(y) for y in ys)
        data = np.vstack([y[:min_len] for y in ys])   # shape (n_rep, min_len)
        gens = np.arange(min_len)

        mean = data.mean(axis=0)
        mx = data.max(axis=0)
        std = data.std(axis=0) if data.shape[0] > 1 else np.zeros_like(mean)

        out[seed] = (gens, mean, mx, std)

    return out


def add_seed_curve(gens, mean, mx, std, label, color=None, shade=False):
    ax = plt.gca()
    if shade and np.any(std):
        ax.fill_between(gens, mean - 0.5 * std, mean + 0.5 * std, alpha=0.25, color=color)
    ax.plot(gens, mean, label=label, linewidth=1.5, color=color)
    ax.plot(gens, mx, linestyle="dashed", linewidth=1.2, color=color)


if __name__ == "__main__":
    plt.figure(figsize=(10, 7))

    all_experiments = discover_experiments(base_path)
    if not all_experiments:
        raise RuntimeError(f"No experiment folders found in: {base_path}")

    global_min_len = None
    all_series = []  # list of (label, gens, mean, mx, std, color)

    for exp_path in all_experiments:
        exp_name = os.path.basename(exp_path)
        exp_type = get_experiment_type(exp_name)

        if exp_type == "unknown":
            print(f"[WARN] Skipping unknown experiment type: {exp_name}")
            continue

        exp_dir = os.path.join(exp_path, "logs", "selection")
        if not os.path.isdir(exp_dir):
            print(f"[WARN] Missing selection directory: {exp_dir}")
            continue

        seed_curves = load_curves_by_seed(exp_dir)

        if not seed_curves:
            print(f"[WARN] No usable curves in {exp_name}")
            continue

        print(f"[INFO] {exp_name}: {len(seed_curves)} seed curve(s)")

        for seed, (gens, mean, mx, std) in sorted(seed_curves.items()):
            if global_min_len is None:
                global_min_len = len(gens)
            else:
                global_min_len = min(global_min_len, len(gens))

            label = f"{exp_type} | seed {seed}"
            color = COLORS.get(exp_type, None)
            all_series.append((label, gens, mean, mx, std, color))

    if not all_series:
        raise RuntimeError("No curves found across discovered experiments.")

    for label, gens, mean, mx, std, color in all_series:
        L = global_min_len
        g = gens[:L]
        if phase > 0:
            g = g * 100
        add_seed_curve(g, mean[:L], mx[:L], std[:L], label, color=color, shade=False)

    plt.legend(loc="lower right", fontsize=8, ncol=1)
    plt.xlabel("Generations")
    plt.ylabel(col_to_plot)
    plt.title("Per-Seed Fitness Curves for Stateful and Stateless Experiments")

    ax = plt.gca()
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    out_dir = os.path.join(base_path, "gazebo_curve_plots")
    os.makedirs(out_dir, exist_ok=True)

    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "fitness_plot.pdf"), bbox_inches="tight")
    plt.savefig(os.path.join(out_dir, "fitness_plot.png"), dpi=300, bbox_inches="tight")
    plt.show()
