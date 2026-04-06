import os
import re
import glob
from collections import defaultdict

import numpy as np
import pandas as pd
from pandas.errors import EmptyDataError
import matplotlib.pyplot as plt

# ------------------------
# CONFIG
# ------------------------
phase = 0
task_set = "_0_"
col_to_plot = "effective_program_instruction_count"
min_gen_cutoff = 50

COLORS = {
    "stateless": "#1f77b4",
    "stateful": "#ff7f0e",
}

plt.rcParams.update({
    "font.size": 13,
    "axes.titlesize": 15,
    "axes.labelsize": 14,
    "xtick.labelsize": 12,
    "ytick.labelsize": 12,
    "legend.fontsize": 12,
    "axes.linewidth": 1.2,
    "lines.linewidth": 2.4,
    "xtick.major.width": 1.1,
    "ytick.major.width": 1.1,
    "xtick.major.size": 5,
    "ytick.major.size": 5,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
})

# ------------------------
# PATHS
# ------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
base_path = "/home/dhilan/tpg_codebase/tpg/experiments/paper_data"

# ------------------------
# HELPERS
# ------------------------
def get_experiment_type(exp_name: str) -> str:
    name = exp_name.lower()
    if "stateless" in name:
        return "stateless"
    if "stateful" in name:
        return "stateful"
    return "unknown"

def discover_experiments(base_dir: str):
    pattern = os.path.join(base_dir, "gazebo_turtlebot4_*")
    return [p for p in sorted(glob.glob(pattern)) if os.path.isdir(p)]

def parse_seed_rep(path: str):
    m = re.search(r"selection\.(\d+)\.(\d+)\.csv$", os.path.basename(path))
    if not m:
        return None, None
    return int(m.group(1)), int(m.group(2))

def load_seed_curves_from_selection_dir(selection_dir: str):
    pattern = os.path.join(selection_dir, "selection.*.*.csv")
    files = sorted(glob.glob(pattern))

    curves_by_seed = defaultdict(list)

    for f in files:
        seed, rep = parse_seed_rep(f)
        if seed is None:
            continue

        try:
            df = pd.read_csv(f)
        except (EmptyDataError, pd.errors.ParserError, UnicodeDecodeError):
            print(f"[WARN] Could not read: {f}")
            continue

        if df.empty:
            continue

        df.columns = df.columns.astype(str).str.strip()

        required = {"phase", "task_set", col_to_plot}
        if not required.issubset(df.columns):
            print(f"[WARN] Missing required columns in {f}")
            continue

        dff = df[(df["phase"] == phase) & (df["task_set"] == task_set)].copy()
        if dff.empty:
            continue

        if "generation" in dff.columns:
            dff = dff.sort_values("generation")

        y = pd.to_numeric(dff[col_to_plot], errors="coerce").dropna().to_numpy()

        if len(y) < min_gen_cutoff:
            continue

        curves_by_seed[seed].append(y)

    seed_curves = []
    for seed, reps in curves_by_seed.items():
        if not reps:
            continue

        min_len = min(len(r) for r in reps)
        data = np.vstack([r[:min_len] for r in reps])

        # average repetitions within each seed
        seed_curve = data.mean(axis=0)
        seed_curves.append(seed_curve)

    return seed_curves

def aggregate_group(experiment_dirs):
    all_curves = []

    for exp_dir in experiment_dirs:
        selection_dir = os.path.join(exp_dir, "logs", "selection")
        if not os.path.isdir(selection_dir):
            print(f"[WARN] Missing selection directory: {selection_dir}")
            continue

        seed_curves = load_seed_curves_from_selection_dir(selection_dir)
        print(f"[INFO] {os.path.basename(exp_dir)} -> {len(seed_curves)} seed curve(s)")
        all_curves.extend(seed_curves)

    if len(all_curves) == 0:
        return np.array([]), np.array([]), np.array([]), 0

    min_len = min(len(c) for c in all_curves)
    data = np.vstack([c[:min_len] for c in all_curves])

    gens = np.arange(min_len)
    mean = data.mean(axis=0)
    std = data.std(axis=0)

    return gens, mean, std, data.shape[0]

def plot_mean_with_std(ax, gens, mean, std, label, color, alpha=0.18):
    ax.plot(gens, mean, label=label, color=color, linewidth=2.4)
    ax.fill_between(gens, mean - std, mean + std, color=color, alpha=alpha)

# ------------------------
# MAIN
# ------------------------
if __name__ == "__main__":
    all_experiments = discover_experiments(base_path)

    if not all_experiments:
        raise RuntimeError(f"No experiment folders found in: {base_path}")

    groups = {
        "stateless": [],
        "stateful": [],
    }

    for exp_dir in all_experiments:
        exp_name = os.path.basename(exp_dir)
        exp_type = get_experiment_type(exp_name)

        if exp_type == "unknown":
            print(f"[WARN] Skipping unknown experiment type: {exp_name}")
            continue

        groups[exp_type].append(exp_dir)

    print(f"[INFO] Found {len(groups['stateless'])} stateless experiment(s)")
    print(f"[INFO] Found {len(groups['stateful'])} stateful experiment(s)")

    gens_s, mean_s, std_s, n_s = aggregate_group(groups["stateless"])
    gens_f, mean_f, std_f, n_f = aggregate_group(groups["stateful"])

    if len(gens_s) == 0 and len(gens_f) == 0:
        raise RuntimeError("No usable effective instruction-count curves found for either group.")

    # ---------------------------------------------
    # FORCE BOTH GROUPS TO END AT SAME GENERATION
    # ---------------------------------------------
    if len(gens_s) > 0 and len(gens_f) > 0:
        L = min(len(gens_s), len(gens_f))
        gens = np.arange(L)

        mean_s = mean_s[:L]
        std_s = std_s[:L]
        mean_f = mean_f[:L]
        std_f = std_f[:L]

    elif len(gens_s) > 0:
        L = len(gens_s)
        gens = gens_s
        mean_s = mean_s[:L]
        std_s = std_s[:L]

    else:
        L = len(gens_f)
        gens = gens_f
        mean_f = mean_f[:L]
        std_f = std_f[:L]

    # ------------------------
    # PLOT
    # ------------------------
    fig, ax = plt.subplots(figsize=(8.2, 5.2))

    if len(gens_s) > 0:
        plot_mean_with_std(
            ax,
            gens,
            mean_s,
            std_s,
            f"Stateless (n={n_s})",
            COLORS["stateless"]
        )

    if len(gens_f) > 0:
        plot_mean_with_std(
            ax,
            gens,
            mean_f,
            std_f,
            f"Stateful (n={n_f})",
            COLORS["stateful"]
        )

    ax.set_xlabel("Generation")
    ax.set_ylabel("Mean Effective Instruction Count")
    ax.set_title("Mean Effective Instruction Count per Generation", pad=10)

    ax.legend(loc="best", frameon=True)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.grid(True, axis="y", alpha=0.25)
    ax.set_xlim(0, L - 1)

    out_dir = os.path.join(base_path, "gazebo_curve_plots")
    os.makedirs(out_dir, exist_ok=True)

    pdf_path = os.path.join(out_dir, "stateful_vs_stateless_mean_effective_instruction_count.pdf")
    png_path = os.path.join(out_dir, "stateful_vs_stateless_mean_effective_instruction_count.png")

    plt.tight_layout()
    plt.savefig(pdf_path, bbox_inches="tight")
    plt.savefig(png_path, dpi=600, bbox_inches="tight")
    plt.show()

    print(f"[INFO] Saved PDF: {pdf_path}")
    print(f"[INFO] Saved PNG: {png_path}")
    print(f"[INFO] Common plotted generation length: {L}")
