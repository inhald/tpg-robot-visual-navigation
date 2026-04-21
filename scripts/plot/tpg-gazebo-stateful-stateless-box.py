import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pandas.errors import EmptyDataError
from matplotlib.colors import to_rgba
from scipy.stats import wilcoxon

# ============================================================
# CONFIG
# ============================================================

COLORS = {
    "Stateless": "#1f77b4",
    "Stateful": "#ff7f0e",
}

SAVE_DIR = "gazebo_curve_plots"
SAVE_PNG = True
SAVE_PDF = True

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
    "figure.facecolor": "white",
    "axes.facecolor": "white",
})

# ============================================================
# HELPERS
# ============================================================

def infer_type_from_filename(path: str) -> str:
    name = os.path.basename(path).lower()
    if "stateless" in name:
        return "Stateless"
    if "stateful" in name:
        return "Stateful"
    return "Unknown"


def load_fitness_from_csv(path: str) -> np.ndarray:
    try:
        df = pd.read_csv(path)
    except (EmptyDataError, UnicodeDecodeError, pd.errors.ParserError):
        return np.array([], dtype=float)

    if df.empty:
        return np.array([], dtype=float)

    df.columns = df.columns.astype(str).str.strip()

    if "fitness" not in df.columns:
        return np.array([], dtype=float)

    vals = pd.to_numeric(df["fitness"], errors="coerce").dropna().to_numpy(dtype=float)
    return vals


def summarize(name: str, x: np.ndarray) -> None:
    mean = float(np.mean(x))
    var = float(np.var(x, ddof=1)) if x.size > 1 else 0.0
    std = float(np.sqrt(var))
    print(f"{name}: N={x.size} mean={mean:.6g} var={var:.6g} std={std:.6g}")


def style_axes(ax):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.grid(True, axis="y", alpha=0.25)
    ax.set_axisbelow(True)


def colored_boxplot(ax, data_list, labels, color_list, width=0.55, alpha=0.40):
    """
    Matches the boxplot styling used in the real-world script:
    - colored outlines
    - colored whiskers/caps/medians
    - white mean marker
    - semi-transparent box fill
    """
    bp = ax.boxplot(
        data_list,
        widths=width,
        patch_artist=True,
        showmeans=True,
        meanprops=dict(
            marker="o",
            markerfacecolor="white",
            markeredgecolor="black",
            markersize=6
        ),
        flierprops=dict(
            marker="o",
            markersize=4,
            markerfacecolor="white",
            markeredgewidth=1.2
        ),
        medianprops=dict(linewidth=2.4),
        whiskerprops=dict(linewidth=2.4),
        capprops=dict(linewidth=2.4),
        boxprops=dict(linewidth=2.4),
    )

    for i, c in enumerate(color_list):
        bp["boxes"][i].set(
            facecolor=to_rgba(c, alpha),
            edgecolor=c,
            linewidth=3.0
        )
        bp["medians"][i].set(color=c, linewidth=2.8)

        for w in bp["whiskers"][2 * i:2 * i + 2]:
            w.set(color=c, linewidth=2.2)

        for cap in bp["caps"][2 * i:2 * i + 2]:
            cap.set(color=c, linewidth=2.2)

        bp["fliers"][i].set(markeredgecolor=c)
        bp["means"][i].set(markeredgecolor=c)

    ax.set_xticklabels(labels)
    return bp


def run_wilcoxon(stateless: np.ndarray, stateful: np.ndarray) -> None:
    """
    Runs Wilcoxon signed-rank test by pairing pooled observations by index,
    matching the logic from the original episode-fitness script.
    """
    L = min(stateless.size, stateful.size)

    if L == 0:
        print("[WARN] Cannot run Wilcoxon test: one or both groups are empty.")
        return

    if stateless.size != stateful.size:
        print(
            f"[WARN] Unequal sample sizes: Stateless={stateless.size}, Stateful={stateful.size}. "
            f"Using first {L} paired observations from each group."
        )

    stateless_paired = stateless[:L]
    stateful_paired = stateful[:L]

    diffs = stateful_paired - stateless_paired
    nonzero_mask = ~np.isclose(diffs, 0.0)

    if np.count_nonzero(nonzero_mask) == 0:
        print("[WARN] All paired differences are zero; Wilcoxon signed-rank test is undefined.")
        return

    result = wilcoxon(
        stateful_paired[nonzero_mask],
        stateless_paired[nonzero_mask],
        alternative="two-sided",
        zero_method="wilcox"
    )

    print("\n[INFO] Wilcoxon signed-rank test on paired pooled observations")
    print(f"[INFO] Number of paired observations used: {np.count_nonzero(nonzero_mask)}")
    print(f"[INFO] Wilcoxon statistic = {result.statistic:.6g}")
    print(f"[INFO] p-value = {result.pvalue:.6g}")


def save_figure(fig, out_dir: str, stem: str) -> None:
    os.makedirs(out_dir, exist_ok=True)

    if SAVE_PNG:
        fig.savefig(os.path.join(out_dir, f"{stem}.png"), dpi=300, bbox_inches="tight")
    if SAVE_PDF:
        fig.savefig(os.path.join(out_dir, f"{stem}.pdf"), bbox_inches="tight")


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    stats_dir = os.path.join(here, "best_agent_stats")
    pattern = os.path.join(stats_dir, "*.csv")
    files = sorted(glob.glob(pattern))

    if not files:
        raise RuntimeError(f"No CSV files found in: {stats_dir}")

    data = {
        "Stateless": [],
        "Stateful": []
    }

    for f in files:
        exp_type = infer_type_from_filename(f)

        if exp_type == "Unknown":
            print(f"[WARN] Skipping file with unknown type: {os.path.basename(f)}")
            continue

        vals = load_fitness_from_csv(f)

        if vals.size == 0:
            print(f"[WARN] No valid fitness values found in: {os.path.basename(f)}")
            continue

        data[exp_type].extend(vals.tolist())

    stateless = np.array(data["Stateless"], dtype=float)
    stateful = np.array(data["Stateful"], dtype=float)

    if stateless.size == 0 or stateful.size == 0:
        raise RuntimeError(
            f"Missing group data. Stateless={stateless.size}, Stateful={stateful.size}"
        )

    summarize("Stateless", stateless)
    summarize("Stateful", stateful)

    run_wilcoxon(stateless, stateful)

    fig, ax = plt.subplots(figsize=(8.2, 5.2))

    colored_boxplot(
        ax,
        data_list=[stateless, stateful],
        labels=["Stateless", "Stateful"],
        color_list=[COLORS["Stateless"], COLORS["Stateful"]],
        alpha=0.40,
    )

    ax.set_ylabel("Episode Fitness")
    ax.set_title("Episode Fitness with Indistribution Targets", pad=10)
    style_axes(ax)

    plt.tight_layout()

    out_dir = os.path.join(here, SAVE_DIR)
    save_figure(fig, out_dir, "boxplot_benchmark_episode_fitness")

    plt.show()
