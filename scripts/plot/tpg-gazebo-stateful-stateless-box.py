import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pandas.errors import EmptyDataError
from matplotlib.colors import to_rgba

COLORS = {
    "Stateless": "#1f77b4",  # blue
    "Stateful":  "#ff7f0e",  # orange
}

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
    except (EmptyDataError, UnicodeDecodeError):
        return np.array([], dtype=float)

    if df.shape[0] == 0:
        return np.array([], dtype=float)

    # Your files may have leading spaces in column names
    df.columns = df.columns.astype(str).str.strip()

    if "fitness" not in df.columns:
        return np.array([], dtype=float)

    vals = pd.to_numeric(df["fitness"], errors="coerce").dropna().to_numpy(dtype=float)
    return vals

def colored_boxplot(ax, data_list, labels, color_list, width=0.55, alpha=0.40):
    bp = ax.boxplot(
        data_list,
        widths=width,
        patch_artist=True,
        showmeans=True,
        meanprops=dict(marker="o", markerfacecolor="white", markeredgecolor="black", markersize=6),
        flierprops=dict(marker="o", markersize=4, markerfacecolor="white", markeredgewidth=1.2),
        medianprops=dict(linewidth=2.4),
        whiskerprops=dict(linewidth=2.0),
        capprops=dict(linewidth=2.0),
        boxprops=dict(linewidth=2.8),
    )

    for i, c in enumerate(color_list):
        bp["boxes"][i].set(facecolor=to_rgba(c, alpha), edgecolor=c, linewidth=3.0)
        bp["medians"][i].set(color=c, linewidth=2.8)

        for w in bp["whiskers"][2*i:2*i+2]:
            w.set(color=c, linewidth=2.2)
        for cap in bp["caps"][2*i:2*i+2]:
            cap.set(color=c, linewidth=2.2)

        bp["fliers"][i].set(markeredgecolor=c)
        bp["means"][i].set(markeredgecolor=c)

    ax.set_xticklabels(labels)
    return bp

def summarize(name: str, x: np.ndarray):
    mean = float(np.mean(x))
    var  = float(np.var(x, ddof=1))  # sample variance
    std  = float(np.sqrt(var))
    print(f"{name}: N={x.size} mean={mean:.6g} var={var:.6g} std={std:.6g}")

if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    stats_dir = os.path.join(here, "best_agent_stats")
    pattern = os.path.join(stats_dir, "*.csv")
    files = sorted(glob.glob(pattern))

    if not files:
        raise RuntimeError(f"No CSV files found in: {stats_dir}")

    data = {"Stateless": [], "Stateful": []}

    for f in files:
        exp_type = infer_type_from_filename(f)
        if exp_type == "Unknown":
            print("[WARN] Skipping (can't infer stateful/stateless from filename):", os.path.basename(f))
            continue

        vals = load_fitness_from_csv(f)
        if vals.size == 0:
            print("[WARN] No fitness values found in:", os.path.basename(f))
            continue

        data[exp_type].extend(vals.tolist())

    stateless = np.array(data["Stateless"], dtype=float)
    stateful  = np.array(data["Stateful"], dtype=float)

    if stateless.size == 0 or stateful.size == 0:
        raise RuntimeError(f"Missing group data. Stateless={stateless.size}, Stateful={stateful.size}")

    summarize("Stateless", stateless)
    summarize("Stateful", stateful)

    fig, ax = plt.subplots(figsize=(6, 5))
    colored_boxplot(
        ax,
        data_list=[stateless, stateful],
        labels=["Stateless", "Stateful"],
        color_list=[COLORS["Stateless"], COLORS["Stateful"]],
        alpha=0.40,
    )

    ax.set_ylabel("Episode Fitness")
    ax.set_title("Episode Fitness with Indistribution Targets")

    out_dir = os.path.join(here, "gazebo_curve_plots")
    os.makedirs(out_dir, exist_ok=True)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "boxplot_benchmark_episode_fitness.pdf"), bbox_inches="tight")
    plt.savefig(os.path.join(out_dir, "boxplot_benchmark_episode_fitness.png"), dpi=300, bbox_inches="tight")
    plt.show()
