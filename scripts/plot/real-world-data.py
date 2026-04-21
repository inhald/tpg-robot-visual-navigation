import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.colors import to_rgba
from scipy.stats import wilcoxon

# ============================================================
# CONFIG
# ============================================================

METHOD_NAMES = ["Stateless", "Stateful"]
SCENE_NAMES = ["Scene 1", "Scene 2", "Scene 3"]
POSE_NAMES = ["Facing target", "Left of target", "Right of target", "Facing behind"]

COLORS = {
    "Stateless": "#1f77b4",
    "Stateful": "#ff7f0e",
}

SAVE_DIR = "plots"
SAVE_PNG = True
SAVE_PDF = True
MULTIPAGE_PDF_NAME = "real_world_results_summary.pdf"

os.makedirs(SAVE_DIR, exist_ok=True)

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
# DATA FORMAT
# ============================================================
# data.shape = (2, 3, 3, 4, N, 2)
#
# axis 0: method        0=stateless, 1=stateful
# axis 1: scene         0,1,2
# axis 2: configuration 0,1,2
# axis 3: pose          0=facing, 1=left, 2=right, 3=behind
# axis 4: trial index   0..N-1
# axis 5: metric        0=success (0/1), 1=solve time (seconds)
# ============================================================


def validate_data(data: np.ndarray) -> None:
    if data.ndim != 6:
        raise ValueError(f"Expected 6D array, got ndim={data.ndim}")
    expected = (2, 3, 3, 4, data.shape[4], 2)
    if data.shape != expected:
        raise ValueError(f"Expected shape like (2,3,3,4,N,2), got {data.shape}")

    success = data[..., 0]
    valid = np.isnan(success) | (success == 0) | (success == 1)
    if not np.all(valid):
        raise ValueError("Success values must be 0, 1, or np.nan")

    solve_time = data[..., 1]
    if np.any(solve_time[~np.isnan(solve_time)] < 0):
        raise ValueError("Solve times must be nonnegative or np.nan")


def save_figure(fig, stem: str, pdf_pages: PdfPages | None = None):
    if SAVE_PNG:
        fig.savefig(os.path.join(SAVE_DIR, f"{stem}.png"), dpi=300, bbox_inches="tight")
    if SAVE_PDF:
        fig.savefig(os.path.join(SAVE_DIR, f"{stem}.pdf"), bbox_inches="tight")
    if pdf_pages is not None:
        pdf_pages.savefig(fig, bbox_inches="tight")


def style_axes(ax):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.grid(True, axis="y", alpha=0.25)
    ax.set_axisbelow(True)


def mean_ignore_nan(x: np.ndarray) -> float:
    x = x[~np.isnan(x)]
    if x.size == 0:
        return np.nan
    return float(np.mean(x))


def success_rate(x: np.ndarray) -> float:
    return mean_ignore_nan(x)


def sem_binary(x: np.ndarray) -> float:
    x = x[~np.isnan(x)]
    n = x.size
    if n <= 1:
        return np.nan
    p = np.mean(x)
    return float(np.sqrt(p * (1 - p) / n))


def mean_solve_time_success_only(success: np.ndarray, solve_time: np.ndarray) -> float:
    vals = solve_time[(success == 1) & ~np.isnan(solve_time)]
    if vals.size == 0:
        return np.nan
    return float(np.mean(vals))


def sem_solve_time_success_only(success: np.ndarray, solve_time: np.ndarray) -> float:
    vals = solve_time[(success == 1) & ~np.isnan(solve_time)]
    if vals.size <= 1:
        return np.nan
    return float(np.std(vals, ddof=1) / np.sqrt(vals.size))


def aggregate_by_scene(data: np.ndarray):
    scene_success = np.full((2, 3), np.nan)
    scene_success_sem = np.full((2, 3), np.nan)
    scene_time = np.full((2, 3), np.nan)
    scene_time_sem = np.full((2, 3), np.nan)

    for m in range(2):
        for s in range(3):
            success = data[m, s, :, :, :, 0].reshape(-1)
            times = data[m, s, :, :, :, 1].reshape(-1)
            scene_success[m, s] = success_rate(success)
            scene_success_sem[m, s] = sem_binary(success)
            scene_time[m, s] = mean_solve_time_success_only(success, times)
            scene_time_sem[m, s] = sem_solve_time_success_only(success, times)

    return scene_success, scene_success_sem, scene_time, scene_time_sem


def aggregate_by_pose(data: np.ndarray):
    pose_success = np.full((2, 4), np.nan)
    pose_success_sem = np.full((2, 4), np.nan)
    pose_time = np.full((2, 4), np.nan)
    pose_time_sem = np.full((2, 4), np.nan)

    for m in range(2):
        for p in range(4):
            success = data[m, :, :, p, :, 0].reshape(-1)
            times = data[m, :, :, p, :, 1].reshape(-1)
            pose_success[m, p] = success_rate(success)
            pose_success_sem[m, p] = sem_binary(success)
            pose_time[m, p] = mean_solve_time_success_only(success, times)
            pose_time_sem[m, p] = sem_solve_time_success_only(success, times)

    return pose_success, pose_success_sem, pose_time, pose_time_sem


def aggregate_scene_pose(data: np.ndarray):
    success_mat = np.full((2, 3, 4), np.nan)
    time_mat = np.full((2, 3, 4), np.nan)

    for m in range(2):
        for s in range(3):
            for p in range(4):
                success = data[m, s, :, p, :, 0].reshape(-1)
                times = data[m, s, :, p, :, 1].reshape(-1)
                success_mat[m, s, p] = success_rate(success)
                time_mat[m, s, p] = mean_solve_time_success_only(success, times)

    return success_mat, time_mat


def print_summary_tables(data: np.ndarray):
    scene_success, _, scene_time, _ = aggregate_by_scene(data)
    pose_success, _, pose_time, _ = aggregate_by_pose(data)

    print("\n=== Success rate by scene ===")
    for m in range(2):
        print(f"\n{METHOD_NAMES[m]}")
        for s in range(3):
            v = scene_success[m, s]
            print(f"  {SCENE_NAMES[s]}: {'nan' if np.isnan(v) else f'{v:.3f}'}")

    print("\n=== Mean solve time by scene (successful trials only) ===")
    for m in range(2):
        print(f"\n{METHOD_NAMES[m]}")
        for s in range(3):
            v = scene_time[m, s]
            print(f"  {SCENE_NAMES[s]}: {'nan' if np.isnan(v) else f'{v:.3f}'}")

    print("\n=== Success rate by pose ===")
    for m in range(2):
        print(f"\n{METHOD_NAMES[m]}")
        for p in range(4):
            v = pose_success[m, p]
            print(f"  {POSE_NAMES[p]}: {'nan' if np.isnan(v) else f'{v:.3f}'}")

    print("\n=== Mean solve time by pose (successful trials only) ===")
    for m in range(2):
        print(f"\n{METHOD_NAMES[m]}")
        for p in range(4):
            v = pose_time[m, p]
            print(f"  {POSE_NAMES[p]}: {'nan' if np.isnan(v) else f'{v:.3f}'}")


def add_bar_labels(ax, bars, values, errors=None, fmt="{:.2f}"):
    ymin, ymax = ax.get_ylim()
    offset = 0.018 * (ymax - ymin)

    for i, (bar, val) in enumerate(zip(bars, values)):
        if np.isnan(val):
            continue

        err = 0.0
        if errors is not None and i < len(errors) and not np.isnan(errors[i]):
            err = errors[i]

        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + err + offset,
            fmt.format(val),
            ha="center",
            va="bottom",
            fontsize=10
        )


def plot_grouped_bar(values, errors, category_names, ylabel, title, stem,
                     pdf_pages=None, ylim=None, value_fmt="{:.2f}"):
    x = np.arange(len(category_names))
    width = 0.36

    fig, ax = plt.subplots(figsize=(8.2, 5.2))
    style_axes(ax)

    vals0 = np.nan_to_num(values[0], nan=0.0)
    vals1 = np.nan_to_num(values[1], nan=0.0)
    err0 = np.nan_to_num(errors[0], nan=0.0)
    err1 = np.nan_to_num(errors[1], nan=0.0)

    bars0 = ax.bar(
        x - width / 2, vals0, width,
        yerr=err0, capsize=4,
        color=COLORS["Stateless"], edgecolor="black", linewidth=1.0,
        label="Stateless"
    )
    bars1 = ax.bar(
        x + width / 2, vals1, width,
        yerr=err1, capsize=4,
        color=COLORS["Stateful"], edgecolor="black", linewidth=1.0,
        label="Stateful"
    )

    ax.set_xticks(x)
    ax.set_xticklabels(category_names, rotation=0, ha="center")
    ax.set_ylabel(ylabel)
    ax.set_title(title, pad=10)

    if ylim is not None:
        ax.set_ylim(ylim)
    else:
        ax.margins(y=0.14)

    add_bar_labels(ax, bars0, values[0], errors[0], fmt=value_fmt)
    add_bar_labels(ax, bars1, values[1], errors[1], fmt=value_fmt)

    ax.legend(
        loc="upper center",
        bbox_to_anchor=(0.5, -0.20),
        ncol=2,
        frameon=False
    )

    plt.tight_layout(rect=[0, 0.10, 1, 1])
    save_figure(fig, stem, pdf_pages=pdf_pages)
    plt.close(fig)


def plot_heatmap_pair(mat, title, stem, cmap="viridis", pdf_pages=None, vmin=None, vmax=None):
    if np.all(np.isnan(mat)):
        return

    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.6), constrained_layout=True)

    im = None
    for m in range(2):
        panel = np.ma.masked_invalid(mat[m])
        im = axes[m].imshow(panel, aspect="auto", cmap=cmap, vmin=vmin, vmax=vmax)
        axes[m].set_title(METHOD_NAMES[m], pad=8)
        axes[m].set_xticks(np.arange(len(POSE_NAMES)))
        axes[m].set_xticklabels(POSE_NAMES, rotation=20, ha="right")
        axes[m].set_yticks(np.arange(len(SCENE_NAMES)))
        axes[m].set_yticklabels(SCENE_NAMES)

        axes[m].set_xticks(np.arange(-0.5, len(POSE_NAMES), 1), minor=True)
        axes[m].set_yticks(np.arange(-0.5, len(SCENE_NAMES), 1), minor=True)
        axes[m].grid(which="minor", color="white", linestyle="-", linewidth=1.0)
        axes[m].tick_params(which="minor", bottom=False, left=False)

        for i in range(mat[m].shape[0]):
            for j in range(mat[m].shape[1]):
                val = mat[m, i, j]
                txt = "NA" if np.isnan(val) else f"{val:.2f}"
                axes[m].text(
                    j, i, txt,
                    ha="center", va="center", fontsize=9,
                    color="white" if not np.isnan(val) else "black"
                )

    cbar = fig.colorbar(im, ax=axes.ravel().tolist(), shrink=0.94)
    cbar.ax.set_ylabel(title)
    fig.suptitle(title, y=1.02)

    save_figure(fig, stem, pdf_pages=pdf_pages)
    plt.close(fig)


def colored_boxplot(ax, data_list, labels, color_list, width=0.55, alpha=0.40):
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
        whiskerprops=dict(linewidth=2.0),
        capprops=dict(linewidth=2.0),
        boxprops=dict(linewidth=2.8),
    )

    for i, c in enumerate(color_list):
        bp["boxes"][i].set(facecolor=to_rgba(c, alpha), edgecolor=c, linewidth=3.0)
        bp["medians"][i].set(color=c, linewidth=2.8)

        for w in bp["whiskers"][2 * i:2 * i + 2]:
            w.set(color=c, linewidth=2.2)
        for cap in bp["caps"][2 * i:2 * i + 2]:
            cap.set(color=c, linewidth=2.2)

        bp["fliers"][i].set(markeredgecolor=c)
        bp["means"][i].set(markeredgecolor=c)

    ax.set_xticklabels(labels)
    return bp


def plot_solve_time_boxplot(data: np.ndarray, pdf_pages=None):
    stateless_success = data[0, :, :, :, :, 0].reshape(-1)
    stateless_times = data[0, :, :, :, :, 1].reshape(-1)
    stateless = stateless_times[(stateless_success == 1) & ~np.isnan(stateless_times)]

    stateful_success = data[1, :, :, :, :, 0].reshape(-1)
    stateful_times = data[1, :, :, :, :, 1].reshape(-1)
    stateful = stateful_times[(stateful_success == 1) & ~np.isnan(stateful_times)]

    if stateless.size == 0 or stateful.size == 0:
        print("[WARN] Missing data for solve-time boxplot.")
        return

    fig, ax = plt.subplots(figsize=(8.2, 5.2))
    colored_boxplot(
        ax,
        data_list=[stateless, stateful],
        labels=["Stateless", "Stateful"],
        color_list=[COLORS["Stateless"], COLORS["Stateful"]],
        alpha=0.40,
    )

    ax.set_ylabel("Solve Time (s)")
    ax.set_title("Solve Time on Successful Trials", pad=10)
    style_axes(ax)

    plt.tight_layout()
    save_figure(fig, "solve_time_boxplot", pdf_pages=pdf_pages)
    plt.close(fig)


def compute_wilcoxon_tests(data: np.ndarray):
    success_stateful = []
    success_stateless = []
    time_stateful = []
    time_stateless = []

    for s in range(3):
        for c in range(3):
            for p in range(4):
                s0 = data[0, s, c, p, :, 0]
                s1 = data[1, s, c, p, :, 0]
                t0 = data[0, s, c, p, :, 1]
                t1 = data[1, s, c, p, :, 1]

                for i in range(len(s0)):
                    if np.isnan(s0[i]) or np.isnan(s1[i]):
                        continue

                    success_stateless.append(s0[i])
                    success_stateful.append(s1[i])

                    if s0[i] == 1 and s1[i] == 1 and not np.isnan(t0[i]) and not np.isnan(t1[i]):
                        time_stateless.append(t0[i])
                        time_stateful.append(t1[i])

    success_stateless = np.array(success_stateless)
    success_stateful = np.array(success_stateful)
    time_stateless = np.array(time_stateless)
    time_stateful = np.array(time_stateful)

    print("\n================ WILCOXON TESTS ================\n")

    print("SUCCESS COMPARISON")
    if len(success_stateless) > 0 and not np.all(success_stateless == success_stateful):
        W, p = wilcoxon(success_stateful, success_stateless)
        print(f"n = {len(success_stateless)}")
        print(f"W = {W:.3f}")
        print(f"p-value = {p:.6f}")
        print(f"Mean success, Stateless = {np.mean(success_stateless):.3f}")
        print(f"Mean success, Stateful  = {np.mean(success_stateful):.3f}")
    else:
        print("Not enough variation for Wilcoxon test.")

    print("\n--------------------------------\n")

    print("SOLVE TIME COMPARISON (PAIRED SUCCESSFUL TRIALS ONLY)")
    if len(time_stateless) > 0 and not np.allclose(time_stateless, time_stateful):
        W, p = wilcoxon(time_stateful, time_stateless)
        print(f"n = {len(time_stateless)}")
        print(f"W = {W:.3f}")
        print(f"p-value = {p:.6f}")
        print(f"Mean solve time, Stateless = {np.mean(time_stateless):.3f}")
        print(f"Mean solve time, Stateful  = {np.mean(time_stateful):.3f}")
    else:
        print("Not enough paired successful trials for Wilcoxon test.")


def make_all_plots(data: np.ndarray):
    validate_data(data)
    print_summary_tables(data)

    scene_success, scene_success_sem, scene_time, scene_time_sem = aggregate_by_scene(data)
    pose_success, pose_success_sem, pose_time, pose_time_sem = aggregate_by_pose(data)
    success_mat, time_mat = aggregate_scene_pose(data)

    multipage_path = os.path.join(SAVE_DIR, MULTIPAGE_PDF_NAME)

    with PdfPages(multipage_path) as pdf:
        plot_grouped_bar(
            scene_success, scene_success_sem, SCENE_NAMES,
            ylabel="Success rate",
            title="Success Rate by Scene",
            stem="success_by_scene",
            pdf_pages=pdf,
            ylim=(0, 1.08),
            value_fmt="{:.2f}",
        )

        plot_grouped_bar(
            pose_success, pose_success_sem, POSE_NAMES,
            ylabel="Success rate",
            title="Success Rate by Pose",
            stem="success_by_pose",
            pdf_pages=pdf,
            ylim=(0, 1.08),
            value_fmt="{:.2f}",
        )

        plot_grouped_bar(
            scene_time, scene_time_sem, SCENE_NAMES,
            ylabel="Mean solve time (s)",
            title="Solve Time by Scene (Successful Trials Only)",
            stem="solve_time_by_scene",
            pdf_pages=pdf,
            value_fmt="{:.2f}",
        )

        plot_grouped_bar(
            pose_time, pose_time_sem, POSE_NAMES,
            ylabel="Mean solve time (s)",
            title="Solve Time by Pose (Successful Trials Only)",
            stem="solve_time_by_pose",
            pdf_pages=pdf,
            value_fmt="{:.2f}",
        )

        plot_heatmap_pair(
            success_mat,
            title="Success Rate",
            stem="success_heatmap",
            cmap="viridis",
            pdf_pages=pdf,
            vmin=0,
            vmax=1,
        )

        plot_heatmap_pair(
            time_mat,
            title="Mean Solve Time (s)",
            stem="solve_time_heatmap",
            cmap="magma",
            pdf_pages=pdf,
        )

        plot_solve_time_boxplot(data, pdf_pages=pdf)

    print(f"\nSaved multipage PDF: {multipage_path}")


# ============================================================
# DATA ENTRY
# ============================================================

N = 1
data = np.full((2, 3, 3, 4, N, 2), np.nan)

# Stateful, Scene 1, Config 1
data[1, 0, 0, 0, 0] = [1, 56.98]
data[1, 0, 0, 1, 0] = [1, 71.0]
data[1, 0, 0, 2, 0] = [1, 44.15]
data[1, 0, 0, 3, 0] = [1, 71.0]

# Stateless, Scene 1, Config 1
data[0, 0, 0, 0, 0] = [0, np.nan]
data[0, 0, 0, 1, 0] = [0, np.nan]
data[0, 0, 0, 2, 0] = [0, np.nan]
data[0, 0, 0, 3, 0] = [0, np.nan]

# Stateless, Scene 1, Config 2
data[0, 0, 1, 0, 0] = [1, 129.0]
data[0, 0, 1, 1, 0] = [1, 162.0]
data[0, 0, 1, 2, 0] = [1, 95.0]
data[0, 0, 1, 3, 0] = [1, 97.0]

# Stateless, Scene 1, Config 3
data[0, 0, 2, 0, 0] = [1, 172.0]
data[0, 0, 2, 1, 0] = [0, np.nan]
data[0, 0, 2, 2, 0] = [1, 132.0]
data[0, 0, 2, 3, 0] = [1, 160.0]

# Stateful, Scene 1, Config 2
data[1, 0, 1, 0, 0] = [1, 49.96]
data[1, 0, 1, 1, 0] = [1, 87.87]
data[1, 0, 1, 2, 0] = [1, 40.11]
data[1, 0, 1, 3, 0] = [1, 62.16]

# Stateful, Scene 1, Config 3
data[1, 0, 2, 0, 0] = [1, 65.97]
data[1, 0, 2, 1, 0] = [1, 34.52]
data[1, 0, 2, 2, 0] = [1, 39.48]
data[1, 0, 2, 3, 0] = [1, 59.57]

# Stateless, Scene 2, Config 1
data[0, 1, 0, 0, 0] = [0, np.nan]
data[0, 1, 0, 1, 0] = [0, np.nan]
data[0, 1, 0, 2, 0] = [1, 155]
data[0, 1, 0, 3, 0] = [1, 144]

# Stateless, Scene 2, Config 2
data[0, 1, 1, 0, 0] = [0, np.nan]
data[0, 1, 1, 1, 0] = [1, 113]
data[0, 1, 1, 2, 0] = [1, 163]
data[0, 1, 1, 3, 0] = [1, 132]

# Stateless, Scene 2, Config 3
data[0, 1, 2, 0, 0] = [1, 125]
data[0, 1, 2, 1, 0] = [0, np.nan]
data[0, 1, 2, 2, 0] = [1, 132]
data[0, 1, 2, 3, 0] = [0, np.nan]

# Stateful, Scene 2, Config 1
data[1, 1, 0, 0, 0] = [1, 37.05]
data[1, 1, 0, 1, 0] = [1, 39.51]
data[1, 1, 0, 2, 0] = [1, 68.70]
data[1, 1, 0, 3, 0] = [1, 63.54]

# Stateful, Scene 2, Config 2
data[1, 1, 1, 0, 0] = [1, 57.83]
data[1, 1, 1, 1, 0] = [1, 51.94]
data[1, 1, 1, 2, 0] = [1, 70.92]
data[1, 1, 1, 3, 0] = [1, 68.72]

# Stateful, Scene 2, Config 3
data[1, 1, 2, 0, 0] = [1, 58.81]
data[1, 1, 2, 1, 0] = [1, 24.92]
data[1, 1, 2, 2, 0] = [1, 53.24]
data[1, 1, 2, 3, 0] = [1, 61.0]

# Stateful, Scene 3, Config 1
data[1, 2, 0, 0, 0] = [1, 49.83]
data[1, 2, 0, 1, 0] = [1, 66.80]
data[1, 2, 0, 2, 0] = [1, 81.95]
data[1, 2, 0, 3, 0] = [1, 56.27]

# Stateful, Scene 3, Config 2
data[1, 2, 1, 0, 0] = [1, 49.17]
data[1, 2, 1, 1, 0] = [1, 53.19]
data[1, 2, 1, 2, 0] = [1, 51.79]
data[1, 2, 1, 3, 0] = [1, 79.51]

# Stateful, Scene 3, Config 3
data[1, 2, 2, 0, 0] = [1, 74.13]
data[1, 2, 2, 1, 0] = [1, 57.06]
data[1, 2, 2, 2, 0] = [1, 58.06]
data[1, 2, 2, 3, 0] = [1, 77.59]

# Stateless, Scene 3, Config 1
data[0, 2, 0, 0, 0] = [1, 155]
data[0, 2, 0, 1, 0] = [0, np.nan]
data[0, 2, 0, 2, 0] = [1, 184]
data[0, 2, 0, 3, 0] = [0, np.nan]

# Stateless, Scene 3, Config 2
data[0, 2, 1, 0, 0] = [1, 154]
data[0, 2, 1, 1, 0] = [0, np.nan]
data[0, 2, 1, 2, 0] = [0, np.nan]
data[0, 2, 1, 3, 0] = [0, np.nan]

# Stateless, Scene 3, Config 3
data[0, 2, 2, 0, 0] = [1, 155]
data[0, 2, 2, 1, 0] = [1, 149]
data[0, 2, 2, 2, 0] = [1, 143]
data[0, 2, 2, 3, 0] = [1, 144]


if __name__ == "__main__":
    make_all_plots(data)
    compute_wilcoxon_tests(data)
