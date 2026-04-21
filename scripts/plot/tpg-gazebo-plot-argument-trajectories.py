#!/usr/bin/env python3
import os
import math
import glob
from typing import Dict, List, Optional, Tuple

import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.lines import Line2D
from pandas.errors import EmptyDataError

# =========================
# CONFIG
# =========================
HERE = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = HERE
OUT_DIR = os.path.join(BASE_DIR, "argument_trajectory_plots")

EXP_PATTERN = "gazebo_turtlebot4_*_f_*"

ROBOT_RADIUS = 0.11
TARGET_BONUS_RADIUS = 0.30
TARGET_WIDTH = 0.20544
TARGET_DEPTH = 0.055639
PAD = 0.10

TRAJ_COLOR = "#1f77b4"
END_COLOR = "#d62728"
TARGET_FACE = "gold"
TARGET_EDGE = "#8a6d00"
BONUS_FACE = "gold"
BONUS_EDGE = "goldenrod"
OBS_FACE = "#e6e6e6"
OBS_EDGE = "black"
OBS_HATCH = "///"

FIGSIZE = (14.5, 6.2)
PNG_DPI = 300

SUBPLOT_TITLE_FONTSIZE = 10
ROW_LABEL_FONTSIZE = 12
LEGEND_FONTSIZE = 11

# =========================
# HELPERS
# =========================
def infer_type(name: str) -> str:
    lname = name.lower()
    if "stateless" in lname:
        return "stateless"
    if "stateful" in lname:
        return "stateful"
    return "unknown"


def discover_experiments(base_dir: str) -> List[str]:
    pattern = os.path.join(base_dir, EXP_PATTERN)
    return [p for p in sorted(glob.glob(pattern)) if os.path.isdir(p)]


def load_csv(path: str) -> Optional[pd.DataFrame]:
    if not os.path.isfile(path):
        return None
    try:
        df = pd.read_csv(path)
    except (EmptyDataError, pd.errors.ParserError, UnicodeDecodeError, FileNotFoundError):
        return None
    if df is None or df.empty:
        return None
    df.columns = df.columns.astype(str).str.strip()
    return df


def heading_line(ax, x: float, y: float, yaw: float, radius: float,
                 color: str = "k", lw: float = 1.2, zorder: int = 6):
    dx = radius * math.cos(yaw)
    dy = radius * math.sin(yaw)
    ax.plot([x, x + dx], [y, y + dy], color=color, linewidth=lw, zorder=zorder)


def compute_global_limits(items: List[Optional[Dict]]) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    xmin = float("inf")
    xmax = float("-inf")
    ymin = float("inf")
    ymax = float("-inf")

    found_any = False

    for item in items:
        if item is None:
            continue

        found_any = True
        t = item["traj"]
        w = item["world_row"]

        obstacle_xmin = w["obstacle_x"] - w["obstacle_width"] / 2.0
        obstacle_xmax = w["obstacle_x"] + w["obstacle_width"] / 2.0
        obstacle_ymin = w["obstacle_y"] - w["obstacle_depth"] / 2.0
        obstacle_ymax = w["obstacle_y"] + w["obstacle_depth"] / 2.0

        xmin = min(xmin, t["robot_x"].min(), w["goal_x"] - TARGET_BONUS_RADIUS, obstacle_xmin)
        xmax = max(xmax, t["robot_x"].max(), w["goal_x"] + TARGET_BONUS_RADIUS, obstacle_xmax)
        ymin = min(ymin, t["robot_y"].min(), w["goal_y"] - TARGET_BONUS_RADIUS, obstacle_ymin)
        ymax = max(ymax, t["robot_y"].max(), w["goal_y"] + TARGET_BONUS_RADIUS, obstacle_ymax)

    if not found_any:
        raise RuntimeError("No valid items found when computing global axis limits.")

    return (xmin - PAD, xmax + PAD), (ymin - PAD, ymax + PAD)


def pad_to_five(items: List[Dict]) -> List[Optional[Dict]]:
    padded: List[Optional[Dict]] = items[:5]
    while len(padded) < 5:
        padded.append(None)
    return padded


# =========================
# DATA COLLECTION
# =========================
def collect_cases() -> Dict[str, List[Dict]]:
    grouped = {"stateless": [], "stateful": []}

    for exp_dir in discover_experiments(BASE_DIR):
        exp_name = os.path.basename(exp_dir)
        kind = infer_type(exp_name)

        if kind == "unknown":
            continue

        best_dir = os.path.join(exp_dir, "logs", "best_agent")
        stats = load_csv(os.path.join(best_dir, "best_agent_stats.csv"))
        traj = load_csv(os.path.join(best_dir, "trajectory.csv"))
        world = load_csv(os.path.join(best_dir, "world_layout.csv"))

        if stats is None or traj is None or world is None:
            print(f"[WARN] Skipping {exp_name}: missing best_agent CSVs")
            continue

        required_stats = {"episode", "fitness"}
        required_traj = {"episode", "robot_x", "robot_y", "robot_yaw"}
        required_world = {
            "episode",
            "goal_x",
            "goal_y",
            "obstacle_x",
            "obstacle_y",
            "obstacle_width",
            "obstacle_depth",
        }

        if not required_stats.issubset(stats.columns):
            print(f"[WARN] Skipping {exp_name}: bad stats columns")
            continue
        if not required_traj.issubset(traj.columns):
            print(f"[WARN] Skipping {exp_name}: bad trajectory columns")
            continue
        if not required_world.issubset(world.columns):
            print(f"[WARN] Skipping {exp_name}: bad world columns")
            continue

        stats["episode"] = pd.to_numeric(stats["episode"], errors="coerce")
        stats["fitness"] = pd.to_numeric(stats["fitness"], errors="coerce")
        stats = stats.dropna(subset=["episode", "fitness"])

        if stats.empty:
            print(f"[WARN] Skipping {exp_name}: no usable fitness rows")
            continue

        grouped[kind].append({
            "name": exp_name,
            "stats": stats.copy(),
            "traj": traj.copy(),
            "world": world.copy(),
        })

        print(f"[INFO] Loaded {kind}: {exp_name}")

    return grouped


def select_one_episode_per_experiment(group_cases: List[Dict], which: str) -> List[Dict]:
    if which not in {"best", "worst"}:
        raise ValueError("which must be 'best' or 'worst'")

    selected: List[Dict] = []

    for case in group_cases:
        stats = case["stats"]

        if which == "best":
            row = stats.loc[stats["fitness"].idxmax()]
        else:
            row = stats.loc[stats["fitness"].idxmin()]

        ep = int(row["episode"])
        fit = float(row["fitness"])

        traj_ep = case["traj"][pd.to_numeric(case["traj"]["episode"], errors="coerce") == ep].copy()
        world_ep = case["world"][pd.to_numeric(case["world"]["episode"], errors="coerce") == ep].copy()

        if traj_ep.empty or world_ep.empty:
            print(f"[WARN] Skipping {case['name']} episode {ep}: missing episode data")
            continue

        selected.append({
            "name": case["name"],
            "episode": ep,
            "fitness": fit,
            "traj": traj_ep,
            "world_row": world_ep.iloc[0],
        })

    selected = sorted(selected, key=lambda d: d["fitness"], reverse=(which == "best"))
    return selected


# =========================
# PLOTTING
# =========================
def plot_one(ax, item: Dict, global_xlim: Tuple[float, float], global_ylim: Tuple[float, float]):
    t = item["traj"]
    w = item["world_row"]

    obstacle = Rectangle(
        (
            w["obstacle_x"] - w["obstacle_width"] / 2.0,
            w["obstacle_y"] - w["obstacle_depth"] / 2.0,
        ),
        w["obstacle_width"],
        w["obstacle_depth"],
        facecolor=OBS_FACE,
        edgecolor=OBS_EDGE,
        hatch=OBS_HATCH,
        linewidth=1.2,
        zorder=1,
    )
    ax.add_patch(obstacle)

    bonus = Circle(
        (w["goal_x"], w["goal_y"]),
        TARGET_BONUS_RADIUS,
        facecolor=BONUS_FACE,
        edgecolor=BONUS_EDGE,
        linestyle="--",
        linewidth=1.1,
        alpha=0.20,
        zorder=2,
    )
    ax.add_patch(bonus)

    target = Rectangle(
        (
            w["goal_x"] - TARGET_WIDTH / 2.0,
            w["goal_y"] - TARGET_DEPTH / 2.0,
        ),
        TARGET_WIDTH,
        TARGET_DEPTH,
        facecolor=TARGET_FACE,
        edgecolor=TARGET_EDGE,
        linewidth=1.1,
        zorder=7,
    )
    ax.add_patch(target)

    ax.plot(
        t["robot_x"],
        t["robot_y"],
        color=TRAJ_COLOR,
        linewidth=2.2,
        zorder=4,
    )

    sx = float(t.iloc[0]["robot_x"])
    sy = float(t.iloc[0]["robot_y"])
    syaw = float(t.iloc[0]["robot_yaw"])

    ex = float(t.iloc[-1]["robot_x"])
    ey = float(t.iloc[-1]["robot_y"])
    eyaw = float(t.iloc[-1]["robot_yaw"])

    start_fp = Circle(
        (sx, sy),
        ROBOT_RADIUS,
        fill=False,
        edgecolor=TRAJ_COLOR,
        linewidth=1.8,
        zorder=5,
    )
    end_fp = Circle(
        (ex, ey),
        ROBOT_RADIUS,
        fill=False,
        edgecolor=END_COLOR,
        linestyle="--",
        linewidth=1.8,
        zorder=5,
    )
    ax.add_patch(start_fp)
    ax.add_patch(end_fp)

    ax.scatter([sx], [sy], color=TRAJ_COLOR, s=22, zorder=6)
    ax.scatter([ex], [ey], color=END_COLOR, s=22, zorder=6)

    heading_line(ax, sx, sy, syaw, ROBOT_RADIUS, color=TRAJ_COLOR, lw=1.2, zorder=6)
    heading_line(ax, ex, ey, eyaw, ROBOT_RADIUS, color=END_COLOR, lw=1.2, zorder=6)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(*global_xlim)
    ax.set_ylim(*global_ylim)
    ax.set_xticks([])
    ax.set_yticks([])

    for spine in ax.spines.values():
        spine.set_linewidth(0.8)


def build_shared_legend(fig):
    handles = [
        Line2D([0], [0], color=TRAJ_COLOR, linewidth=2.2, label="trajectory"),
        Circle((0, 0), 0.1, fill=False, edgecolor=TRAJ_COLOR, linewidth=1.8, label="start"),
        Circle((0, 0), 0.1, fill=False, edgecolor=END_COLOR, linestyle="--", linewidth=1.8, label="end"),
        Rectangle((0, 0), 1.0, 0.35, facecolor=TARGET_FACE, edgecolor=TARGET_EDGE, linewidth=1.0, label="target"),
        Circle((0, 0), 0.1, facecolor=BONUS_FACE, edgecolor=BONUS_EDGE, linestyle="--", linewidth=1.0, alpha=0.20, label="bonus region"),
        Rectangle((0, 0), 1.0, 1.0, facecolor=OBS_FACE, edgecolor=OBS_EDGE, hatch=OBS_HATCH, linewidth=1.0, label="obstacle"),
    ]
    labels = [
        "trajectory",
        "start",
        "end",
        "target",
        "bonus region",
        "obstacle",
    ]
    fig.legend(
        handles,
        labels,
        loc="lower center",
        ncol=3,
        frameon=True,
        fontsize=LEGEND_FONTSIZE,
        bbox_to_anchor=(0.5, 0.04),
        columnspacing=1.2,
        handletextpad=0.5,
    )


def make_grid_plot(
    stateless_items: List[Dict],
    stateful_items: List[Dict],
    out_stem: str,
):
    stateless_items_padded = pad_to_five(stateless_items)
    stateful_items_padded = pad_to_five(stateful_items)

    all_items = stateless_items_padded + stateful_items_padded
    global_xlim, global_ylim = compute_global_limits(all_items)

    fig, axes = plt.subplots(2, 5, figsize=FIGSIZE)

    row_defs = [
        ("Stateless", stateless_items_padded),
        ("Stateful", stateful_items_padded),
    ]

    for r, (row_name, items) in enumerate(row_defs):
        for c in range(5):
            ax = axes[r, c]
            item = items[c]

            if item is not None:
                plot_one(ax, item, global_xlim, global_ylim)
                ax.set_title(f"fit = {item['fitness']:.1f}", fontsize=SUBPLOT_TITLE_FONTSIZE, pad=4)
            else:
                ax.axis("off")

            if c == 0:
                ax.set_ylabel(row_name, fontsize=ROW_LABEL_FONTSIZE)

    build_shared_legend(fig)

    fig.subplots_adjust(
        left=0.04,
        right=0.99,
        top=0.92,
        bottom=0.18,
        wspace=0.10,
        hspace=0.15,
    )

    png_path = os.path.join(OUT_DIR, f"{out_stem}.png")
    pdf_path = os.path.join(OUT_DIR, f"{out_stem}.pdf")

    fig.savefig(png_path, dpi=PNG_DPI, bbox_inches="tight")
    fig.savefig(pdf_path, bbox_inches="tight")
    plt.close(fig)

    print(f"[INFO] Saved {png_path}")
    print(f"[INFO] Saved {pdf_path}")


# =========================
# SUMMARY
# =========================
def write_summary(title: str, items: List[Dict], fh):
    fh.write(title + "\n")
    for item in items:
        fh.write(
            f"{item['name']}, episode={item['episode']}, fitness={item['fitness']:.6f}\n"
        )
    fh.write("\n")


# =========================
# MAIN
# =========================
def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    grouped = collect_cases()

    best_stateless = select_one_episode_per_experiment(grouped["stateless"], "best")[:5]
    best_stateful = select_one_episode_per_experiment(grouped["stateful"], "best")[:5]

    worst_stateless = select_one_episode_per_experiment(grouped["stateless"], "worst")[:5]
    worst_stateful = select_one_episode_per_experiment(grouped["stateful"], "worst")[:5]

    print(f"[INFO] Best stateless count: {len(best_stateless)}")
    print(f"[INFO] Best stateful count: {len(best_stateful)}")
    print(f"[INFO] Worst stateless count: {len(worst_stateless)}")
    print(f"[INFO] Worst stateful count: {len(worst_stateful)}")

    make_grid_plot(
        stateless_items=best_stateless,
        stateful_items=best_stateful,
        out_stem="best_trajectories_comparison",
    )

    make_grid_plot(
        stateless_items=worst_stateless,
        stateful_items=worst_stateful,
        out_stem="worst_trajectories_comparison",
    )

    summary_path = os.path.join(OUT_DIR, "trajectory_selection_summary.txt")
    with open(summary_path, "w", encoding="utf-8") as fh:
        write_summary("BEST STATELESS", best_stateless, fh)
        write_summary("BEST STATEFUL", best_stateful, fh)
        write_summary("WORST STATELESS", worst_stateless, fh)
        write_summary("WORST STATEFUL", worst_stateful, fh)

    print(f"[INFO] Saved summary: {summary_path}")
    print(f"[INFO] Done. Outputs written to: {OUT_DIR}")


if __name__ == "__main__":
    main()
