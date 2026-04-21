import os
import math
import glob
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
OUT_DIR = os.path.join(BASE_DIR, "bev")

# Approximate TurtleBot4 footprint radius [m]
ROBOT_RADIUS = 0.11

# Bonus-active radius around target [m]
TARGET_BONUS_RADIUS = 0.30
TARGET_WIDTH = 0.20544
TARGET_DEPTH = 0.055639

# Figure styling
FIGSIZE = (5.2, 5.2)
TRAJ_COLOR = "#1f77b4"
END_COLOR = "#d62728"
TARGET_FACE = "gold"
TARGET_EDGE = "#8a6d00"
BONUS_FACE = "gold"
BONUS_EDGE = "goldenrod"
OBS_FACE = "#e6e6e6"
OBS_EDGE = "black"
OBS_HATCH = "///"

# Padding around global bounds
PAD = 0.10

# Save DPI for PNG
PNG_DPI = 300


def infer_type_from_name(name: str) -> str:
    lname = name.lower()
    if "stateless" in lname:
        return "stateless"
    if "stateful" in lname:
        return "stateful"
    return "unknown"


def discover_experiments(base_dir: str):
    pattern = os.path.join(base_dir, "gazebo_turtlebot4_*_f_*")
    exp_dirs = [p for p in sorted(glob.glob(pattern)) if os.path.isdir(p)]
    return exp_dirs


def heading_line(ax, x, y, yaw, radius, color="k", lw=1.2, zorder=6):
    dx = radius * math.cos(yaw)
    dy = radius * math.sin(yaw)
    ax.plot([x, x + dx], [y, y + dy], color=color, linewidth=lw, zorder=zorder)


def compute_global_limits(traj_df, world_df, bonus_radius, pad):
    obstacle_xmin = (world_df["obstacle_x"] - world_df["obstacle_width"] / 2.0).min()
    obstacle_xmax = (world_df["obstacle_x"] + world_df["obstacle_width"] / 2.0).max()
    obstacle_ymin = (world_df["obstacle_y"] - world_df["obstacle_depth"] / 2.0).min()
    obstacle_ymax = (world_df["obstacle_y"] + world_df["obstacle_depth"] / 2.0).max()

    xmin = min(
        traj_df["robot_x"].min(),
        world_df["goal_x"].min() - bonus_radius,
        obstacle_xmin,
    ) - pad

    xmax = max(
        traj_df["robot_x"].max(),
        world_df["goal_x"].max() + bonus_radius,
        obstacle_xmax,
    ) + pad

    ymin = min(
        traj_df["robot_y"].min(),
        world_df["goal_y"].min() - bonus_radius,
        obstacle_ymin,
    ) - pad

    ymax = max(
        traj_df["robot_y"].max(),
        world_df["goal_y"].max() + bonus_radius,
        obstacle_ymax,
    ) + pad

    return xmin, xmax, ymin, ymax


def make_legend(ax):
    trajectory_handle = Line2D(
        [0], [0],
        color=TRAJ_COLOR,
        linewidth=2.0,
        label="trajectory"
    )

    start_fp_handle = Circle(
        (0, 0),
        0.12,
        fill=False,
        edgecolor=TRAJ_COLOR,
        linewidth=1.8,
        label="start footprint"
    )

    end_fp_handle = Circle(
        (0, 0),
        0.12,
        fill=False,
        edgecolor=END_COLOR,
        linestyle="--",
        linewidth=1.8,
        label="end footprint"
    )

    target_handle = Rectangle(
        (0, 0),
        1.0,
        0.35,
        facecolor=TARGET_FACE,
        edgecolor=TARGET_EDGE,
        linewidth=1.2,
        label="target"
    )

    bonus_handle = Circle(
        (0, 0),
        0.12,
        facecolor=BONUS_FACE,
        edgecolor=BONUS_EDGE,
        linestyle="--",
        linewidth=1.4,
        alpha=0.20,
        label="bonus-active region"
    )

    obstacle_handle = Rectangle(
        (0, 0),
        1.0,
        1.0,
        facecolor=OBS_FACE,
        edgecolor=OBS_EDGE,
        hatch=OBS_HATCH,
        linewidth=1.4,
        label="obstacle"
    )

    handles = [
        trajectory_handle,
        start_fp_handle,
        end_fp_handle,
        target_handle,
        bonus_handle,
        obstacle_handle,
    ]

    labels = [
        "trajectory",
        "start footprint",
        "end footprint",
        "target",
        "bonus-active region",
        "obstacle",
    ]

    ax.legend(
        handles,
        labels,
        loc="best",
        fontsize=8,
        frameon=True,
        framealpha=0.95,
        edgecolor="black",
        borderpad=0.4,
        handlelength=1.8,
        labelspacing=0.4,
    )


def plot_episode(ep, t, w, xlim, ylim, out_dir, exp_name):
    fig, ax = plt.subplots(figsize=FIGSIZE)

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
        linewidth=1.4,
        zorder=1,
    )
    ax.add_patch(obstacle)

    bonus_region = Circle(
        (w["goal_x"], w["goal_y"]),
        TARGET_BONUS_RADIUS,
        facecolor=BONUS_FACE,
        edgecolor=BONUS_EDGE,
        linestyle="--",
        linewidth=1.4,
        alpha=0.20,
        zorder=2,
    )
    ax.add_patch(bonus_region)

    target_rect = Rectangle(
        (
            w["goal_x"] - TARGET_WIDTH / 2.0,
            w["goal_y"] - TARGET_DEPTH / 2.0,
        ),
        TARGET_WIDTH,
        TARGET_DEPTH,
        facecolor=TARGET_FACE,
        edgecolor=TARGET_EDGE,
        linewidth=1.2,
        zorder=7,
    )
    ax.add_patch(target_rect)

    ax.plot(
        t["robot_x"],
        t["robot_y"],
        color=TRAJ_COLOR,
        linewidth=2.0,
        zorder=4,
    )

    x0 = t.iloc[0]["robot_x"]
    y0 = t.iloc[0]["robot_y"]
    yaw0 = t.iloc[0]["robot_yaw"]

    x1 = t.iloc[-1]["robot_x"]
    y1 = t.iloc[-1]["robot_y"]
    yaw1 = t.iloc[-1]["robot_yaw"]

    start_fp = Circle(
        (x0, y0),
        ROBOT_RADIUS,
        facecolor="none",
        edgecolor=TRAJ_COLOR,
        linewidth=1.8,
        zorder=5,
    )
    end_fp = Circle(
        (x1, y1),
        ROBOT_RADIUS,
        facecolor="none",
        edgecolor=END_COLOR,
        linestyle="--",
        linewidth=1.8,
        zorder=5,
    )
    ax.add_patch(start_fp)
    ax.add_patch(end_fp)

    ax.scatter(x0, y0, color=TRAJ_COLOR, s=28, zorder=6)
    ax.scatter(x1, y1, color=END_COLOR, s=28, zorder=6)

    heading_line(ax, x0, y0, yaw0, ROBOT_RADIUS, color=TRAJ_COLOR, lw=1.2, zorder=6)
    heading_line(ax, x1, y1, yaw1, ROBOT_RADIUS, color=END_COLOR, lw=1.2, zorder=6)

    ax.set_aspect("equal")
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"{exp_name} | Episode {ep}")
    ax.grid(False)

    for spine in ax.spines.values():
        spine.set_linewidth(1.0)

    make_legend(ax)
    plt.tight_layout()

    stem = f"{exp_name}_episode_{int(ep):03d}"
    png_path = os.path.join(out_dir, f"{stem}.png")
    pdf_path = os.path.join(out_dir, f"{stem}.pdf")

    plt.savefig(png_path, dpi=PNG_DPI, bbox_inches="tight")
    plt.savefig(pdf_path, bbox_inches="tight")
    plt.close(fig)

    print(f"[INFO] Saved {png_path}")
    print(f"[INFO] Saved {pdf_path}")


def load_csv(path: str):
    try:
        df = pd.read_csv(path)
    except (FileNotFoundError, EmptyDataError, pd.errors.ParserError, UnicodeDecodeError) as e:
        print(f"[WARN] Could not read {path}: {e}")
        return None

    if df.empty:
        print(f"[WARN] Empty CSV: {path}")
        return None

    df.columns = df.columns.astype(str).str.strip()
    return df


def process_experiment(exp_dir: str, out_root: str):
    exp_name = os.path.basename(exp_dir)
    exp_type = infer_type_from_name(exp_name)

    if exp_type == "unknown":
        print(f"[WARN] Skipping unknown type: {exp_name}")
        return 0

    best_agent_dir = os.path.join(exp_dir, "logs", "best_agent")
    traj_csv = os.path.join(best_agent_dir, "trajectory.csv")
    world_csv = os.path.join(best_agent_dir, "world_layout.csv")

    traj = load_csv(traj_csv)
    world = load_csv(world_csv)

    if traj is None or world is None:
        print(f"[WARN] Skipping {exp_name}: missing usable CSVs")
        return 0

    required_traj_cols = {"episode", "robot_x", "robot_y", "robot_yaw"}
    required_world_cols = {
        "episode",
        "goal_x",
        "goal_y",
        "obstacle_x",
        "obstacle_y",
        "obstacle_width",
        "obstacle_depth",
    }

    missing_traj = required_traj_cols - set(traj.columns)
    missing_world = required_world_cols - set(world.columns)

    if missing_traj:
        print(f"[WARN] {exp_name}: missing trajectory columns {sorted(missing_traj)}")
        return 0
    if missing_world:
        print(f"[WARN] {exp_name}: missing world columns {sorted(missing_world)}")
        return 0

    traj_eps = set(pd.to_numeric(traj["episode"], errors="coerce").dropna().astype(int).unique())
    world_eps = set(pd.to_numeric(world["episode"], errors="coerce").dropna().astype(int).unique())
    episodes = sorted(traj_eps.intersection(world_eps))

    if not episodes:
        print(f"[WARN] {exp_name}: no common episodes found")
        return 0

    x0, x1, y0, y1 = compute_global_limits(traj, world, TARGET_BONUS_RADIUS, PAD)
    xlim = (x0, x1)
    ylim = (y0, y1)

    type_dir = os.path.join(out_root, exp_type)
    os.makedirs(type_dir, exist_ok=True)

    count = 0
    for ep in episodes:
        t = traj[pd.to_numeric(traj["episode"], errors="coerce") == ep].copy()
        w_rows = world[pd.to_numeric(world["episode"], errors="coerce") == ep]

        if t.empty or w_rows.empty:
            continue

        w = w_rows.iloc[0]
        plot_episode(ep, t, w, xlim, ylim, type_dir, exp_name)
        count += 1

    return count


def copy_summary_plots(base_dir: str, out_root: str):
    src_dir = os.path.join(base_dir, "gazebo_curve_plots")
    dst_dir = os.path.join(out_root, "summary_plots")
    os.makedirs(dst_dir, exist_ok=True)

    if not os.path.isdir(src_dir):
        print(f"[WARN] Missing summary plots directory: {src_dir}")
        return

    for path in glob.glob(os.path.join(src_dir, "*")):
        if not os.path.isfile(path):
            continue
        name = os.path.basename(path)
        dst = os.path.join(dst_dir, name)
        with open(path, "rb") as fsrc, open(dst, "wb") as fdst:
            fdst.write(fsrc.read())
        print(f"[INFO] Copied summary plot: {name}")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(os.path.join(OUT_DIR, "stateful"), exist_ok=True)
    os.makedirs(os.path.join(OUT_DIR, "stateless"), exist_ok=True)

    experiments = discover_experiments(BASE_DIR)
    if not experiments:
        raise RuntimeError(f"No experiment folders found in: {BASE_DIR}")

    total = 0
    for exp_dir in experiments:
        total += process_experiment(exp_dir, OUT_DIR)

    copy_summary_plots(BASE_DIR, OUT_DIR)

    print(f"[INFO] Done. Saved {total} trajectory plot set(s) to: {OUT_DIR}")
    print(f"[INFO] Structure:")
    print(f"[INFO]   {OUT_DIR}/summary_plots")
    print(f"[INFO]   {OUT_DIR}/stateful")
    print(f"[INFO]   {OUT_DIR}/stateless")


if __name__ == "__main__":
    main()
