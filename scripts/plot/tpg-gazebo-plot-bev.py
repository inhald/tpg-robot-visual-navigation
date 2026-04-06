import os
import math
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.lines import Line2D

# =========================
# CONFIG
# =========================
TRAJ_CSV = "bev/trajectory.csv"
WORLD_CSV = "bev/world_layout.csv"
OUT_DIR = "bev/paper_plots"

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
# TARGET_EDGE = "black"
BONUS_FACE = "gold"
BONUS_EDGE = "goldenrod"
OBS_FACE = "#e6e6e6"
OBS_EDGE = "black"
OBS_HATCH = "///"

# Padding around global bounds
PAD = 0.10

# Save DPI for PNG
PNG_DPI = 300


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


def plot_episode(ep, t, w, xlim, ylim, out_dir):
    fig, ax = plt.subplots(figsize=FIGSIZE)

    # Obstacle with stripes
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

    # Bonus-active region
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

    # Target footprint (cereal box top view)
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


    # Trajectory
    ax.plot(
        t["robot_x"],
        t["robot_y"],
        color=TRAJ_COLOR,
        linewidth=2.0,
        zorder=4,
    )

    # Start/end states
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

    # Axes and style
    ax.set_aspect("equal")
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"Episode {ep}")
    ax.grid(False)

    for spine in ax.spines.values():
        spine.set_linewidth(1.0)

    make_legend(ax)

    plt.tight_layout()

    png_path = os.path.join(out_dir, f"episode_{ep:03d}.png")
    pdf_path = os.path.join(out_dir, f"episode_{ep:03d}.pdf")

    plt.savefig(png_path, dpi=PNG_DPI, bbox_inches="tight")
    plt.savefig(pdf_path, bbox_inches="tight")
    plt.close(fig)

    print(f"Saved {png_path}")
    print(f"Saved {pdf_path}")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    traj = pd.read_csv(TRAJ_CSV)
    world = pd.read_csv(WORLD_CSV)

    # Remove whitespace in headers
    traj.columns = traj.columns.str.strip()
    world.columns = world.columns.str.strip()

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
        raise ValueError(f"Missing trajectory columns: {sorted(missing_traj)}")
    if missing_world:
        raise ValueError(f"Missing world columns: {sorted(missing_world)}")

    traj_eps = set(traj["episode"].unique())
    world_eps = set(world["episode"].unique())
    episodes = sorted(traj_eps.intersection(world_eps))

    if not episodes:
        raise ValueError("No common episodes found in trajectory.csv and world_layout.csv")

    print(f"Found episodes: {episodes}")

    xlim_y = compute_global_limits(traj, world, TARGET_BONUS_RADIUS, PAD)
    xlim = (xlim_y[0], xlim_y[1])
    ylim = (xlim_y[2], xlim_y[3])

    for ep in episodes:
        t = traj[traj["episode"] == ep].copy()
        w_rows = world[world["episode"] == ep]

        if t.empty:
            print(f"Skipping episode {ep}: no trajectory data")
            continue
        if w_rows.empty:
            print(f"Skipping episode {ep}: no world layout data")
            continue

        w = w_rows.iloc[0]
        plot_episode(ep, t, w, xlim, ylim, OUT_DIR)

    print(f"Done. Saved plots to: {OUT_DIR}")


if __name__ == "__main__":
    main()
