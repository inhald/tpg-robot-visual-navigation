import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import PercentFormatter

# ============================================================
# PUBLICATION-STYLE GROUPED BAR CHART:
# REAL-WORLD SUCCESS RATE BY SCENE
# ============================================================

COLORS = {
    "Stateless": "#1f77b4",
    "Stateful": "#ff7f0e",
}

plt.rcParams.update({
    "font.size": 13,
    "axes.titlesize": 15,
    "axes.labelsize": 14,
    "xtick.labelsize": 12,
    "ytick.labelsize": 12,
    "legend.fontsize": 12,
    "axes.linewidth": 1.2,
    "xtick.major.width": 1.1,
    "ytick.major.width": 1.1,
    "xtick.major.size": 5,
    "ytick.major.size": 5,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
})

# ============================================================
# ENTER DATA HERE
# Each entry is the number of successful trials out of 9
# ============================================================

scenes = ["Scene 1", "Scene 2", "Scene 3"]

stateless_successes = np.array([7, 9, 7], dtype=float)
stateful_successes = np.array([7, 9, 9], dtype=float)

trials_per_scene = 9.0

# ============================================================
# VALIDATION
# ============================================================

if len(scenes) == 0:
    raise ValueError("scenes must not be empty")

if len(stateless_successes) != len(scenes):
    raise ValueError(
        f"stateless_successes must have length {len(scenes)}, "
        f"got {len(stateless_successes)}"
    )

if len(stateful_successes) != len(scenes):
    raise ValueError(
        f"stateful_successes must have length {len(scenes)}, "
        f"got {len(stateful_successes)}"
    )

if trials_per_scene <= 0:
    raise ValueError("trials_per_scene must be positive")

if np.any(stateless_successes < 0) or np.any(stateless_successes > trials_per_scene):
    raise ValueError(
        f"All stateless_successes values must lie in [0, {trials_per_scene}]"
    )

if np.any(stateful_successes < 0) or np.any(stateful_successes > trials_per_scene):
    raise ValueError(
        f"All stateful_successes values must lie in [0, {trials_per_scene}]"
    )

# ============================================================
# COMPUTE SUCCESS RATES
# success rate = 100 * successes / trials_per_scene
# ============================================================

stateless_rates = 100.0 * stateless_successes / trials_per_scene
stateful_rates = 100.0 * stateful_successes / trials_per_scene

# ============================================================
# BUILD FIGURE
# ============================================================

x = np.arange(len(scenes), dtype=float)
bar_width = 0.34

fig, ax = plt.subplots(figsize=(8.2, 5.2))

bars_stateless = ax.bar(
    x - bar_width / 2,
    stateless_rates,
    width=bar_width,
    label="Stateless",
    color=COLORS["Stateless"],
    edgecolor=COLORS["Stateless"],
    linewidth=1.4,
)

bars_stateful = ax.bar(
    x + bar_width / 2,
    stateful_rates,
    width=bar_width,
    label="Stateful",
    color=COLORS["Stateful"],
    edgecolor=COLORS["Stateful"],
    linewidth=1.4,
)

# ============================================================
# AXIS STYLING
# ============================================================

fig.suptitle(
    "Real-World Navigation Success Rate by Scene",
    fontsize=16,
    y=0.98,
)

ax.set_xlabel("Scene")
ax.set_ylabel("Success Rate (%)")

ax.set_xticks(x)
ax.set_xticklabels(scenes)
ax.set_ylim(0, 108)
ax.yaxis.set_major_formatter(PercentFormatter(xmax=100, decimals=0))

ax.grid(True, axis="y", alpha=0.25)
ax.set_axisbelow(True)

ax.spines["top"].set_visible(False)
ax.spines["right"].set_visible(False)

ax.legend(loc="upper right", frameon=True)

# Reserve vertical space so annotations do not collide with the title
fig.subplots_adjust(top=0.82)

# ============================================================
# BAR ANNOTATIONS
# ============================================================

def annotate_bars(ax, bars, counts, total_trials, y_offset=3):
    for bar, count in zip(bars, counts):
        height = float(bar.get_height())
        ax.annotate(
            f"{height:.1f}%\n({int(count)}/{int(total_trials)})",
            xy=(bar.get_x() + bar.get_width() / 2.0, height),
            xytext=(0, y_offset),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontsize=11,
        )

annotate_bars(ax, bars_stateless, stateless_successes, trials_per_scene)
annotate_bars(ax, bars_stateful, stateful_successes, trials_per_scene)

# ============================================================
# SAVE OUTPUTS
# ============================================================

here = os.path.dirname(os.path.abspath(__file__))
out_dir = os.path.join(here, "real_world_plots")
os.makedirs(out_dir, exist_ok=True)

pdf_path = os.path.join(out_dir, "real_world_success_rate_by_scene.pdf")
png_path = os.path.join(out_dir, "real_world_success_rate_by_scene.png")

plt.savefig(pdf_path, bbox_inches="tight")
plt.savefig(png_path, dpi=600, bbox_inches="tight")
plt.show()

print(f"[INFO] Saved PDF: {pdf_path}")
print(f"[INFO] Saved PNG: {png_path}")
