import pandas as pd
from pandas.errors import EmptyDataError
import matplotlib.pyplot as plt
import numpy as np
import os
import glob
import matplotlib.gridspec as gridspec

phase = 1

# --- Load both CSV files ---
csv1 = "/home/skelly/tpg/experiments/mj_reacher_dynamic_2025_11_12_18_51_52_ac712d60/logs/selection/selection.20.0.csv"
csv2 = "/home/skelly/tpg/experiments/mj_ant_goal_dynamic_2025_11_12_18_51_52_ac712d60/logs/selection/selection.30.0.csv"

df1 = pd.read_csv(csv1)
df2 = pd.read_csv(csv2)

# --- Function: process a CSV into quantiles per generation ---
def compute_ribbons(df, phase):
    m_cols = [col for col in df.columns if col.startswith("m_size")]
    df = df.reset_index(drop=True)
    df = df[df["phase"] == phase]

    data_for_plot = []
    for i in range(len(df)):
        row_data = []
        for col in m_cols:
            val = df.iloc[i][col]
            if pd.notna(val) and val > 0:
                col_num = int(col.replace("m_size_", ""))
                row_data.extend([col_num] * int(val))
        data_for_plot.append(row_data)

    # Compute row-wise quantiles
    q05 = []
    q25 = []
    q50 = []
    q75 = []
    q95 = []

    for row in data_for_plot:
        if len(row) == 0:
            q05.append(np.nan)
            q25.append(np.nan)
            q50.append(np.nan)
            q75.append(np.nan)
            q95.append(np.nan)
        else:
            q05.append(np.percentile(row, 5))
            q25.append(np.percentile(row, 25))
            q50.append(np.percentile(row, 50))
            q75.append(np.percentile(row, 75))
            q95.append(np.percentile(row, 95))

    return {
        "q05": np.array(q05),
        "q25": np.array(q25),
        "q50": np.array(q50),
        "q75": np.array(q75),
        "q95": np.array(q95),
        "x": np.arange(len(q50))
    }

# --- Compute for both datasets ---
r1 = compute_ribbons(df1, phase)
r2 = compute_ribbons(df2, phase)

fig = plt.figure(figsize=(14, 8))

gs = gridspec.GridSpec(
    5, 1,
    height_ratios=[0.4, 1, 0.025, 0.4, 1]  
)

ax1 = fig.add_subplot(gs[0])
ax2 = fig.add_subplot(gs[1])
ax3 = fig.add_subplot(gs[3])
ax4 = fig.add_subplot(gs[4])

# ---------- Top Plot (CSV 1) ----------
ax2.fill_between(r1["x"], r1["q05"], r1["q95"], alpha=0.15, label="5-95%")
ax2.fill_between(r1["x"], r1["q25"], r1["q75"], alpha=0.3, label="25-75%")
ax2.plot(r1["x"], r1["q50"], linewidth=2, label="Median")
ax2.set_xlabel("Generation")
ax2.set_ylabel("Memory sizes")
ax2.legend(ncol=3, loc="upper right")

# Top axis
ax2.set_xlim(0, len(r1["x"]) - 1)
ticks = np.linspace(0, len(r1["x"]) - 1, 6)
ax2.set_xticks(ticks)
ax2.set_xticklabels((ticks * 100).astype(int))

# ---------- Bottom Plot (CSV 2) ----------
ax4.fill_between(r2["x"], r2["q05"], r2["q95"], alpha=0.15, label="5-95%")
ax4.fill_between(r2["x"], r2["q25"], r2["q75"], alpha=0.3, label="25-75%")
ax4.plot(r2["x"], r2["q50"], linewidth=2, label="Median")
ax4.set_xlabel("Generation")
ax4.set_ylabel("Memory sizes")
ax4.legend(ncol=3, loc="upper right")

# Bottom axis
ax4.set_xlim(0, len(r2["x"]) - 1)
ticks = np.linspace(0, len(r2["x"]) - 1, 6)
ax4.set_xticks(ticks)
ax4.set_xticklabels((ticks * 100).astype(int))

################################
mask1 = df1["phase"] == phase
mask2 = df2["phase"] == phase

# Ensure program_count is aligned to ribbon x-axis lengths
prog1 = df1.loc[mask1, "program_count"].reset_index(drop=True)
prog2 = df2.loc[mask2, "program_count"].reset_index(drop=True)

# Plot program_count for CSV1 on ax1
ax1.plot(
    r1["x"],
    prog1,
    linewidth=2,
    label="program_count"
)
ax1.set_ylabel("# Program")
ax1.set_xlim(0, len(r1["x"]) - 1)
ticks = np.linspace(0, len(r1["x"]) - 1, 6)
ax1.set_xticks(ticks)
# ax1.set_xticklabels((ticks * 100).astype(int))
ax1.set_xticklabels([])
ax1.set_title("Reacher")

# Plot program_count for CSV2 on ax3
ax3.plot(
    r2["x"],
    prog2,
    linewidth=2,
    label="program_count"
)
ax3.set_ylabel("# Program")
ax3.set_xlim(0, len(r2["x"]) - 1)
ticks = np.linspace(0, len(r2["x"]) - 1, 6)
ax3.set_xticks(ticks)
# ax3.set_xticklabels((ticks * 100).astype(int))
ax3.set_xticklabels([])
ax3.set_title("Ant-Goal")
################################

plt.tight_layout()
plt.savefig("plot.pdf", format="pdf", bbox_inches='tight', pad_inches=0)
plt.show()
