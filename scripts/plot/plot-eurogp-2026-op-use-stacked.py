import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
import seaborn as sns
import os, glob
from pandas.errors import EmptyDataError
from matplotlib.ticker import MaxNLocator

phase = 1

experiment_name_1 = "mj_reacher_dynamic_2025_11_12_18_51_52_ac712d60"
# experiment_name_2 = "mj_reacher_static_2025_11_12_18_51_52_ac712d60"
experiment_name_2 = "mj_reacher_static_2025_11_12_23_22_01_6062d257"

experiment_name_3 = "mj_ant_goal_dynamic_2025_11_12_18_51_52_ac712d60"
experiment_name_4 = "mj_ant_goal_static_2025_11_12_18_51_52_ac712d60"



base_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.getenv("TPG"), "experiments/"))

exp_dir_1 = os.path.join(base_path, experiment_name_1, "logs", "selection")
exp_dir_2 = os.path.join(base_path, experiment_name_2, "logs", "selection")
exp_dir_3 = os.path.join(base_path, experiment_name_3, "logs", "selection")
exp_dir_4 = os.path.join(base_path, experiment_name_4, "logs", "selection")

def process_csv(directory):
    files = glob.glob(os.path.join(directory, "selection.*.csv"))
    max_indices = {}

    for f in files:
        df = pd.read_csv(f)
        if not {"phase", "task_set"}.issubset(df.columns):
            continue

        mask = (df["phase"] == phase) & (df["task_set"] == "_0_")
        phase1_idx = df.index[mask]
        if len(phase1_idx) == 0:
            continue
        max_indices[f] = phase1_idx.max()

    if not max_indices:
        raise ValueError("No rows found where phase == 1 and task_set == '_0_'")

    common_max_index = min(max_indices.values())

    rows = []
    for f in files:
        df = pd.read_csv(f)
        if common_max_index < len(df):
            row = df.iloc[[common_max_index]].copy()
            row["source_file"] = os.path.basename(f)
            rows.append(row)

    result_df = pd.concat(rows, ignore_index=True)
    result_df = result_df[[col for col in result_df.columns if col.endswith("_op")]]
    omit_cols = ["scalar_uniform_set_op", "vector_uniform_set_op", "matrix_uniform_set_op",
                 "copy_scalars_to_vector_op", "obs_buff_slice_op"]
    result_df = result_df.drop(columns=omit_cols, errors="ignore")
    return result_df

def melt_nonzero(df):
    melted = df.melt(var_name="Column", value_name="Value")
    melted = melted[melted["Value"] != 0]
    return melted

# --- Process both experiments ---
result_df1 = process_csv(exp_dir_1)
result_df2 = process_csv(exp_dir_2)
result_df3 = process_csv(exp_dir_3)
result_df4 = process_csv(exp_dir_4)

# Melt for plotting
melted1 = melt_nonzero(result_df1)
melted2 = melt_nonzero(result_df2)
melted3 = melt_nonzero(result_df3)
melted4 = melt_nonzero(result_df4)

ordered_cols = result_df1.columns.tolist()
# renamed_cols = {
#     col: f"{i+1}: {col.replace('_op', '')}"
#     for i, col in enumerate(ordered_cols)
# }
renamed_cols = {
    col: f"{i+1}: {col.replace('_op', '').replace('_', ' ')}"
    for i, col in enumerate(ordered_cols)
}

melted1["Column"] = melted1["Column"].replace(renamed_cols)
melted2["Column"] = melted2["Column"].replace(renamed_cols)
melted3["Column"] = melted3["Column"].replace(renamed_cols)
melted4["Column"] = melted4["Column"].replace(renamed_cols)

ordered_labels = list(renamed_cols.values())

# --- Create stacked plots ---
fig = plt.figure(figsize=(14, 12))

gs = gridspec.GridSpec(
    5, 1,
    height_ratios=[1, 1, 0.01, 1, 1]  # spacer = row 2
)

ax1 = fig.add_subplot(gs[0])
ax2 = fig.add_subplot(gs[1])
ax3 = fig.add_subplot(gs[3])
ax4 = fig.add_subplot(gs[4])

# Top plot
sns.boxplot(x="Column", y="Value", data=melted1,
            order=ordered_labels, width=0.4, ax=ax1)
ax1.set_title("Reacher")
ax1.set_xlabel("")
ax1.set_ylabel("Count")
ax1.set_xticklabels([])
ax1.yaxis.set_major_locator(MaxNLocator(integer=True))

ax1.text(
    0.95, 0.95,              # x, y in axes fraction coordinates
    "Dynamic",
    ha='right', va='top',   # anchor the text to that corner
    transform=ax1.transAxes   # interpret coords as fraction of axes
)

# Mid-Top plot
sns.boxplot(x="Column", y="Value", data=melted2,
            order=ordered_labels, width=0.4, ax=ax2)
# ax1.set_title(f"Experiment 1: {experiment_name_1}")
ax2.set_xlabel("")
ax2.set_ylabel("Count")
ax2.set_xticklabels([])
ax2.yaxis.set_major_locator(MaxNLocator(integer=True))

ax2.text(
    0.95, 0.95,              # x, y in axes fraction coordinates
    "Static",
    ha='right', va='top',   # anchor the text to that corner
    transform=ax2.transAxes   # interpret coords as fraction of axes
)

# Mid-Bottom plot
sns.boxplot(x="Column", y="Value", data=melted3,
            order=ordered_labels, width=0.4, ax=ax3)
ax3.set_title("Ant-Goal")
ax3.set_xlabel("")
ax3.set_ylabel("Count")
ax3.set_xticklabels([])
ax3.yaxis.set_major_locator(MaxNLocator(integer=True))

ax3.text(
    0.95, 0.95,              # x, y in axes fraction coordinates
    "Dynamic",
    ha='right', va='top',   # anchor the text to that corner
    transform=ax3.transAxes   # interpret coords as fraction of axes
)

# Bottom plot
sns.boxplot(x="Column", y="Value", data=melted4,
            order=ordered_labels, width=0.4, ax=ax4)
# ax2.set_title(f"Experiment 2: {experiment_name_2}")
ax4.set_xlabel("")
ax4.set_ylabel("Count")

# Explicitly set tick labels AFTER plotting
ax4.set_xticks(range(len(ordered_labels)))
ax4.set_xticklabels(ordered_labels, rotation=90, fontsize=8)
ax4.yaxis.set_major_locator(MaxNLocator(integer=True))

ax4.text(
    0.95, 0.95,              # x, y in axes fraction coordinates
    "Static",
    ha='right', va='top',   # anchor the text to that corner
    transform=ax4.transAxes   # interpret coords as fraction of axes
)

# Adjust layout to make space for rotated labels
fig.subplots_adjust(left=0.1, right=0.95, top=0.93, bottom=0.35)

fig.savefig("op_use.pdf", format="pdf", bbox_inches='tight', pad_inches=0)
plt.show()
