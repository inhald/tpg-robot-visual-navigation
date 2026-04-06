import numpy as np
import pandas as pd
from pandas.errors import EmptyDataError
import matplotlib.pyplot as plt
import os
import glob

phase=1
# experiment_name_1 = "mj_ant_goal_dynamic_2025_11_04_22_57_04_1e3f1c3c"
# experiment_name_2 = "mj_ant_goal_static_2025_11_04_22_57_04_1e3f1c3c"

experiment_name_1 = "mj_reacher_dynamic_2025_11_10_23_32_43_d82e02ff"
experiment_name_2 = "mj_reacher_static_2025_11_10_23_32_43_d82e02ff"

base_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.getenv("TPG"), "experiments/"))

exp_dir_1 = os.path.join(base_path, experiment_name_1, "logs", "selection")
exp_dir_2 = os.path.join(base_path, experiment_name_2, "logs", "selection")

def process_csv(directory):
    # Directory containing your CSV files
    # directory = exp_dir_2 #"/path/to/csvs"

    # Find all matching files
    files = glob.glob(os.path.join(directory, "selection.*.csv"))

    max_indices = {}

    for f in files:
        df = pd.read_csv(f)
        # Skip files missing required columns
        if not {"phase", "task_set"}.issubset(df.columns):
            continue

        # Filter rows where phase == 1 and task_set == "_0_"
        mask = (df["phase"] == 1) & (df["task_set"] == "_0_")
        phase1_idx = df.index[mask]

        if len(phase1_idx) == 0:
            continue

        # Store the maximum (last) index meeting the condition
        max_indices[f] = phase1_idx.max()

    # Ensure we found valid rows
    if not max_indices:
        raise ValueError("No rows found where phase == 1 and task_set == '_0_'")

    # The common maximum row index present in all files
    common_max_index = min(max_indices.values())

    # Extract that row from each file
    rows = []
    for f in files:
        df = pd.read_csv(f)
        if common_max_index < len(df):
            row = df.iloc[[common_max_index]].copy()
            row["source_file"] = os.path.basename(f)
            rows.append(row)

    # Combine all extracted rows
    result_df = pd.concat(rows, ignore_index=True)
    result_df = result_df[[col for col in result_df.columns if col.endswith("_op")]]
    return result_df

#plot
import seaborn as sns
import matplotlib.pyplot as plt

result_df = process_csv(exp_dir_1)
result_df2 = process_csv(exp_dir_1)

# Exclude zero entries but keep all columns
melted = result_df.melt(var_name="Column", value_name="Value")
melted = melted[melted["Value"] != 0]

# Keep original order of columns
ordered_cols = result_df.columns.tolist()

# Add numbering prefix (1:, 2:, 3:, ...)
renamed_cols = {col: f"{i+1}: {col}" for i, col in enumerate(ordered_cols)}
melted["Column"] = melted["Column"].replace(renamed_cols)

fig, ax = plt.subplots(figsize=(14, 5))  # total figure size
# plt.figure(figsize=(max(8, len(result_df.columns) * 0.4), 5))
sns.boxplot(x="Column", y="Value", data=melted, order=renamed_cols.values(), width=0.4)

# Adjust the *plot box* size inside the figure
# left, right, bottom, top = [0–1] as fraction of figure
fig.subplots_adjust(left=0.1, right=0.95, bottom=0.25, top=0.9)

plt.xticks(rotation=90)
# plt.xticks([]) # removes tick labels
# plt.title("Numbered Boxplots (Zeros Excluded, All Columns Kept)")
plt.ylabel("Count")
plt.tight_layout()
fig.savefig("plot.pdf", format="pdf")
plt.show()

