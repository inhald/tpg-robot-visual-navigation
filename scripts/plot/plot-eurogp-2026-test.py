import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import glob
import os

experiment_name_1 = "mj_reacher_dynamic_2025_11_12_18_51_52_ac712d60"
experiment_name_2 = "mj_reacher_static_2025_11_12_23_22_01_6062d257"
experiment_name_3 = "mj_ant_goal_dynamic_2025_11_13_11_57_01_6062d257"
experiment_name_4 = "mj_reacher_dynamic_2025_11_12_18_51_52_ac712d60"

base_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.getenv("TPG"), "experiments/"))

# ----- CONFIG: FOUR DIRECTORIES -----
dirs = {
    "Reacher Dynamic": os.path.join(base_path, experiment_name_1, "logs", "misc"),
    "Reacher Static": os.path.join(base_path, experiment_name_2, "logs", "misc"),
    "Ant-Goal Dynamic": os.path.join(base_path, experiment_name_3, "logs", "misc"),
    "Ant-Goal Static": os.path.join(base_path, experiment_name_1, "logs", "misc"),
}

# ----- LOAD AND LABEL CSV DATA -----
df_all = []

for label, d in dirs.items():
    pattern = os.path.join(d, "*.std")
    df_list = []

    for file in glob.glob(pattern):
        try:
            df = pd.read_csv(file)
            df["Directory"] = label   # tag with directory
            df_list.append(df)
        except Exception as e:
            print(f"Skipping {file}: {e}")

    if df_list:
        df_combined = pd.concat(df_list, ignore_index=True)
        df_all.append(df_combined)

# Combine all directories into one DataFrame
final_df = pd.concat(df_all, ignore_index=True)

print(final_df)

# ----- SINGLE PLOT WITH 4 BOXPLOTS -----
plt.figure(figsize=(5, 5))
sns.swarmplot(data=final_df, x="Directory", y="mean_outcome")

plt.xlabel("")
# plt.xticks(rotation=45)
labels = [t.get_text().replace(" ", "\n") for t in plt.xticks()[1]]
# Apply new labels, centered
plt.xticks(plt.xticks()[0], labels, ha="center")
plt.ylabel("Success Rate")
plt.tight_layout()
plt.show()