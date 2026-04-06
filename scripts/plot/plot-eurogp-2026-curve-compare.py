import numpy as np
import pandas as pd
from pandas.errors import EmptyDataError
import matplotlib.pyplot as plt
import matplotlib.style as style
# Use a colorblind-friendly style
style.use('tableau-colorblind10') # or 'seaborn-colorblind'
import os
import glob

phase=1
col_to_plot = "fitness_task_0"
# col_to_plot = "flop_complexity_task_0"
# col_to_plot = "program_count"

min_gen_cutoff = 100

# # experiment_name_1 = "mj_ant_goal_dynamic_2025_11_12_18_51_52_ac712d60"
# # # experiment_name_2 = "mj_ant_goal_static_2025_11_12_18_51_52_ac712d60"
# # # # experiment_name_2 = "mj_ant_goal_static_2025_11_12_23_22_01_6062d257"

# experiment_name_1 = "mj_reacher_dynamic_2025_11_12_18_51_52_ac712d60"
# # # experiment_name_2 = "mj_reacher_static_2025_11_12_18_51_52_ac712d60"
# # experiment_name_2 = "mj_reacher_static_2025_11_12_23_22_01_6062d257"

# experiment_name_1 = "mj_ant_goal_dynamic_2025_11_17_22_03_15_c096998c"
# experiment_name_2 = "mj_ant_goal_static_2025_11_17_22_03_15_c096998c"

# experiment_name_1 = "mj_reacher_dynamic_2025_11_18_21_15_15_c096998c"
# experiment_name_2 = "mj_reacher_static_2025_11_20_19_43_16_c096998c"

# experiment_name_1 = "mj_ant_goal_dynamic_2025_11_17_22_03_15_c096998c"
# experiment_name_2 = "mj_ant_goal_dynamic_2025_11_26_21_15_24_b142d7c8"

# experiment_name_1 = "mj_ant_goal_dynamic_2025_11_26_21_15_24_b142d7c8"
# base_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.getenv("TPG"), "experiments/"))
# exp_dir_1 = os.path.join(base_path, experiment_name_1, "logs", "selection")

# experiment_name_2 = "mj_ant_goal_dynamic_2025_11_27_22_30_38_d131ca61"
# base_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.getenv("HOME"), "dra_scratch/"))
# exp_dir_2 = os.path.join(base_path, experiment_name_2, "logs", "selection")

################################################################################

experiment_name_1 = "mj_ant_goal_dynamic_2025_12_06_15_52_50_ba94751a"
experiment_name_2 = "mj_ant_goal_static_2025_12_06_15_52_50_ba94751a" 

# experiment_name_1 = "mj_reacher_dynamic_2025_12_06_15_52_50_ba94751a"
# experiment_name_2 = "mj_reacher_static_2025_12_06_15_52_50_ba94751a" 

# experiment_name_1 = "mj_reacher_dynamic_2025_12_07_22_47_58_ba94751a"
# experiment_name_2 = "mj_reacher_static_2025_12_07_22_47_58_ba94751a"

base_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.getenv("TPG"), "experiments/"))
# base_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.getenv("HOME"), "dra_scratch/"))

exp_dir_1 = os.path.join(base_path, experiment_name_1, "logs", "selection")
exp_dir_2 = os.path.join(base_path, experiment_name_2, "logs", "selection")

def load_best_fitness_reps(exp_dir):
    pattern = os.path.join(exp_dir, "selection.*.csv")
    files = sorted(glob.glob(pattern))
    reps = []
    for f in files:
        try:
          df = pd.read_csv(f)
        except EmptyDataError:
          continue  
        if df.empty or len(df) < min_gen_cutoff:
           continue
        df = df[df["phase"] == phase]
        rslt = df.loc[df["task_set"] == "_0_", col_to_plot]
        reps.append(rslt.values)
        reps = [r for r in reps if len(r) > 0] #if a seed has a min of 0 (invalid)
    if not reps:
        return np.array([]), np.array([]), np.array([])
    
    min_len = min(len(r) for r in reps)
    truncated = [r[:min_len] for r in reps]
    data = np.vstack(truncated)
    gens = np.arange(min_len)
    means = data.mean(axis=0)
    maxs = data.max(axis=0)
    stds = data.std(axis=0)
    print(len(data))
    return gens, means, maxs, stds

def AddToPlot(gens, mean, max, std, label):
    ax = plt.gca()
    # color = next(ax._get_lines.prop_cycler)['color']
    ax.fill_between(gens, mean + 0.5*std, mean - 0.5*std, alpha=0.4)
    ax.plot(gens, mean, label=label, linewidth=1.5)
    previous_color = plt.gca().lines[-1].get_color()
    ax.plot(gens, max, label='', linewidth=1.5, linestyle='dashed', color=previous_color)

if __name__ == "__main__":
    gens1, mean1, max1, std1 = load_best_fitness_reps(exp_dir_1)
    gens2, mean2, max2, std2 = load_best_fitness_reps(exp_dir_2)

    max_gens_in_common = gens1
    if len(gens2) < len(gens1):
        max_gens_in_common = gens2

    mean1 = mean1[:len(max_gens_in_common)]
    max1 = max1[:len(max_gens_in_common)]
    std1 = std1[:len(max_gens_in_common)]

    mean2 = mean2[:len(max_gens_in_common)]
    max2 = max2[:len(max_gens_in_common)]
    std2 = std2[:len(max_gens_in_common)]
    
    if phase > 0:
       max_gens_in_common *= 100

    plt.figure(figsize=(8,6))
    # AddToPlot(max_gens_in_common, mean1, max1, std1, experiment_name_1)
    # AddToPlot(max_gens_in_common, mean2, max2, std2, experiment_name_2)

    AddToPlot(max_gens_in_common, mean1, max1, std1, "Dynamic")
    AddToPlot(max_gens_in_common, mean2, max2, std2, "Static")

    plt.legend(loc="best")
    plt.xlabel("Generations")
    # plt.ylabel("Mean FLOPs per Action")
    plt.ylabel("Best Fitness")
    plt.savefig("plot.pdf", format="pdf", bbox_inches='tight', pad_inches=0)
    plt.show()