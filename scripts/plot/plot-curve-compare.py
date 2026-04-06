import numpy as np
import pandas as pd
from pandas.errors import EmptyDataError
import matplotlib.pyplot as plt
import os
import glob

phase=0
col_to_plot = "fitness_task_0"
# col_to_plot = "op_complexity_task_0"
min_gen_cutoff = 100
# task_set = "_0_"


experiment_name_1 = "gazebo_turtlebot4_2026_02_21_15_35_28_717c2fec"
experiment_name_2 = "gazebo_turtlebot4_2026_02_23_10_45_48_717c2fec"


base_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.getenv("TPG"), "experiments/"))

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
            print("file:", f, "rows:", len(df), "empty:", df.empty,
      "phases:", df["phase"].unique() if "phase" in df else "NO_PHASE_COL",
      "task_sets:", df["task_set"].unique() if "task_set" in df else "NO_TASKSET_COL")
            continue  
        if df.empty or len(df) < min_gen_cutoff:
            print("file:", f, "rows:", len(df), "empty:", df.empty,
      "phases:", df["phase"].unique() if "phase" in df else "NO_PHASE_COL",
      "task_sets:", df["task_set"].unique() if "task_set" in df else "NO_TASKSET_COL")
            continue
        df = df[df["phase"] == phase]
        df_phase = df[df["phase"] == phase]
        if df_phase.empty:
            print("NO ROWS AFTER PHASE FILTER:", f, "wanted phase=", phase,
                  "available phases=", df["phase"].unique())
            continue
        rslt = df.loc[df["task_set"] == "_0_", col_to_plot]
        if rslt.empty:
            print("EMPTY RSLT (unexpected):", f)
            continue
        reps.append(rslt.values)
        reps = [r for r in reps if len(r) > 0] #if a seed has a min of 0 (invalid)
    if not reps:
        return np.array([]), np.array([]), np.array([]), np.array([])
    
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
    AddToPlot(max_gens_in_common, mean1, max1, std1, experiment_name_1)
    AddToPlot(max_gens_in_common, mean2, max2, std2, experiment_name_2)

    plt.legend(loc="lower right")
    plt.xlabel("Generations")
    plt.ylabel("Fitness")
    # plt.savefig("fitness_plot.pdf")
    plt.show()
