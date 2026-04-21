#!/usr/bin/env bash
set -euo pipefail

BASE_DIR="${HOME}/tpg_codebase/tpg/experiments/paper_data"
OUT_DIR="${BASE_DIR}/best_agent_stats"

mkdir -p "${OUT_DIR}"

count=0
skipped=0

for exp_dir in "${BASE_DIR}"/gazebo_turtlebot4_*_f_*; do
    [[ -d "${exp_dir}" ]] || continue

    exp_name="$(basename "${exp_dir}")"
    src_csv="${exp_dir}/logs/best_agent/best_agent_stats.csv"
    dst_csv="${OUT_DIR}/${exp_name}.csv"

    if [[ ! -f "${src_csv}" ]]; then
        echo "[WARN] Missing: ${src_csv}"
        ((skipped+=1))
        continue
    fi

    cp -f "${src_csv}" "${dst_csv}"
    echo "[INFO] Copied:"
    echo "       ${src_csv}"
    echo "    -> ${dst_csv}"
    ((count+=1))
done

echo "[INFO] Done. Copied ${count} file(s), skipped ${skipped}."
