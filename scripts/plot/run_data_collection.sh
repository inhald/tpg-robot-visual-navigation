#!/usr/bin/env bash
set -uo pipefail

EXPERIMENTS_DIR="${HOME}/tpg_codebase/tpg/experiments"
PAPER_DATA_DIR="${EXPERIMENTS_DIR}/paper_data"
MAX_JOBS="${MAX_JOBS:-4}"

cd "${EXPERIMENTS_DIR}" || exit 1

mapfile -d '' DIRS < <(
    find "${PAPER_DATA_DIR}" -maxdepth 1 -mindepth 1 -type d -name "gazebo_turtlebot4_*_f_*" -print0 | sort -z
)

run_one() {
    local full_dir="$1"
    local job_id="$2"

    local name
    name="$(basename "${full_dir}")"

    local link_path="${EXPERIMENTS_DIR}/${name}"
    local log_dir="${PAPER_DATA_DIR}/parallel_replay_logs"
    local partition="replay_${job_id}"
    local created_link=0

    mkdir -p "${log_dir}"

    # symlink
    if [[ ! -L "${link_path}" ]]; then
        ln -s "${full_dir}" "${link_path}"
        created_link=1
    fi

    echo "[INFO] ${name} → IGN_PARTITION=${partition}"

    IGN_PARTITION="${partition}" \
    tpg replay "${name}" \
        > "${log_dir}/${name}.out" \
        2> "${log_dir}/${name}.err" \
        < /dev/null

    if [[ "${created_link}" -eq 1 ]]; then
        rm "${link_path}"
    fi
}

export -f run_one
export EXPERIMENTS_DIR PAPER_DATA_DIR

active=0
job_id=0

for full_dir in "${DIRS[@]}"; do
    run_one "${full_dir}" "${job_id}" &
    ((active++))
    ((job_id++))

    if (( active >= MAX_JOBS )); then
        wait -n
        ((active--))
    fi
done

wait
echo "[INFO] All parallel replays completed."
