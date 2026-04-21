#!/usr/bin/env bash
set -u

BASE_DIR="${HOME}/tpg_codebase/tpg/experiments/paper_data"
PYTHON_BIN="${PYTHON_BIN:-python3}"

OUT_DIR="${BASE_DIR}/paper_figures"
LOG_DIR="${OUT_DIR}/logs"
TIMESTAMP="$(date +%Y_%m_%d_%H_%M_%S)"

mkdir -p "${OUT_DIR}" "${LOG_DIR}"
mkdir -p "${BASE_DIR}/gazebo_curve_plots" "${BASE_DIR}/real_world_plots"

cd "${BASE_DIR}" || exit 1

# Only include scripts that actually exist
SCRIPTS=(
  "tpg-gazebo-plot-compare.py"
  "tpg-gazebo-std-dev.py"
  "tpg-gazebo-flops.py"
  "tpg-gazebo-eff-instruction-count.py"
  "tpg-gazebo-stateful-stateless-box.py"
)

if [[ -f "tpg-real-world.py" ]]; then
  SCRIPTS+=("tpg-real-world.py")
elif [[ -f "tpg-gazebo-plot-bev.py" ]]; then
  SCRIPTS+=("tpg-gazebo-plot-bev.py")
fi

echo "[INFO] Running scripts from ${BASE_DIR}"

for script in "${SCRIPTS[@]}"; do
  echo "[INFO] Running ${script}"
  if ! "${PYTHON_BIN}" "${script}" > "${LOG_DIR}/${script%.py}_${TIMESTAMP}.log" 2>&1; then
    echo "[WARN] ${script} failed. See log: ${LOG_DIR}/${script%.py}_${TIMESTAMP}.log"
  fi
done

echo "[INFO] Copying figures..."

find "${BASE_DIR}/gazebo_curve_plots" -maxdepth 1 -type f \
  \( -iname "*.pdf" -o -iname "*.png" \) \
  -exec cp -f {} "${OUT_DIR}/" \;

find "${BASE_DIR}/real_world_plots" -maxdepth 1 -type f \
  \( -iname "*.pdf" -o -iname "*.png" \) \
  -exec cp -f {} "${OUT_DIR}/" \;

echo "[INFO] Final contents of ${OUT_DIR}:"
find "${OUT_DIR}" -maxdepth 1 -type f | sort
