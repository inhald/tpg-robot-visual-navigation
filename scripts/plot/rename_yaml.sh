#!/usr/bin/env bash
set -euo pipefail

BASE_DIR="${HOME}/tpg_codebase/tpg/experiments/paper_data"

cd "$BASE_DIR" || exit 1

for dir in gazebo_turtlebot4_*_f_*; do
    # skip if not a directory
    [[ -d "$dir" ]] || continue

    old_file="$dir/gazebo_turtlebot4.yaml"

    # skip if file doesn't exist
    [[ -f "$old_file" ]] || continue

    # new filename = folder name + .yaml
    new_file="$dir/${dir}.yaml"

    echo "[INFO] Renaming:"
    echo "       $old_file"
    echo "    -> $new_file"

    mv "$old_file" "$new_file"
done

echo "[INFO] Done."
