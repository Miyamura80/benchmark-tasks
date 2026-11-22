#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")" && pwd)"
out="${1:?Pass an unused run directory}"
mkdir "$out"
instruction="$(python -c 'import json,sys; print(json.load(open(sys.argv[1]))["instruction"])' "$root/task.json")"
inspect-robots run --embodiment yam_arena --policy dream \
  -E "task=$root/task.py" -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$instruction" --scorer task_score --max-steps 1500 --seed 0 \
  --no-prompt --fail-on-error 1 --store-frames --log-dir "$out"
for log in "$out"/*.json; do
  inspect-robots video "$log"
done
