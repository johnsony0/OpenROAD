#!/usr/bin/env bash
# Run the A* hardware trial on several designs at once, one
# run_astar_hw_trial.sh per design, each into its own directory:
#   astar_hw_results/<platform>/<design>/<run>/   and   .../<run>.log
#
#   initialization/run_astar_hw_batch.sh [--run NAME] [--baseline] \
#       [--designs "sky130/aes sky130/gcd ..."]
#
# Defaults: --run full, the seven designs below. DRT_ASTAR_HW_* and DRT_*
# settings in the environment are passed through (e.g. DRT_ASTAR_HW_NET_COUNT,
# DRT_END_ITER). The trial script is copied into
# astar_hw_results/.batch_<run>/ first, so editing the repo copy cannot
# disturb a running batch. Check on it with initialization/astar_hw_status.sh.

set -uo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
run=full
baseline=""
designs="sky130/aes sky130/gcd sky130/ibex sky130/jpeg asap7/aes asap7/gcd nangate45/aes"
while (($#)); do
  case "$1" in
    --run) run=$2; shift ;;
    --baseline) baseline=--baseline ;;
    --designs) designs=$2; shift ;;
    -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

results="$root_dir/astar_hw_results"
snap="$results/.batch_$run"
mkdir -p "$snap"
cp "$root_dir/initialization/run_astar_hw_trial.sh" "$snap/"
echo "$(date '+%F %T') batch $run: $designs" >> "$snap/batch.log"

for pd in $designs; do
  p=${pd%/*} d=${pd#*/}
  out="$results/$p/$d/$run"
  mkdir -p "$out"
  (
    ASTAR_ROOT="$root_dir" bash "$snap/run_astar_hw_trial.sh" $baseline \
      --threads 1 --platform "$p" --design "$d" --out "$out" > "$out.log" 2>&1
    echo "$(date '+%F %T') $pd exit=$?" >> "$snap/batch.log"
  ) &
done
wait
echo "$(date '+%F %T') batch $run finished" >> "$snap/batch.log"
