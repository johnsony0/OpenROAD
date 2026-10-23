#!/usr/bin/env bash
# Run the A* hardware trial on several designs at once, one
# run_astar_hw_trial.sh per design, each into its own directory:
#   astar_hw_results/<platform>/<design>/<run>/   and   .../<run>.log
#
#   initialization/run_astar_hw_batch.sh [--run NAME] [--baseline] \
#       [--threads N] [--baseline-threads N] \
#       [--designs "sky130/jpeg:6 sky130/gcd ..."]
#
# A design may carry its own trial thread count as <platform>/<design>:<N>;
# others use --threads (default 1). The CPU-only baseline that supplies the
# iteration timing runs with --baseline-threads (default 1); routing does not
# depend on the thread count, so the DEF check still holds.
# Defaults: --run full, the seven designs below. DRT_ASTAR_HW_* and DRT_*
# settings in the environment are passed through (e.g. DRT_ASTAR_HW_NET_COUNT,
# DRT_END_ITER). The trial script is copied into
# astar_hw_results/.batch_<run>/ first, so editing the repo copy cannot
# disturb a running batch. Check on it with initialization/astar_hw_status.sh.

set -uo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
run=full
baseline=""
threads=1
baseline_threads=1
designs="sky130/aes sky130/gcd sky130/ibex sky130/jpeg asap7/aes asap7/gcd nangate45/aes"
while (($#)); do
  case "$1" in
    --run) run=$2; shift ;;
    --baseline) baseline=--baseline ;;
    --designs) designs=$2; shift ;;
    --threads) threads=$2; shift ;;
    --baseline-threads) baseline_threads=$2; shift ;;
    -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

results="$root_dir/astar_hw_results"
snap="$results/.batch_$run"
mkdir -p "$snap"
cp "$root_dir/initialization/run_astar_hw_trial.sh" "$snap/"
echo "$(date '+%F %T') batch $run: $designs (default $threads thread(s), baseline $baseline_threads)" >> "$snap/batch.log"

for spec in $designs; do
  pd=${spec%%:*}
  n=$threads
  [[ $spec == *:* ]] && n=${spec#*:}
  p=${pd%/*} d=${pd#*/}
  out="$results/$p/$d/$run"
  mkdir -p "$out"
  (
    ASTAR_ROOT="$root_dir" bash "$snap/run_astar_hw_trial.sh" $baseline \
      --threads "$n" --baseline-threads "$baseline_threads" \
      --platform "$p" --design "$d" --out "$out" > "$out.log" 2>&1
    echo "$(date '+%F %T') $pd exit=$?" >> "$snap/batch.log"
  ) &
done
wait
echo "$(date '+%F %T') batch $run finished" >> "$snap/batch.log"
