#!/usr/bin/env bash
# Run TritonRoute with the A* hardware trial (see src/drt/src/dr/AstarHwTrial.h)
# on one prepared design, optionally next to a CPU-only baseline run.
#
#   initialization/run_astar_hw_trial.sh [--baseline] [--no-trial] \
#       [--platform nangate45] [--design gcd] [--threads 1] [--out DIR]
#
# --baseline also runs the same inputs CPU-only (DRT_ASTAR_HW_MODE=record),
# checks that the two runs committed the same routing (output DEF and DRC
# report), which is the guarantee the trial is built around, and scores the
# trial with the baseline's clean CPU timing. Results land in --out
# (default astar_hw_results/<platform>/<design>); score.csv is the summary.
#
# Environment overrides: OPENROAD, ASTAR_ACCEL (a-star-accelerator checkout),
# DRT_ASTAR_HW_NET_PROB, DRT_ASTAR_HW_NET_COUNT, DRT_ASTAR_HW_MIN_ITER, DRT_ASTAR_HW_MAX_ITER,
# DRT_ASTAR_HW_KEEP_JOBS, DRT_ASTAR_HW_CLK_MHZ, DRT_ASTAR_HW_SEED,
# DRT_ASTAR_HW_CONVERT (cpp | python | both, see src/drt/src/dr/AstarHwTrial.h),
# DRT_END_ITER (stop after that DR iteration), DRT_VERBOSE (default 1: the
# router's "Completing N%" lines in openroad.log).
#
# Progress: <out>/trial/hw/progress.log gets a line per trialed net and per DR
# iteration; initialization/astar_hw_status.sh summarizes every run.

set -euo pipefail

# ASTAR_ROOT lets run_astar_hw_batch.sh run a snapshot copy of this script.
root_dir=${ASTAR_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}
openroad=${OPENROAD:-"$root_dir/bazel-bin/openroad"}
accel=${ASTAR_ACCEL:-"$root_dir/../a_star_testing/a-star-accelerator"}
input_root="$root_dir/initialization/dump_sweep_inputs"

platform=nangate45
design=gcd
threads=1
out=""
baseline=0
trial=1
while (($#)); do
  case "$1" in
    --platform) platform=$2; shift ;;
    --design) design=$2; shift ;;
    --threads) threads=$2; shift ;;
    --out) out=$2; shift ;;
    --baseline) baseline=1 ;;
    --no-trial) trial=0 ;;
    -h|--help) sed -n '2,23p' "$0"; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done
out=${out:-"$root_dir/astar_hw_results/$platform/$design"}

case "$platform" in
  nangate45)
    tech_lef="$root_dir/test/Nangate45/Nangate45_tech.lef"
    cell_lef="$root_dir/test/Nangate45/Nangate45_stdcell.lef" ;;
  asap7)
    tech_lef="$root_dir/test/asap7/asap7_tech_1x_201209.lef"
    cell_lef="$root_dir/test/asap7/asap7sc7p5t_28_R_1x_220121a.lef" ;;
  sky130)
    tech_lef="$root_dir/test/sky130hd/sky130hd.tlef"
    cell_lef="$root_dir/test/sky130hd/sky130hd_std_cell.lef" ;;
  *) echo "Unsupported platform: $platform" >&2; exit 1 ;;
esac
input_dir="$input_root/$platform/$design"
for f in "$input_dir/${design}_${platform}_preroute.def" \
         "$input_dir/${design}_${platform}.route_guide"; do
  [[ -f "$f" ]] || { echo "Missing $f (run run_drt_dump_sweep.sh first)" >&2; exit 1; }
done

# One TritonRoute run. $1 = label, $2 = output dir, $3 = trial|record.
run_drt() {
  local label=$1 hw_out=$2 mode=$3
  local res="$out/$label"
  rm -rf "$res"
  mkdir -p "$res"
  echo "[$platform/$design] $label run ($threads thread(s)) -> $res"
  env \
    -u DRT_DUMP_GG_DIR -u DRT_DUMP_GG_DATA_DIR -u DRT_DUMP_GG_TIMING_DIR \
    -u DRT_DUMP_GG_SUMMARY_DIR -u DRT_DUMP_EXP_DIR -u DRT_DUMP_COST_DIR \
    DRT_ASTAR_HW_OUT="$hw_out" \
    DRT_ASTAR_HW_MODE="$mode" \
    DRT_ASTAR_HW_RUN_ID="$label" \
    DRT_ASTAR_HW_CMD="python3 $accel/tests/hw_job.py --cache $out/simv_cache" \
    DRT_THREADS="$threads" \
    DRT_VERBOSE="${DRT_VERBOSE:-1}" \
    DRT_TECH_LEF="$tech_lef" \
    DRT_CELL_LEF="$cell_lef" \
    DRT_INPUT_DEF="$input_dir/${design}_${platform}_preroute.def" \
    DRT_INPUT_GUIDE="$input_dir/${design}_${platform}.route_guide" \
    DRT_DRC_REPORT="$res/${design}_${platform}_drc.rpt" \
    DRT_MAZE_LOG="$res/${design}_${platform}.maze.log" \
    DRT_OUTPUT_DEF="$res/${design}_${platform}.def" \
    /usr/bin/time -f "%e" -o "$res/wall_s" \
    "$openroad" -no_init -exit "$root_dir/initialization/run_drt_only.tcl" \
    > "$res/openroad.log" 2>&1
  echo "  done in $(cat "$res/wall_s") s"
}

if [[ $trial -eq 1 ]]; then
  run_drt trial "$out/trial/hw" trial
fi
if [[ $baseline -eq 1 ]]; then
  run_drt baseline "$out/baseline/hw" record
fi

if [[ $trial -eq 1 && $baseline -eq 1 ]]; then
  # write_def stamps nothing run-specific, so the committed routing must match
  # byte for byte; the DRC report must match too.
  status=0
  for f in "${design}_${platform}.def" "${design}_${platform}_drc.rpt"; do
    if cmp -s "$out/trial/$f" "$out/baseline/$f"; then
      echo "  identical: $f"
    else
      echo "  DIFFERENT: $f  (the trial changed the committed routing)"
      status=1
    fi
  done
  python3 "$root_dir/initialization/astar_hw_score.py" "$out/trial/hw" \
    --baseline "$out/baseline/hw" || status=1
  exit $status
elif [[ $trial -eq 1 ]]; then
  python3 "$root_dir/initialization/astar_hw_score.py" "$out/trial/hw"
fi
