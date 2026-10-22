#!/usr/bin/env bash

set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
user_name=${USER:-$(whoami)}

bazel=${BAZEL:-"$HOME/.local/bin/bazel"}
openroad=${OPENROAD:-"$root_dir/bazel-bin/openroad"}

# Write temporary dumps and results to local /tmp to bypass NFS locking issues
output_root=${DRT_DUMP_ROOT:-"a_star_dumps"}
results_root=${DRT_RESULTS_ROOT:-"/tmp/${user_name}/dump_sweep_results"}
input_root="$root_dir/initialization/dump_sweep_inputs"
sample_plan_py="$root_dir/initialization/sample_plan.py"

# gg_dump sampling: ~SAMPLE_BOXES random route boxes per iteration; searches per
# box scale with the iteration's share of TritonRoute searches (read from
# <input_dir>/search_distribution.txt, refreshed after every 1-thread run).
sample_boxes=${DRT_SAMPLE_BOXES:-30}
sample_max_searches=${DRT_SAMPLE_MAX_SEARCHES:-5}
sample_min_searches=${DRT_SAMPLE_MIN_SEARCHES:-1}
sample_box_cap=${DRT_SAMPLE_BOX_CAP:-40}
sample_seed=${DRT_DUMP_SEED:-1}

if [[ "$output_root" != /* ]]; then
  output_root="$root_dir/$output_root"
fi

usage() {
  cat <<EOF
Usage: $0 [--no-build]

Runs grid-graph and 1, 8, and 16 thread measurements for selected benchmarks.

Environment overrides:
  BAZEL            Bazel executable (default: ~/.local/bin/bazel)
  OPENROAD         OpenROAD executable (default: bazel-bin/openroad)
  DRT_DUMP_ROOT    Output root (default: /tmp/${user_name}/a_star_dumps)
  DRT_RESULTS_ROOT Results root (default: /tmp/${user_name}/dump_sweep_results)
  DRT_SAMPLE_BOXES         Target route boxes dumped per iteration (default: 30)
  DRT_SAMPLE_MAX_SEARCHES  Searches per box in the busiest iteration (default: 5)
  DRT_SAMPLE_MIN_SEARCHES  Minimum searches per box in any iteration (default: 1)
  DRT_SAMPLE_BOX_CAP       Hard cap on dumped boxes per iteration (default: 40)
  DRT_DUMP_SEED            Seed for the box/search sampling hash (default: 1)
EOF
}

build=1
while (($#)); do
  case "$1" in
    --no-build) build=0 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

if [[ $build -eq 1 ]]; then
  "$bazel" build //:openroad
fi

if [[ ! -x "$openroad" ]]; then
  echo "OpenROAD executable not found: $openroad" >&2
  echo "Run without --no-build or set OPENROAD." >&2
  exit 1
fi

prepare_flow() {
  local platform=$1
  local design=$2
  local flow=$3
  local input_dir="$input_root/$platform/$design"
  local results_dir="$results_root/$platform/$design/prepare"

  # Clean local /tmp results directory
  rm -rf "$results_dir"
  mkdir -p "$results_dir" "$input_dir"

  echo "[$platform/$design] prepare reusable DRT inputs"
  (
    cd "$root_dir/test"
    env \
      -u DRT_DUMP_GG_DIR \
      -u DRT_DUMP_EXP_DIR \
      -u DRT_DUMP_COST_DIR \
      DRT_PREPARE_DIR="$input_dir" \
      RESULTS_DIR="$results_dir" \
      "$openroad" -no_init -exit "$flow"
  )
}

# Populated by run_drt_only with the highest "iterN" number found among the
# files it dumped, so callers can reuse it (e.g. to target the final
# iteration precisely on a later run). Empty if no dump files were produced.
LAST_RUN_FINAL_ITER=""

run_drt_only() {
  local platform=$1
  local design=$2
  local label=$3
  local threads=$4
  local iter_filter=$5
  local sample_plan=${6:-}
  local input_dir="$input_root/$platform/$design"
  local dump_dir="$output_root/$platform/$design/$label"
  local results_dir="$results_root/$platform/$design/$label"
  local tech_lef cell_lef

  case "$platform" in
    nangate45)
      tech_lef="$root_dir/test/Nangate45/Nangate45_tech.lef"
      cell_lef="$root_dir/test/Nangate45/Nangate45_stdcell.lef"
      ;;
    asap7)
      tech_lef="$root_dir/test/asap7/asap7_tech_1x_201209.lef"
      cell_lef="$root_dir/test/asap7/asap7sc7p5t_28_R_1x_220121a.lef"
      ;;
    sky130)
      tech_lef="$root_dir/test/sky130hd/sky130hd.tlef"
      cell_lef="$root_dir/test/sky130hd/sky130hd_std_cell.lef"
      ;;
    *) echo "Unsupported platform: $platform" >&2; exit 1 ;;
  esac

  # Local /tmp directories wipe cleanly without NFS lock collisions
  rm -rf "$dump_dir" "$results_dir"
  mkdir -p "$dump_dir" "$results_dir"
  rm -f "$results_dir/final_iteration"

  local data_dir=""
  if [[ "$label" == "gg_dump" ]]; then
    data_dir="$dump_dir"
  fi
  local timing_dir=""
  if [[ "$label" == ggdump_thread* ]]; then
    timing_dir="$dump_dir"
  fi

  echo "[$platform/$design] $label"
  echo "  Dumps   -> $dump_dir"
  echo "  Results -> $results_dir"
  echo "  Iter    -> $iter_filter"

  # Without a sampling plan, fall back to first-come boxes/searches.
  local max_boxes=$sample_box_cap
  local max_searches=$sample_max_searches
  if [[ -n "$sample_plan" ]]; then
    echo "  Sample  -> $sample_plan"
  elif [[ "$label" == "gg_dump" ]]; then
    max_boxes=$sample_boxes
    echo "  Sample  -> none (first-come: $max_boxes boxes x $max_searches searches per iteration)"
  fi

  env \
    DRT_DUMP_GG_DIR="$dump_dir" \
    DRT_DUMP_GG_DATA_DIR="$data_dir" \
    DRT_DUMP_GG_SUMMARY_DIR= \
    DRT_DUMP_GG_TIMING_DIR="$timing_dir" \
    DRT_DUMP_EXP_DIR= \
    DRT_DUMP_COST_DIR= \
    DRT_DUMP_MIN_ITER="$iter_filter" \
    DRT_DUMP_ITER="$iter_filter" \
    DRT_DUMP_MAX_BOXES_PER_ITER="$max_boxes" \
    DRT_DUMP_MAX_SEARCHES_PER_BOX="$max_searches" \
    DRT_DUMP_SAMPLE_PLAN="$sample_plan" \
    DRT_DUMP_SEED="$sample_seed" \
    DRT_FINAL_ITER_FILE="$results_dir/final_iteration" \
    DRT_THREADS="$threads" \
    DRT_TECH_LEF="$tech_lef" \
    DRT_CELL_LEF="$cell_lef" \
    DRT_INPUT_DEF="$input_dir/${design}_${platform}_preroute.def" \
    DRT_INPUT_GUIDE="$input_dir/${design}_${platform}.route_guide" \
    DRT_DRC_REPORT="$results_dir/${design}_${platform}_drc.rpt" \
    DRT_MAZE_LOG="$results_dir/${design}_${platform}.maze.log" \
    DRT_OUTPUT_DEF="$results_dir/${design}_${platform}.def" \
    "$openroad" -no_init -exit "$root_dir/initialization/run_drt_only.tcl"

}

run_sweep() {
  local platform=$1
  local design=$2

  # 1) Discovery run: perform DRT without graph-data dumps. DRT writes the
  #    terminal iteration once to the results directory.
  # run_drt_only "$platform" "$design" iteration-discover 1 ""
  # local final_iter
  # final_iter=$(cat "$results_root/$platform/$design/iteration-discover/final_iteration")

  # if [[ -z "$final_iter" || ! "$final_iter" =~ ^[0-9]+$ ]]; then
  #  echo "[$platform/$design] ERROR: could not determine final iteration; skipping gg-dump/threads runs" >&2
  #  return 1
  # fi
  # echo "[$platform/$design] final iteration detected: $final_iter"

  target_iter=""

  # 2) Dump sampled graph data, using the stored search distribution if any.
  local dist_file="$input_root/$platform/$design/search_distribution.txt"
  local sample_plan=""
  if [[ -f "$dist_file" ]]; then
    sample_plan=$(python3 "$sample_plan_py" plan -v "$dist_file" \
                    --boxes "$sample_boxes" \
                    --max-searches "$sample_max_searches" \
                    --min-searches "$sample_min_searches")
  else
    echo "[$platform/$design] WARNING: no $dist_file; gg_dump uses first-come sampling." >&2
    echo "[$platform/$design] It is written after the 1-thread run; re-run gg_dump afterwards for random sampling." >&2
  fi
  run_drt_only "$platform" "$design" gg_dump 1 "$target_iter" "$sample_plan"

  # 3-5) Timing-only runs at 1/8/16 threads. Use the same final iteration
  #    while leaving DRT_DUMP_GG_DATA_DIR empty, so graph data is not dumped.
  for threads in 1 8 16 32 48; do
    run_drt_only "$platform" "$design" "ggdump_thread${threads}" "$threads" "$target_iter"
    if [[ $threads -eq 1 ]]; then
      # Persist the per-iteration distribution; later sweeps wipe ggdump_thread1.
      python3 "$sample_plan_py" extract \
        "$output_root/$platform/$design/ggdump_thread1" "$dist_file"
    fi
  done
}

while read -r platform design flow; do
  # Skip empty lines or commented-out entries
  [[ -z "$platform" || "$platform" == \#* ]] && continue

  input_dir="$input_root/$platform/$design"
  if [[ ! -f "$input_dir/${design}_${platform}_preroute.def" || \
        ! -f "$input_dir/${design}_${platform}.route_guide" ]]; then
    prepare_flow "$platform" "$design" "$flow"
  fi

  run_sweep "$platform" "$design"
done <<'EOF'
asap7 gcd gcd_asap7.tcl
nangate45 aes aes_nangate45.tcl
asap7 aes aes_asap7.tcl
sky130 aes aes_sky130hd.tcl
sky130 ibex ibex_sky130hd.tcl
sky130 jpeg jpeg_sky130hd.tcl
EOF