#!/usr/bin/env bash

set -euo pipefail

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
user_name=${USER:-$(whoami)}

bazel=${BAZEL:-"$HOME/.local/bin/bazel"}
openroad=${OPENROAD:-"$root_dir/bazel-bin/openroad"}
results_root=${FLOW_MEASURE_RESULTS_ROOT:-"/tmp/${user_name}/flow_measure_results"}
flow_threads=${FLOW_THREADS:-48}
combined_csv="$root_dir/initialization/runtime_data.csv"
timings_dir="$results_root/timings"

usage() {
  cat <<EOF
Usage: $0 [--no-build]

Runs the full test flow with per-section timing enabled for each listed
design/PDK pair. Results are combined into initialization/runtime_data.csv
with one column per pair.
The flow runs with $flow_threads OpenROAD threads by default.

Environment overrides:
  BAZEL                      Bazel executable (default: ~/.local/bin/bazel)
  OPENROAD                   OpenROAD executable (default: bazel-bin/openroad)
  FLOW_MEASURE_RESULTS_ROOT  Results root (default: /tmp/${user_name}/flow_measure_results)
  FLOW_THREADS               OpenROAD thread count (default: 48)
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

mkdir -p "$results_root"
mkdir -p "$timings_dir"

run_flow() {
  local pdk=$1
  local design=$2
  local flow=$3
  local pair_dir="$results_root/$pdk"
  local results_dir="$pair_dir/${design}_results"
  local timing_csv="$timings_dir/${pdk}__${design}.csv"
  local log_file="$pair_dir/${design}_${pdk}.log"

  mkdir -p "$pair_dir" "$results_dir"
  echo "[$pdk/$design] running $flow"
  echo "  Threads -> $flow_threads"
  echo "  Combined CSV -> $combined_csv"

  (
    cd "$root_dir/test"
    env \
      DRT_THREADS="$flow_threads" \
      FLOW_MEASURE_TIMING=1 \
      FLOW_TIMING_FILE="$timing_csv" \
      RESULTS_DIR="$results_dir" \
      "$openroad" -no_init -exit "$flow"
  ) 2>&1 | tee "$log_file"
}

while read -r pdk design flow; do
  [[ -z "$pdk" || "$pdk" == \#* ]] && continue
  run_flow "$pdk" "$design" "$flow"
done <<'EOF'
nangate45 gcd gcd_nangate45.tcl
asap7 gcd gcd_asap7.tcl
nangate45 aes aes_nangate45.tcl
asap7 aes aes_asap7.tcl
sky130hd aes aes_sky130hd.tcl
sky130hd gcd gcd_sky130hd.tcl
sky130hd ibex ibex_sky130hd.tcl
sky130hd jpeg jpeg_sky130hd.tcl
EOF

python3 - "$timings_dir" "$combined_csv" <<'PY'
import csv
import glob
import os
import sys

timings_dir, output_path = sys.argv[1:]
timing_files = sorted(glob.glob(os.path.join(timings_dir, "*.csv")))
if not timing_files:
    raise SystemExit("No flow timing CSVs were produced")

pairs = {}
sections = []
for timing_file in timing_files:
    pdk, design = os.path.basename(timing_file)[:-4].split("__", 1)
    pair = f"{pdk}/{design}"
    with open(timing_file, newline="") as stream:
        for row in csv.DictReader(stream):
            section = row["section"]
            if section not in sections:
                sections.append(section)
            pairs.setdefault(pair, {})[section] = row["elapsed_seconds"]

with open(output_path, "w", newline="") as stream:
    writer = csv.writer(stream)
    pair_names = sorted(pairs)
    writer.writerow(["section", *pair_names])
    for section in sections:
        writer.writerow(
            [section, *(pairs[pair].get(section, "") for pair in pair_names)]
        )

print(f"Combined flow timings -> {output_path}")
PY
