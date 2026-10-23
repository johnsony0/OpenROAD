#!/usr/bin/env bash
# Show how every A* hardware trial run of one name is doing.
#
#   initialization/astar_hw_status.sh [RUN] [QUIET_MIN]
#
# RUN defaults to "full" (the run_astar_hw_batch.sh --run name). For each
# astar_hw_results/<platform>/<design>/RUN it prints the phase, the newest
# progress.log line (one per trialed net and per DR iteration) and the
# router's latest "Completing" line, each with its age. A run whose files
# have not changed for QUIET_MIN minutes (default 60) is marked QUIET, and
# one with no live openroad process of its own is marked STOPPED.

root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
run=${1:-full}
quiet_min=${2:-60}
now=$(date +%s)

age() {  # minutes since $1 was modified
  [[ -e $1 ]] || { echo "-"; return; }
  echo $(( (now - $(stat -c %Y "$1")) / 60 ))m
}

# Output dirs of the live router processes (their DRT_ASTAR_HW_OUT).
live=$(for p in $(pgrep -u "$USER" -f openroad); do
         tr '\0' '\n' < /proc/$p/environ 2>/dev/null | sed -n 's/^DRT_ASTAR_HW_OUT=//p'
       done)

shopt -s nullglob
for dir in "$root_dir"/astar_hw_results/*/*/"$run"; do
  [[ -d $dir ]] || continue
  name=${dir#"$root_dir"/astar_hw_results/}
  name=${name%/"$run"}
  log="$dir.log"
  prog="$dir/trial/hw/progress.log"
  orlog="$dir/trial/openroad.log"
  [[ -s $dir/baseline/wall_s || -f $dir/baseline/openroad.log ]] && orlog="$dir/baseline/openroad.log"
  if grep -qE "DIFFERENT|Traceback|command not found" "$log" 2>/dev/null; then
    phase=FAILED
  elif grep -q "wrote .*score.csv" "$log" 2>/dev/null; then
    phase=done
  elif [[ -f $dir/baseline/openroad.log ]]; then
    phase=baseline
  elif [[ -f $dir/trial/openroad.log ]]; then
    phase=trial
  else
    phase=starting
  fi
  newest=0
  for f in "$prog" "$dir/trial/openroad.log" "$dir/baseline/openroad.log" "$log"; do
    [[ -e $f ]] && (( $(stat -c %Y "$f") > newest )) && newest=$(stat -c %Y "$f")
  done
  if [[ $phase != done && $phase != FAILED ]]; then
    if ! grep -q "^$dir/" <<< "$live"; then
      phase="$phase STOPPED"
    elif (( newest > 0 && now - newest > quiet_min * 60 )); then
      phase="$phase QUIET"
    fi
  fi
  printf '%-16s %-14s\n' "$name" "$phase"
  [[ -s $prog ]] && printf '    progress (%s ago): %s\n' "$(age "$prog")" "$(tail -1 "$prog" | cut -c1-150)"
  last=$(grep -E "Completing|optimization iteration|Start detail routing" "$orlog" 2>/dev/null | tail -1)
  [[ -n $last ]] && printf '    router   (%s ago): %s\n' "$(age "$orlog")" "$(echo "$last" | cut -c1-150)"
  if [[ $phase == done ]]; then
    grep -E "identical|DIFFERENT" "$log" | sed 's/^ */    /'
    grep -E "^ +TOTAL" "$log" | sed 's/^ */    /' | cut -c1-150
  fi
done
batch="$root_dir/astar_hw_results/.batch_$run/batch.log"
if [[ -f $batch ]]; then
  echo "batch log:"
  tail -n 9 "$batch" | sed 's/^/    /'
fi
