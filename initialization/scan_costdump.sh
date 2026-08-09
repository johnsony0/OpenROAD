#!/usr/bin/env bash
#
# Scan FlexGridGraph cost dumps for cost terms the accelerator does not model yet.
#
# Reads every *.txt under a costdump directory, parses the four record kinds
# emitted by the maze-search instrumentation, and reports every record that
# trips a check -- with counts and file:line locations.
#
#   ./initialization/scan_costdump.sh top_gcd_snippets/costdump
#
# The default checks are the terms we currently punt on:
#
#   est:forbidden != 0    forbidden-length penalty folded into the heuristic
#   next:v2v      != 0    via-to-via spacing cost in getNextPathCost()
#   next:vtlen    != 0    via-to-via *length* cost in getNextPathCost()
#
# Any other field can be checked from the command line -- the parser exposes
# every field of every record kind, so this stays useful as more terms get
# ported:
#
#   --check 'base:costs.drc > 0'
#   --check 'base:flags.marker != 0'
#   --check 'next:turnCost != 0'
#   --check 'est:bendCnt >= 3'
#
# Record kinds and their fields (see the '#' header lines in the dump itself):
#
#   expanding  x y z ptX ptY cost pathCost lastDir
#   est        nx ny nz dir manX manY manZ bendCnt forbidden finalEstCost
#   base       dir edgeLen flags.<name> costs.<name> totalBaseCost
#   next       currPathCosts currDir nextDir edgeLength turnCost v2v vtlen
#              finalNextCost vlenX vlenY currViaUp prevViaUp tLen tLenViaUp
#
# Needs only bash + awk + find, so it runs inside the openroad_sandbox image.

set -uo pipefail

# FlexGridGraph writes std::numeric_limits<frCoord>::max() for "no via seen yet"
# on vlenX/vlenY/tLen.  Those are sentinels, not real lengths, so they are
# skipped unless --keep-sentinel is given.
INT_MAX=2147483647

DEFAULT_CHECKS=(
  "est:forbidden != 0"
  "next:v2v != 0"
  "next:vtlen != 0"
)

usage() {
  sed -n '2,/^# Needs only/p' "$0" | sed 's/^#\{1,2\} \{0,1\}//'
  cat <<'EOF'

usage: scan_costdump.sh [PATH] [options]

  PATH               costdump directory or a single dump file
                     (default: top_gcd_snippets/costdump)
  --check EXPR       extra check 'kind:field OP value'; repeatable
                     OP is one of != == >= <= > <
  --only             use only --check expressions, drop the defaults
  --pattern REGEX    filename regex a file must match (default: \.txt$)
  --limit N          example hits printed per check per file (default 10,
                     0 = print all)
  --values           also print the distinct offending values and counts
  --keep-sentinel    do not skip INT_MAX "unset" values
  -h, --help         this message

exit status: 0 = clean, 1 = hits found, 2 = usage/no input error
EOF
}

path="top_gcd_snippets/costdump"
pattern='\.txt$'
limit=10
show_values=0
keep_sentinel=0
only=0
extra_checks=()
path_set=0

while [ $# -gt 0 ]; do
  case "$1" in
    --check)   [ $# -ge 2 ] || { echo "--check needs an argument" >&2; exit 2; }
               extra_checks+=("$2"); shift 2 ;;
    --check=*) extra_checks+=("${1#--check=}"); shift ;;
    --only)    only=1; shift ;;
    --pattern) pattern="$2"; shift 2 ;;
    --pattern=*) pattern="${1#--pattern=}"; shift ;;
    --limit)   limit="$2"; shift 2 ;;
    --limit=*) limit="${1#--limit=}"; shift ;;
    --values)  show_values=1; shift ;;
    --keep-sentinel) keep_sentinel=1; shift ;;
    -h|--help) usage; exit 0 ;;
    -*)        echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    *)         path="$1"; path_set=1; shift ;;
  esac
done
[ "$path_set" = 1 ] || :

checks=()
[ "$only" = 1 ] || checks=("${DEFAULT_CHECKS[@]}")
[ ${#extra_checks[@]} -eq 0 ] || checks+=("${extra_checks[@]}")
if [ ${#checks[@]} -eq 0 ]; then
  echo "no checks: --only was given with no --check" >&2
  exit 2
fi
case "$limit" in
  ''|*[!0-9]*) echo "--limit takes a non-negative integer" >&2; exit 2 ;;
esac

# ---------------------------------------------------------------------------
# collect input files
# ---------------------------------------------------------------------------
files=()
if [ -f "$path" ]; then
  files=("$path")
elif [ -d "$path" ]; then
  while IFS= read -r f; do
    files+=("$f")
  done < <(find "$path" -type f | grep -E "$pattern" | sort)
else
  echo "no such file or directory: $path" >&2
  exit 2
fi

if [ ${#files[@]} -eq 0 ]; then
  echo "no dump files matching /$pattern/ under $path" >&2
  exit 2
fi

# ---------------------------------------------------------------------------
# parse + check, one streaming pass per file
# ---------------------------------------------------------------------------
check_specs=$(printf '%s\n' "${checks[@]}")

awk -v limit="$limit" \
    -v show_values="$show_values" \
    -v keep_sentinel="$keep_sentinel" \
    -v int_max="$INT_MAX" \
    -v check_specs="$check_specs" \
    '
function isnum(v) { return v ~ /^-?[0-9]+$/ }

# pull "<name> <value>" pairs out of the current record for the listed keys
function kv(keys,   i, n, arr, j) {
  n = split(keys, arr, " ")
  for (i = 1; i < NF; i++)
    for (j = 1; j <= n; j++)
      if ($i == arr[j]) { f[arr[j]] = $(i + 1); break }
}

# pull "<group>[a:0 b:1 ...]" into f["<group>.a"], f["<group>.b"], ...
function group(g, s,   idx, rest, endp, body, k, arr, i, p) {
  idx = index(s, g "[")
  if (idx == 0) return
  rest = substr(s, idx + length(g) + 1)
  endp = index(rest, "]")
  if (endp == 0) return
  body = substr(rest, 1, endp - 1)
  k = split(body, arr, / +/)
  for (i = 1; i <= k; i++) {
    if (split(arr[i], p, ":") != 2) continue
    f[g "." p[1]] = p[2]
  }
}

function cmp(op, a, b) {
  if (isnum(a) && isnum(b)) { a += 0; b += 0 }
  else { a = "" a; b = "" b }
  if (op == "!=") return a != b
  if (op == "==") return a == b
  if (op == ">=") return a >= b
  if (op == "<=") return a <= b
  if (op == ">")  return a >  b
  if (op == "<")  return a <  b
  return 0
}

BEGIN {
  # ---- parse the check specs (one per line, passed in via -v) ----
  nspec = split(check_specs, spec_list, "\n")
  for (s = 1; s <= nspec; s++) {
    spec = spec_list[s]
    if (spec ~ /^[ \t]*$/) continue
    if (match(spec, /^[ \t]*[A-Za-z_]+[ \t]*:[ \t]*[A-Za-z_][A-Za-z0-9_.]*[ \t]*(!=|==|>=|<=|>|<)[ \t]*[^ \t]+[ \t]*$/) == 0) {
      printf("bad check %s -- expected e.g. next:v2v != 0\n", spec) > "/dev/stderr"
      bad = 1
      continue
    }
    nc++
    split(spec, part, /[ \t]*:[ \t]*/)
    ck_kind[nc] = part[1]
    gsub(/^[ \t]+/, "", ck_kind[nc])
    rest = part[2]
    match(rest, /(!=|==|>=|<=|>|<)/)
    ck_field[nc] = substr(rest, 1, RSTART - 1)
    ck_op[nc]    = substr(rest, RSTART, RLENGTH)
    ck_rhs[nc]   = substr(rest, RSTART + RLENGTH)
    gsub(/[ \t]/, "", ck_field[nc])
    gsub(/[ \t]/, "", ck_rhs[nc])
    ck_name[nc] = ck_kind[nc] ":" ck_field[nc] " " ck_op[nc] " " ck_rhs[nc]
  }
  if (nc == 0 && !bad) print "no usable checks" > "/dev/stderr"
  if (bad || nc == 0) { fatal = 1; exit 2 }
}

FNR == 1 {
  nf_files++
  file_order[nf_files] = FILENAME
  node = "-"; target = "-"
}

{
  delete f
  kind = ""

  if ($1 == "expanding") {
    kind = "expanding"
    f["x"] = $2; f["y"] = $3; f["z"] = $4
    f["ptX"] = $6; f["ptY"] = $7
    kv("cost pathCost lastDir")
    node = "(" $2 "," $3 "," $4 ")"
    target = "-"
  } else if ($1 == "est" && $2 == "coords") {
    kind = "est"
    f["nx"] = $3; f["ny"] = $4; f["nz"] = $5
    kv("dir manX manY manZ bendCnt forbidden finalEstCost")
    target = "(" $3 "," $4 "," $5 ")"
  } else if ($1 == "base") {
    kind = "base"
    kv("dir edgeLen totalBaseCost")
    group("flags", $0)
    group("costs", $0)
  } else if ($1 == "next") {
    kind = "next"
    kv("currPathCosts currDir nextDir edgeLength turnCost v2v vtlen finalNextCost vlenX vlenY currViaUp prevViaUp tLen tLenViaUp")
  }
  if (kind == "") next

  for (c = 1; c <= nc; c++) {
    if (ck_kind[c] != kind) continue
    if (!((ck_field[c]) in f)) continue
    val = f[ck_field[c]]
    if (!keep_sentinel && isnum(val) && val + 0 == int_max + 0) continue
    if (!cmp(ck_op[c], val, ck_rhs[c])) continue

    total++
    count[c, FILENAME]++
    if (show_values) {
      if (!((c SUBSEP val) in seen_val)) { seen_val[c, val] = 1; vals[c] = vals[c] " " val }
      vcount[c, val]++
    }
    n = count[c, FILENAME]
    if (limit == 0 || n <= limit) {
      line = $0
      sub(/^[ \t]+/, "", line)
      ex[c, FILENAME, n] = sprintf("    %s:%d  node %s -> %s  | %s",
                                   FILENAME, FNR, node, target, line)
    }
  }
}

END {
  if (fatal) exit 2
  printf("scanned %d file(s)\n", nf_files)
  for (c = 1; c <= nc; c++) {
    ctotal = 0
    for (i = 1; i <= nf_files; i++) ctotal += count[c, file_order[i]]
    printf("\n=== %s -- %d hit(s)\n", ck_name[c], ctotal)
    if (ctotal == 0) continue
    if (show_values) {
      nv = split(vals[c], vv, " ")
      out = ""
      for (i = 1; i <= nv; i++)
        out = out (i > 1 ? ", " : "") vv[i] " x" vcount[c, vv[i]]
      printf("    values: %s\n", out)
    }
    for (i = 1; i <= nf_files; i++) {
      fn = file_order[i]
      n = count[c, fn]
      if (n == 0) continue
      printf("  %s: %d\n", fn, n)
      shown = (limit == 0 || n < limit) ? n : limit
      for (k = 1; k <= shown; k++) print ex[c, fn, k]
      if (limit != 0 && n > limit)
        printf("    ... %d more (use --limit 0)\n", n - limit)
    }
  }
  printf("\ntotal: %d hit(s) across %d check(s)\n", total, nc)
  exit (total > 0 ? 1 : 0)
}
' "${files[@]}"
