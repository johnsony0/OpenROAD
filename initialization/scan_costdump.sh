#!/usr/bin/env bash
#
# Scan FlexGridGraph cost dumps for cost terms the accelerator does not model yet.
#
# Reads every *.txt under a costdump directory, parses the four record kinds
# emitted by the maze-search instrumentation, and reports HOW MANY records trip
# each check, per file -- not the records themselves.  A dump is ~15k records
# per search, so printing the offending lines by default just floods the
# terminal; use --examples N when you actually want to eyeball them.
#
#   ./initialization/scan_costdump.sh top_gcd_snippets/costdump
#   ./initialization/scan_costdump.sh top_gcd_snippets/costdump --values
#   ./initialization/scan_costdump.sh top_gcd_snippets/costdump --examples 3
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
# Two dump layouts are in the wild and both are read here.  In version 3 the
# base/next records are tagged by keyword ("base dir ...", " next currPathCosts
# ...").  In version 2 both start with "coords <x> <y> <z>" instead; they are
# still told apart cheaply because the next record -- and only the next record
# -- is written with a leading space.  Version 2 also lacks the vlenX/vlenY/
# currViaUp/prevViaUp/tLen/tLenViaUp fields of the next record; checks against
# those simply never match on a v2 file (and are reported as unparsed).
#
# Speed: the scanner never lets awk split a record into fields (it dispatches on
# a leading substring and pulls only the handful of values some check actually
# asks for, via index/substr), rejects the common "field is already the value we
# are comparing against" case with a single substring search, and skips whole
# record kinds no check refers to.  Files are scanned in parallel -- see --jobs.
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
  --values           break the counts down by offending value
  --top N            values shown per breakdown before "+K more" (default 8,
                     0 = show all)
  --examples N       also print N offending records per check per file
                     (default 0 = counts only; 'all' = every hit)
  --keep-sentinel    do not skip INT_MAX "unset" values
  --jobs N           files to scan concurrently (default: min(cpus, files, 8))
  --no-progress      do not print the "[i/N] file" progress tracker
  -h, --help         this message

exit status: 0 = clean, 1 = hits found, 2 = usage/no input error,
             3 = no hits, but some file yielded no record of a checked kind
                 (parse gap -- "clean" cannot be trusted for those)
EOF
}

path="top_gcd_snippets/costdump"
pattern='\.txt$'
examples=0
top=8
show_values=0
keep_sentinel=0
only=0
extra_checks=()
progress=1
jobs=""

while [ $# -gt 0 ]; do
  case "$1" in
    --check)   [ $# -ge 2 ] || { echo "--check needs an argument" >&2; exit 2; }
               extra_checks+=("$2"); shift 2 ;;
    --check=*) extra_checks+=("${1#--check=}"); shift ;;
    --only)    only=1; shift ;;
    --pattern) pattern="$2"; shift 2 ;;
    --pattern=*) pattern="${1#--pattern=}"; shift ;;
    --examples) examples="$2"; shift 2 ;;
    --examples=*) examples="${1#--examples=}"; shift ;;
    --limit)   echo "note: --limit is now --examples (0 = none, the new default)" >&2
               examples="$2"; shift 2 ;;
    --limit=*) echo "note: --limit is now --examples (0 = none, the new default)" >&2
               examples="${1#--limit=}"; shift ;;
    --top)     top="$2"; shift 2 ;;
    --top=*)   top="${1#--top=}"; shift ;;
    --values)  show_values=1; shift ;;
    --keep-sentinel) keep_sentinel=1; shift ;;
    --jobs)    jobs="$2"; shift 2 ;;
    --jobs=*)  jobs="${1#--jobs=}"; shift ;;
    --no-progress) progress=0; shift ;;
    -h|--help) usage; exit 0 ;;
    -*)        echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    *)         path="$1"; shift ;;
  esac
done

[ "$examples" = "all" ] && examples=-1
case "$examples" in
  -1|''|*[!0-9]*) [ "$examples" = -1 ] || { echo "--examples takes a non-negative integer or 'all'" >&2; exit 2; } ;;
esac
case "$top" in
  ''|*[!0-9]*) echo "--top takes a non-negative integer" >&2; exit 2 ;;
esac
if [ -n "$jobs" ]; then
  case "$jobs" in
    ''|*[!0-9]*|0) echo "--jobs takes a positive integer" >&2; exit 2 ;;
  esac
fi

checks=()
[ "$only" = 1 ] || checks=("${DEFAULT_CHECKS[@]}")
[ ${#extra_checks[@]} -eq 0 ] || checks+=("${extra_checks[@]}")
if [ ${#checks[@]} -eq 0 ]; then
  echo "no checks: --only was given with no --check" >&2
  exit 2
fi

# mawk is several times faster than gawk on this workload and the program sticks
# to POSIX awk, so take it when the image has it.
AWK=awk
for a in mawk gawk awk; do
  if command -v "$a" >/dev/null 2>&1; then AWK=$a; break; fi
done

# ---------------------------------------------------------------------------
# collect input files
# ---------------------------------------------------------------------------
files=()
strip=""
if [ -f "$path" ]; then
  files=("$path")
elif [ -d "$path" ]; then
  strip="${path%/}/"
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

tmpd=$(mktemp -d "${TMPDIR:-/tmp}/scan_costdump.XXXXXX") || exit 2
trap 'rm -rf "$tmpd"' EXIT INT TERM

# ---------------------------------------------------------------------------
# normalise + validate the checks once, into "kind<TAB>field<TAB>op<TAB>rhs"
# ---------------------------------------------------------------------------
cat >"$tmpd/parse.awk" <<'PARSE_AWK'
{
  spec = $0
  if (spec ~ /^[ \t]*$/) next
  if (match(spec, /^[ \t]*[A-Za-z_]+[ \t]*:[ \t]*[A-Za-z_][A-Za-z0-9_.]*[ \t]*(!=|==|>=|<=|>|<)[ \t]*[^ \t]+[ \t]*$/) == 0) {
    printf("bad check: %s -- expected e.g. next:v2v != 0\n", spec) > "/dev/stderr"
    bad = 1
    next
  }
  ci = index(spec, ":")
  kind = substr(spec, 1, ci - 1)
  rest = substr(spec, ci + 1)
  match(rest, /(!=|==|>=|<=|>|<)/)
  field = substr(rest, 1, RSTART - 1)
  op    = substr(rest, RSTART, RLENGTH)
  rhs   = substr(rest, RSTART + RLENGTH)
  gsub(/[ \t]/, "", kind); gsub(/[ \t]/, "", field); gsub(/[ \t]/, "", rhs)
  if (kind != "expanding" && kind != "est" && kind != "base" && kind != "next") {
    printf("bad check: %s -- unknown record kind %s (want expanding|est|base|next)\n",
           spec, kind) > "/dev/stderr"
    bad = 1
    next
  }
  key = kind ":" field op rhs
  if (key in seen) next            # a repeated check would double-count
  seen[key] = 1
  printf("%s\t%s\t%s\t%s\n", kind, field, op, rhs)
  n++
}
END {
  if (!bad && n == 0) print "no usable checks" > "/dev/stderr"
  if (bad || n == 0) exit 2
}
PARSE_AWK

specs=$(printf '%s\n' "${checks[@]}" | "$AWK" -f "$tmpd/parse.awk") || exit 2

# ---------------------------------------------------------------------------
# stage 1: scan.  One streaming pass per file, emitting tab-separated tallies.
#
#   F <file> <version>              file was read
#   R <file> <kindid> <n>           n records of that kind
#   S <file> <check> 1              the checked field existed on some record
#   H <file> <check> <n>            n records tripped the check
#   V <file> <check> <value> <n>    value breakdown (--values)
#   X <file> <check> <i> <fnr> <node> <target> <line>   example (--examples)
# ---------------------------------------------------------------------------
cat >"$tmpd/scan.awk" <<'SCAN_AWK'
# ---- value extraction -----------------------------------------------------
# Records are "<name> <value>" pairs, so the value of NAME is whatever follows
# the first " NAME " in the raw record.  Doing it with index/substr rather than
# $1..$NF keeps awk from ever splitting the record into fields, which is the
# single biggest cost when the record has 30 of them and we want one.
function getf(name,   p, s, q) {
  p = index($0, " " name " ")
  if (p == 0) return NOTF
  s = substr($0, p + length(name) + 2)
  q = index(s, " ")
  return q ? substr(s, 1, q - 1) : s
}

# "<group>[a:0 b:1 ...]" -> the value of key <k> inside that group only, so
# flags.drc and costs.drc do not shadow each other.
function getg(g, k,   p, s, e, b, q) {
  p = index($0, g "[")
  if (p == 0) return NOTF
  s = substr($0, p + length(g) + 1)
  e = index(s, "]")
  if (e == 0) return NOTF
  b = " " substr(s, 1, e - 1)
  q = index(b, " " k ":")
  if (q == 0) return NOTF
  s = substr(b, q + length(k) + 2)
  e = index(s, " ")
  return e ? substr(s, 1, e - 1) : s
}

function isnum(v) { return v ~ /^-?[0-9]+$/ }

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

function flushfile(   c, kk, i, nv, parts) {
  if (curfile == "") return
  printf("F\t%s\t%s\n", curfile, ver)
  for (kk = 1; kk <= 4; kk++)
    if (rec[kk]) printf("R\t%s\t%d\t%d\n", curfile, kk, rec[kk])
  for (c = 1; c <= nc; c++) {
    if (sawf[c]) printf("S\t%s\t%d\t1\n", curfile, c)
    if (hits[c]) printf("H\t%s\t%d\t%d\n", curfile, c, hits[c])
    if (show_values) {
      nv = split(vlist[c], parts, SUBSEP)
      for (i = 1; i <= nv; i++)
        if (parts[i] != "")
          printf("V\t%s\t%d\t%s\t%d\n", curfile, c, parts[i], vcount[c, parts[i]])
    }
    for (i = 1; i <= exn[c]; i++)
      printf("X\t%s\t%d\t%d\t%s\n", curfile, c, i, ex[c, i])
  }
  delete rec; delete sawf; delete hits; delete vlist
  delete vcount; delete vseen; delete ex; delete exn
}

BEGIN {
  NOTF = "\001"                     # cannot occur in a dump token
  int_max_s = "" int_max

  kid["expanding"] = 1; kid["est"] = 2; kid["base"] = 3; kid["next"] = 4

  # the two record kinds with positional (not "name value") fields
  pos["1:x"] = 2; pos["1:y"] = 3; pos["1:z"] = 4; pos["1:ptX"] = 6; pos["1:ptY"] = 7
  pos["2:nx"] = 3; pos["2:ny"] = 4; pos["2:nz"] = 5

  nc = split(specs, sl, "\n")
  for (c = 1; c <= nc; c++) {
    split(sl[c], p, "\t")
    k = kid[p[1]]
    ck_kid[c] = k; ck_field[c] = p[2]; ck_op[c] = p[3]; ck_rhs[c] = p[4]

    d = index(p[2], ".")
    if (d) { ck_grp[c] = substr(p[2], 1, d - 1); ck_key[c] = substr(p[2], d + 1) }
    else   { ck_grp[c] = ""; ck_key[c] = "" }

    ck_pos[c] = ((k ":" p[2]) in pos) ? pos[k ":" p[2]] : 0
    if (ck_pos[c]) need_split[k] = 1

    # Fast reject: if the record literally reads "<field> <rhs> " then the value
    # IS the right-hand side, and the outcome is whatever "rhs OP rhs" is -- one
    # substring search settles the overwhelmingly common "!= 0" case with no
    # extraction at all.  Not usable for grouped fields (the same key lives in
    # both flags[] and costs[]) nor when rhs is the skipped sentinel.
    if (ck_grp[c] == "" && ck_pos[c] == 0 && (keep_sentinel || p[4] != int_max_s)) {
      ck_fast[c] = " " p[2] " " p[4] " "
      ck_self[c] = cmp(p[3], p[4], p[4])
    } else ck_fast[c] = ""

    kcn[k]++
    kc[k * 1000 + kcn[k]] = c
  }
  if (examples != 0) { need_split[1] = 1; need_split[2] = 1 }
}

FNR == 1 {
  flushfile()
  curfile = FILENAME
  ver = "?"
  node = "-"; target = "-"
  nseen++
  if (progress) {
    if (progress_tty)
      printf("\r[%d/%d] %s\033[K", nseen, total_files, FILENAME) > "/dev/stderr"
    else if (total_files)
      printf("[%d/%d] %s\n", nseen, total_files, FILENAME) > "/dev/stderr"
    else
      printf("  scanning %s\n", FILENAME) > "/dev/stderr"
    fflush("/dev/stderr")
  }
}

{
  # Kind dispatch on a leading substring: no field reference, so no field split.
  # v3 tags the records ("est ", "expanding ", "base ", " next "); v2 writes
  # "coords " for base and " coords " for next -- the leading space is the tell.
  c1 = substr($0, 1, 1)
  if (c1 == " ")      k = 4
  else if (c1 == "e") k = (substr($0, 1, 4) == "est ") ? 2 : ((substr($0, 1, 10) == "expanding ") ? 1 : 0)
  else if (c1 == "b") k = (substr($0, 1, 5) == "base ") ? 3 : 0
  else if (c1 == "c") k = (substr($0, 1, 7) == "coords ") ? 3 : 0
  else {
    if (substr($0, 1, 8) == "version ") ver = substr($0, 9)
    next
  }
  if (k == 0) next
  rec[k]++

  if (need_split[k]) {
    split($0, T, " ")
    if (k == 1)      { node = "(" T[2] "," T[3] "," T[4] ")"; target = "-" }
    else if (k == 2) { target = "(" T[3] "," T[4] "," T[5] ")" }
  }

  n = kcn[k]
  if (n == 0) next

  for (i = 1; i <= n; i++) {
    c = kc[k * 1000 + i]

    if (ck_fast[c] != "" && index($0, ck_fast[c])) {
      sawf[c] = 1
      if (!ck_self[c]) continue
      val = ck_rhs[c]
    } else {
      if (ck_pos[c])           val = (T[ck_pos[c]] == "") ? NOTF : T[ck_pos[c]]
      else if (ck_grp[c] != "") val = getg(ck_grp[c], ck_key[c])
      else                      val = getf(ck_field[c])
      if (val == NOTF) continue
      sawf[c] = 1
      if (!keep_sentinel && val == int_max_s) continue
      if (!cmp(ck_op[c], val, ck_rhs[c])) continue
    }

    hits[c]++
    if (show_values) {
      if (!((c SUBSEP val) in vseen)) { vseen[c, val] = 1; vlist[c] = vlist[c] SUBSEP val }
      vcount[c, val]++
    }
    if (examples != 0 && (examples < 0 || exn[c] < examples)) {
      line = $0
      sub(/^[ \t]+/, "", line)
      exn[c]++
      ex[c, exn[c]] = FNR "\t" node "\t" target "\t" line
    }
  }
}

END {
  flushfile()
  if (progress && progress_tty) { printf("\r\033[K") > "/dev/stderr"; fflush("/dev/stderr") }
}
SCAN_AWK

# ---------------------------------------------------------------------------
# stage 2: report.  Reads the ordered file list, then every tally part file.
# ---------------------------------------------------------------------------
cat >"$tmpd/report.awk" <<'REPORT_AWK'
function disp(f) {
  return (striplen && substr(f, 1, striplen) == strip) ? substr(f, striplen + 1) : f
}

# top-N of a check/file value breakdown, biggest count first
function valstr(vl, cnt_c, cnt_f,   nv, parts, i, j, best, bi, used, out, shown, rest) {
  nv = split(vl, parts, SUBSEP)
  if (nv == 0) return ""
  out = ""; shown = 0
  for (j = 1; j <= nv; j++) {
    best = -1; bi = 0
    for (i = 1; i <= nv; i++) {
      if (parts[i] == "" || (i in used)) continue
      if (vcnt[cnt_c, cnt_f, parts[i]] > best) { best = vcnt[cnt_c, cnt_f, parts[i]]; bi = i }
    }
    if (bi == 0) break
    used[bi] = 1
    if (top && shown >= top) { rest++; continue }
    out = out (shown ? ", " : "") parts[bi] " x" best
    shown++
  }
  if (rest) out = out ", +" rest " more value(s)"
  return out
}

BEGIN {
  FS = "\t"
  kname[1] = "expanding"; kname[2] = "est"; kname[3] = "base"; kname[4] = "next"
  kid["expanding"] = 1; kid["est"] = 2; kid["base"] = 3; kid["next"] = 4
  striplen = length(strip)

  nc = split(specs, sl, "\n")
  for (c = 1; c <= nc; c++) {
    split(sl[c], p, "\t")
    ck_kid[c] = kid[p[1]]
    ck_kind[c] = p[1]; ck_field[c] = p[2]
    ck_name[c] = p[1] ":" p[2] " " p[3] " " p[4]
  }
}

NR == FNR { order[++nf] = $0; w = length(disp($0)); if (w > namew) namew = w; next }

$1 == "F" { present[$2] = 1; ver[$2] = $3; next }
$1 == "R" { rec[$2, $3 + 0] = $4; next }
$1 == "S" { sawf[$2, $3 + 0] = 1; next }
$1 == "H" { hit[$2, $3 + 0] = $4; ctot[$3 + 0] += $4; total += $4; next }
$1 == "V" {
  c = $3 + 0
  vcnt[c, $2, $4] = $5
  if (!((c SUBSEP $2 SUBSEP $4) in vs)) { vs[c, $2, $4] = 1; vl[c, $2] = vl[c, $2] SUBSEP $4 }
  vcnt[c, "", $4] += $5
  if (!((c SUBSEP "" SUBSEP $4) in vs)) { vs[c, "", $4] = 1; vl[c, ""] = vl[c, ""] SUBSEP $4 }
  next
}
$1 == "X" { c = $3 + 0; exn[c, $2] = $4; ex[c, $2, $4 + 0] = $5 "\t" $6 "\t" $7 "\t" $8; next }

END {
  nempty = 0
  for (i = 1; i <= nf; i++) if (!(order[i] in present)) nempty++
  if (nempty == 0) printf("scanned %d file(s)\n", nf)
  else printf("scanned %d of %d file(s) (%d empty)\n", nf - nempty, nf, nempty)

  if (namew < 4) namew = 4

  for (c = 1; c <= nc; c++) {
    kk = ck_kid[c]
    kindtot = 0
    for (i = 1; i <= nf; i++) kindtot += rec[order[i], kk]
    printf("\n=== %s -- %d hit(s) of %d %s record(s)\n",
           ck_name[c], ctot[c], kindtot, ck_kind[c])

    # A check can only fire on records it actually parsed.  If a file yielded no
    # record of this kind at all, "0 hits" means "not scanned", not "clean" --
    # say so loudly, since that is exactly how a dump format change hides.
    nunparsed = 0; unparsed = ""
    for (i = 1; i <= nf; i++) {
      fn = order[i]
      if ((fn SUBSEP c) in sawf) continue
      why = rec[fn, kk] ? sprintf("no %s field on its %s records", ck_field[c], ck_kind[c]) \
                        : sprintf("no %s records at all", ck_kind[c])
      nunparsed++
      unparsed = unparsed sprintf("\n      %s (version %s): %s",
                                  disp(fn), (fn in ver) ? ver[fn] : "?", why)
    }
    if (nunparsed) {
      printf("  !! not evaluated on %d file(s) -- 0 hits there is NOT proof of clean:%s\n",
             nunparsed, unparsed)
      unparsed_total++
    }
    if (ctot[c] == 0) continue

    dw = length("of " ck_kind[c])
    if (dw < 8) dw = 8
    printf("  %-*s  %8s  %*s  %7s\n", namew, "file", "hits", dw, "of " ck_kind[c], "rate")
    for (i = 1; i <= nf; i++) {
      fn = order[i]
      n = hit[fn, c]
      if (n == 0) continue
      d = rec[fn, kk]
      printf("  %-*s  %8d  %*d  %6.2f%%\n", namew, disp(fn), n, dw, d, d ? 100 * n / d : 0)
      if (show_values) printf("  %-*s  %s\n", namew, "", valstr(vl[c, fn], c, fn))
      for (j = 1; j <= exn[c, fn]; j++) {
        split(ex[c, fn, j], e, "\t")
        printf("      %s:%s  node %s -> %s  | %s\n", disp(fn), e[1], e[2], e[3], e[4])
      }
      if (examples > 0 && n > exn[c, fn])
        printf("      ... %d more (use --examples all)\n", n - exn[c, fn])
    }
    if (show_values && nf > 1)
      printf("  %-*s  %s\n", namew, "all files", valstr(vl[c, ""], c, ""))
  }

  printf("\ntotal: %d hit(s) across %d check(s)\n", total, nc)
  if (unparsed_total)
    printf("warning: %d check(s) could not be evaluated on every file (see !! above)\n",
           unparsed_total)
  if (total > 0) exit 1
  exit (unparsed_total > 0 ? 3 : 0)
}
REPORT_AWK

# Reject bad checks once, before spawning workers, so the diagnostic is printed
# a single time instead of once per job.
"$AWK" -f "$tmpd/scan.awk" \
    -v specs="$specs" -v int_max="$INT_MAX" -v keep_sentinel="$keep_sentinel" \
    -v show_values="$show_values" -v examples="$examples" \
    -v progress=0 -v progress_tty=0 -v total_files=0 </dev/null >/dev/null || exit 2

# ---------------------------------------------------------------------------
# run the scan, one worker per chunk of files
# ---------------------------------------------------------------------------
nfiles=${#files[@]}
if [ -z "$jobs" ]; then
  cpus=$(nproc 2>/dev/null || getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)
  jobs=$cpus
  [ "$jobs" -gt 8 ] && jobs=8
fi
[ "$jobs" -gt "$nfiles" ] && jobs=$nfiles

progress_tty=0
if [ "$progress" = 1 ] && [ "$jobs" = 1 ] && [ -t 2 ]; then progress_tty=1; fi
# with several workers the [i/N] counter is per-worker and meaningless
tf=$nfiles
[ "$jobs" = 1 ] || tf=0

printf '%s\n' "${files[@]}" >"$tmpd/files.txt"

# round-robin so a slow file does not leave one worker holding the whole tail
for i in "${!files[@]}"; do
  printf '%s\n' "${files[i]}" >>"$tmpd/chunk.$((i % jobs))"
done

for j in $(seq 0 $((jobs - 1))); do
  [ -s "$tmpd/chunk.$j" ] || continue
  (
    chunk=()
    while IFS= read -r f; do chunk+=("$f"); done <"$tmpd/chunk.$j"
    "$AWK" -f "$tmpd/scan.awk" \
        -v specs="$specs" -v int_max="$INT_MAX" -v keep_sentinel="$keep_sentinel" \
        -v show_values="$show_values" -v examples="$examples" \
        -v progress="$progress" -v progress_tty="$progress_tty" -v total_files="$tf" \
        "${chunk[@]}" >"$tmpd/part.$j"
    echo $? >"$tmpd/rc.$j"
  ) &
done
wait

for j in $(seq 0 $((jobs - 1))); do
  [ -s "$tmpd/chunk.$j" ] || continue
  rc=$(cat "$tmpd/rc.$j" 2>/dev/null || echo 2)
  if [ "$rc" != 0 ]; then
    echo "scan worker $j failed (exit $rc)" >&2
    exit 2
  fi
done

"$AWK" -f "$tmpd/report.awk" \
    -v specs="$specs" -v strip="$strip" -v show_values="$show_values" \
    -v examples="$examples" -v top="$top" \
    "$tmpd/files.txt" "$tmpd"/part.*
