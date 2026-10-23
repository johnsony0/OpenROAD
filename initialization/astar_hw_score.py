#!/usr/bin/env python3
"""Score an A* hardware trial run with ISPD18 weights and single-thread speedup.

    python3 initialization/astar_hw_score.py <DRT_ASTAR_HW_OUT dir> [-o out.csv]

Reads iters.csv / nets.csv / searches.csv written by the router
(src/drt/src/dr/AstarHwTrial.h) and writes one row per DR iteration plus a
TOTAL row.

Quality.  Every category of the ISPD 2018 detailed-routing score is measured
on three populations:
  design_*  the committed (CPU) routing of the whole design after the iteration
  cpu_*     the CPU route of every trialed net, at the time it was routed
  hw_*      the accelerator route of the same nets from the same router state
Lengths are converted to M2 pitches and short area to M2-pitch^2 as the
contest does; weights are in WEIGHTS below (override with --weight k=v).
The geometry checks are our own implementation of the evaluator's categories
(see AstarHwTrial.cpp: off-track = centerline/origin not on a track of the
layer, out-of-guide = not covered by the net's original guides), so check them
against ispd18eval on an ISPD18 testcase before quoting absolute scores.

The hardware's effect on the design score is estimated as
  est_hw_design_score = design_score + (hw_net_score - cpu_net_score)
because the trial never commits hardware routes.

Open-list overflow.  A search the accelerator gives up on because a priority
queue is completely full has hw status pq_full; its net is scored as the CPU
route (the router sets hw_* = cpu_* for that net).  searches_pq_full,
nets_pq_full and nets_hw_ok (every search of the net on the accelerator) count
them, with searches_timeout for searches that ran out of cycles.

Speed.  trial_cpu_ms and trial_hw_ms are the CPU A* and the accelerator
(cycles / clk) on identical search inputs, so astar_speedup is a single-thread
like-for-like number.  Both cover only the searches whose hardware path was
used: a pq_full, timeout or invalid-path search adds to neither, so the
speedup is over the searches the accelerator completed (see the counts above
for how many did not).  The iteration projection replaces the CPU route's A*
time with the accelerator time at that speedup:
  hw_iter_ms = cpu_iter_ms - cpu_route_astar_ms + cpu_route_astar_ms / astar_speedup
cpu_iter_ms and cpu_route_astar_ms come from --baseline (a CPU-only run with
DRT_ASTAR_HW_MODE=record, which commits identical routing) when given; without
it they fall back to the trial run's iter_wall_ms - trial_ms, which still
carries trial overhead (gg.txt writes, state copies).  Only meaningful for a
1-thread run; multi-thread numbers are extrapolated from these.
"""

import argparse
import csv
import sys
from pathlib import Path

# ISPD 2018 contest weights (Mantik et al., "ISPD 2018 Initial Detailed
# Routing Contest and Benchmarks"). Verify against your ispd18eval build.
WEIGHTS = {
    "wl": 0.5,             # per M2 pitch of wire
    "vias": 2.0,           # per via
    "offguide_wire": 1.0,  # per M2 pitch
    "offguide_vias": 1.0,
    "offtrack_wire": 0.5,  # per M2 pitch
    "offtrack_vias": 1.0,
    "wrongway": 1.0,       # per M2 pitch
    "short_area": 500.0,   # per M2 pitch^2
    "minarea": 500.0,      # per violation
    "spacing": 500.0,      # per violation
}

# Outcome counters from iters.csv (absent in runs before they were added).
FAIL_COUNTS = ["nets_hw_ok", "nets_pq_full", "searches_pq_full",
               "searches_timeout"]

QUALITY = ["wl_dbu", "vias", "offguide_wire_dbu", "offguide_vias",
           "offtrack_wire_dbu", "offtrack_vias", "wrongway_dbu", "patches",
           "drc", "shorts", "short_area", "minarea", "spacing", "other_drc"]


def score(row, prefix, pitch):
    def g(name):
        return float(row[prefix + name])
    return (WEIGHTS["wl"] * g("wl_dbu") / pitch
            + WEIGHTS["vias"] * g("vias")
            + WEIGHTS["offguide_wire"] * g("offguide_wire_dbu") / pitch
            + WEIGHTS["offguide_vias"] * g("offguide_vias")
            + WEIGHTS["offtrack_wire"] * g("offtrack_wire_dbu") / pitch
            + WEIGHTS["offtrack_vias"] * g("offtrack_vias")
            + WEIGHTS["wrongway"] * g("wrongway_dbu") / pitch
            + WEIGHTS["short_area"] * g("short_area") / (pitch * pitch)
            + WEIGHTS["minarea"] * g("minarea")
            + WEIGHTS["spacing"] * g("spacing"))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("hw_out", type=Path)
    ap.add_argument("-o", "--out", type=Path, default=None,
                    help="output CSV (default <hw_out>/score.csv)")
    ap.add_argument("--run-id", default=None,
                    help="score only this run_id (default: the last one)")
    ap.add_argument("--baseline", type=Path, default=None,
                    help="DRT_ASTAR_HW_OUT of a CPU-only record run of the "
                    "same inputs; supplies the iteration timing")
    ap.add_argument("--weight", action="append", default=[],
                    help="override a weight, e.g. --weight vias=4")
    args = ap.parse_args()
    for kv in args.weight:
        k, v = kv.split("=", 1)
        if k not in WEIGHTS:
            sys.exit("unknown weight {!r}; one of {}".format(k, ", ".join(WEIGHTS)))
        WEIGHTS[k] = float(v)

    with open(args.hw_out / "iters.csv") as f:
        iters = list(csv.DictReader(f))
    if not iters:
        sys.exit("no rows in iters.csv")
    run_id = args.run_id or iters[-1]["run_id"]
    iters = [r for r in iters if r["run_id"] == run_id]

    base = {}
    if args.baseline is not None:
        with open(args.baseline / "iters.csv") as f:
            rows = list(csv.DictReader(f))
        base_id = rows[-1]["run_id"]
        base = {r["iter"]: r for r in rows if r["run_id"] == base_id}

    out_rows = []
    total = {"cpu_net": 0.0, "hw_net": 0.0, "trial_cpu_ms": 0.0,
             "trial_hw_ms": 0.0, "cpu_iter_ms": 0.0, "hw_iter_ms": 0.0,
             "cpu_route_astar_ms": 0.0, "nets_trialed": 0,
             "searches": 0, "hw_used": 0, "path_match": 0, "cost_match": 0}
    for prefix in ("cpu_", "hw_"):
        for q in QUALITY:
            total[prefix + q] = 0.0
    sources = set()
    for r in iters:
        pitch = float(r["m2_pitch_dbu"]) or 1.0
        design_score = score(r, "design_", pitch)
        cpu_net = score(r, "cpu_", pitch)
        hw_net = score(r, "hw_", pitch)
        cpu_ms = float(r["trial_cpu_ms"])
        hw_ms = float(r["trial_hw_ms"])
        speedup = cpu_ms / hw_ms if hw_ms > 0 else float("nan")
        b = base.get(r["iter"])
        if b is not None:
            cpu_iter = float(b["iter_wall_ms"])
            astar = float(b["cpu_route_astar_ms"])
        else:
            cpu_iter = float(r["iter_wall_ms"]) - float(r["trial_ms"])
            astar = float(r["cpu_route_astar_ms"])
        hw_iter = (cpu_iter - astar + astar / speedup) if hw_ms > 0 else float("nan")
        searches = int(r["searches"])
        row = {
            "run_id": run_id, "iter": r["iter"], "threads": r["threads"],
            "nets_trialed": r["nets_trialed"], "searches": searches,
            "hw_used": r["hw_used"], "path_match": r["path_match"],
            "cost_match": r["cost_match"],
            **{k: int(r.get(k, 0)) for k in FAIL_COUNTS},
            "path_match_rate": int(r["path_match"]) / searches if searches else 0,
            "design_score": round(design_score, 2),
            "cpu_net_score": round(cpu_net, 2),
            "hw_net_score": round(hw_net, 2),
            "hw_minus_cpu_score": round(hw_net - cpu_net, 2),
            "est_hw_design_score": round(design_score + hw_net - cpu_net, 2),
            "design_drc": r["design_drc"],
            "cpu_net_drc": r["cpu_drc"], "hw_net_drc": r["hw_drc"],
            "trial_cpu_ms": round(cpu_ms, 3), "trial_hw_ms": round(hw_ms, 3),
            "astar_speedup": round(speedup, 3),
            "timing_source": "baseline" if b is not None else "trial",
            "cpu_route_astar_ms": round(astar, 3),
            "astar_fraction": round(astar / cpu_iter, 4) if cpu_iter > 0 else 0,
            "cpu_iter_ms": round(cpu_iter, 3), "hw_iter_ms": round(hw_iter, 3),
            "iter_speedup": round(cpu_iter / hw_iter, 3) if hw_ms > 0 else float("nan"),
        }
        for prefix in ("design_", "cpu_", "hw_"):
            for q in QUALITY:
                row[prefix + q] = r[prefix + q]
        out_rows.append(row)
        total["cpu_net"] += cpu_net
        total["hw_net"] += hw_net
        total["trial_cpu_ms"] += cpu_ms
        total["trial_hw_ms"] += hw_ms
        total["cpu_iter_ms"] += cpu_iter
        total["cpu_route_astar_ms"] += astar
        if hw_ms > 0:
            total["hw_iter_ms"] += hw_iter
        else:
            total["hw_iter_ms"] += cpu_iter
        for k in ("nets_trialed", "searches", "hw_used", "path_match",
                  "cost_match"):
            total[k] += int(r[k])
        for k in FAIL_COUNTS:
            total[k] = total.get(k, 0) + int(r.get(k, 0))
        for prefix in ("cpu_", "hw_"):
            for q in QUALITY:
                total[prefix + q] += float(r[prefix + q])
        sources.add(row["timing_source"])

    last = out_rows[-1]
    t_speed = (total["trial_cpu_ms"] / total["trial_hw_ms"]
               if total["trial_hw_ms"] > 0 else float("nan"))
    total_row = {
        "run_id": run_id, "iter": "TOTAL", "threads": last["threads"],
        "nets_trialed": total["nets_trialed"],
        "searches": total["searches"], "hw_used": total["hw_used"],
        "path_match": total["path_match"], "cost_match": total["cost_match"],
        **{k: total.get(k, 0) for k in FAIL_COUNTS},
        "path_match_rate": (total["path_match"] / total["searches"]
                            if total["searches"] else 0),
        # The final design score is the last iteration's committed routing.
        "design_score": last["design_score"],
        "cpu_net_score": round(total["cpu_net"], 2),
        "hw_net_score": round(total["hw_net"], 2),
        "hw_minus_cpu_score": round(total["hw_net"] - total["cpu_net"], 2),
        "est_hw_design_score": round(last["design_score"]
                                     + total["hw_net"] - total["cpu_net"], 2),
        "design_drc": last["design_drc"],
        "cpu_net_drc": int(total["cpu_drc"]),
        "hw_net_drc": int(total["hw_drc"]),
        "trial_cpu_ms": round(total["trial_cpu_ms"], 3),
        "trial_hw_ms": round(total["trial_hw_ms"], 3),
        "astar_speedup": round(t_speed, 3),
        "timing_source": sources.pop() if len(sources) == 1 else "mixed",
        "cpu_route_astar_ms": round(total["cpu_route_astar_ms"], 3),
        "astar_fraction": (round(total["cpu_route_astar_ms"]
                                 / total["cpu_iter_ms"], 4)
                           if total["cpu_iter_ms"] > 0 else 0),
        "cpu_iter_ms": round(total["cpu_iter_ms"], 3),
        "hw_iter_ms": round(total["hw_iter_ms"], 3),
        "iter_speedup": (round(total["cpu_iter_ms"] / total["hw_iter_ms"], 3)
                         if total["hw_iter_ms"] > 0 else float("nan")),
    }
    # design_* is the committed routing, so the last iteration's state is the
    # final one; cpu_* / hw_* are per-iteration trial populations and add up.
    for q in QUALITY:
        total_row["design_" + q] = last["design_" + q]
        for prefix in ("cpu_", "hw_"):
            v = total[prefix + q]
            total_row[prefix + q] = int(v) if v.is_integer() else round(v, 3)
    out_rows.append(total_row)

    out = args.out or args.hw_out / "score.csv"
    fields = list(out_rows[0].keys())
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(out_rows)
    cols = ["iter", "searches", "nets_hw_ok", "nets_pq_full",
            "path_match_rate", "design_score",
            "cpu_net_score", "hw_net_score", "est_hw_design_score",
            "astar_speedup", "astar_fraction", "iter_speedup"]
    print("  ".join("{:>14}".format(c) for c in cols))
    for r in out_rows:
        print("  ".join("{:>14}".format(
            "{:.3f}".format(r[c]) if isinstance(r.get(c), float) else str(r.get(c, "")))
            for c in cols))
    print("wrote", out)


if __name__ == "__main__":
    main()
