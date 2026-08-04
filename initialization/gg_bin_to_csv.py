#!/usr/bin/env python3
"""Expand a dumpSearchGraph() .bin into a CSV, one row per grid-graph node.

The .bin is the compact snapshot of the node state on entry to one search()
call (see FlexGridGraph::dumpSearchGraph).  This unpacks it to text for
inspection and plotting.  It is lossless where gg_to_mem.py is not: the .mem
node word reduces every cost counter to a nonzero bit, whereas the CSV keeps
the magnitudes and the five NDR fields.

    python3 initialization/gg_bin_to_csv.py \
        work/ggdump/ggs_iter1_x136800_y136800_s1.bin \
        --gg  work/ggdump/gg_iter1_x136800_y136800.txt \
        --out work/ggdump/ggs_iter1_x136800_y136800_s1.csv

--gg is optional and only adds the xdbu/ydbu/layer columns, which the binary
does not carry; without it those are left empty.

A full grid is one row per node (243k rows / ~20 MB for a 174x233x6 box), which
loads fine in pandas but is a lot to scroll.  For a quick look, filter:

    --nonzero        drop nodes whose every state field is 0 and that carry no
                     edge -- usually cuts the row count by more than half
    --interesting    keep only nodes with something the router reacted to: a
                     blockage, any cost, a guide, or a src/dst flag
    --layer 2        one z layer, which is what you want for a 2-D heatmap

Column order matches the C++ Node declaration order, so a row lines up
field-by-field with the struct in FlexGridGraph.h.
"""

import argparse
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gg_to_mem import (  # noqa: E402  -- keeps one copy of the format constants
    COST_BITS,
    FLAG_BITS,
    NDR_COLS,
    SNAP_HEADER_BYTES,
    SNAP_MAGIC,
    SNAP_STRIDE,
)

# The binary packs the flags into two bytes: record byte 0 holds the first six
# FLAG_BITS entries (its bits 6,7 are the C++ unused1/unused2), byte 1 holds the
# remaining eight.
BYTE0_FLAGS = FLAG_BITS[:6]
BYTE1_FLAGS = FLAG_BITS[6:]
COST_ORDER = COST_BITS + NDR_COLS  # record bytes 2..13
VECTORS = ["guide", "src", "dst"]

COLUMNS = (["addr", "x", "y", "z", "xdbu", "ydbu", "layer"]
           + BYTE0_FLAGS + BYTE1_FLAGS + COST_ORDER + VECTORS)

# Fields that mean "the router will treat this node specially".  Edges and grid
# costs are excluded: nearly every node has them, so they say nothing.
INTERESTING = (["isBlockedEast", "isBlockedNorth", "isBlockedUp",
                "hasSpecialVia", "overrideShapeCostVia",
                "hasApCostEast", "hasApCostNorth", "hasApCostUp"]
               + COST_ORDER + VECTORS)


def die(msg):
    sys.exit("error: " + msg)


def read_gg_header(path):
    """Pull just the coordinate arrays out of a gg_*.txt dump."""
    want = {"xCoords", "yCoords", "zCoords(layerNum)", "dim"}
    out = {}
    with open(path) as f:
        for line in f:
            tok = line.split()
            if tok and tok[0] in want:
                out[tok[0]] = [int(v) for v in tok[1:]]
                if len(out) == len(want):
                    break
    missing = want - set(out)
    if missing:
        die("{} has no {} line".format(path, ", ".join(sorted(missing))))
    return out


def read_snapshot(path):
    with open(path, "rb") as f:
        blob = f.read()
    if len(blob) < SNAP_HEADER_BYTES or blob[:8] != SNAP_MAGIC:
        die("{} is not a {} search snapshot".format(path, SNAP_MAGIC.decode()))
    x_dim, y_dim, z_dim, cost_bits, stride, search_id = struct.unpack(
        "<6i", blob[8:SNAP_HEADER_BYTES])
    if stride != SNAP_STRIDE:
        die("{} has a {}-byte node record, expected {}".format(
            path, stride, SNAP_STRIDE))
    n = x_dim * y_dim * z_dim
    bm = (n + 7) // 8
    node_end = SNAP_HEADER_BYTES + n * stride
    want = node_end + 3 * bm
    if len(blob) != want:
        die("{} is {} bytes, expected {} for dim {}x{}x{}".format(
            path, len(blob), want, x_dim, y_dim, z_dim))
    meta = {"dim": (x_dim, y_dim, z_dim), "searchId": search_id,
            "costBits": cost_bits, "stride": stride, "n": n}
    bitmaps = [blob[node_end + i * bm:node_end + (i + 1) * bm] for i in range(3)]
    return blob, meta, bitmaps


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("snapshot", help="dumpSearchGraph() .bin")
    ap.add_argument("--gg", default=None,
                    help="gg_*.txt of the same route box, for the xdbu/ydbu/"
                         "layer columns")
    ap.add_argument("--out", default=None,
                    help="output path (default: the .bin with a .csv suffix)")
    ap.add_argument("--layer", type=int, default=None,
                    help="emit only this z")
    ap.add_argument("--nonzero", action="store_true",
                    help="drop nodes with no edge and no state at all")
    ap.add_argument("--interesting", action="store_true",
                    help="keep only blocked / costed / guided / src / dst nodes")
    args = ap.parse_args()

    if not os.path.isfile(args.snapshot):
        die("no such file: " + args.snapshot)
    out_path = args.out or os.path.splitext(args.snapshot)[0] + ".csv"

    blob, meta, (guides, srcs, dsts) = read_snapshot(args.snapshot)
    x_dim, y_dim, z_dim = meta["dim"]
    if args.layer is not None and not 0 <= args.layer < z_dim:
        die("--layer {} is outside 0..{}".format(args.layer, z_dim - 1))

    xc = yc = layers = None
    if args.gg:
        if not os.path.isfile(args.gg):
            die("no such file: " + args.gg)
        hdr = read_gg_header(args.gg)
        if tuple(hdr["dim"]) != meta["dim"]:
            die("{} dim {} does not match snapshot dim {} -- not the same "
                "route box".format(args.gg, tuple(hdr["dim"]), meta["dim"]))
        xc, yc, layers = hdr["xCoords"], hdr["yCoords"], hdr["zCoords(layerNum)"]

    n_flag0, n_cost = len(BYTE0_FLAGS), len(COST_ORDER)
    # Column index of each INTERESTING field within the row, resolved once.
    keep_idx = [COLUMNS.index(c) for c in INTERESTING]
    state_idx = keep_idx + [COLUMNS.index(c) for c in
                            ("hasEastEdge", "hasNorthEdge", "hasUpEdge")]

    written = 0
    with open(out_path, "w", newline="") as f:
        f.write(",".join(COLUMNS) + "\n")
        addr = 0
        for z in range(z_dim):
            skip = args.layer is not None and z != args.layer
            for y in range(y_dim):
                if skip:
                    addr += x_dim
                    continue
                chunk = []
                for x in range(x_dim):
                    off = SNAP_HEADER_BYTES + addr * SNAP_STRIDE
                    b0, b1 = blob[off], blob[off + 1]
                    byte, mask = addr >> 3, 1 << (addr & 7)
                    row = [addr, x, y, z,
                           xc[x] if xc else "",
                           yc[y] if yc else "",
                           layers[z] if layers else ""]
                    row += [(b0 >> i) & 1 for i in range(n_flag0)]
                    row += [(b1 >> i) & 1 for i in range(len(BYTE1_FLAGS))]
                    row += [blob[off + 2 + i] for i in range(n_cost)]
                    row += [1 if guides[byte] & mask else 0,
                            1 if srcs[byte] & mask else 0,
                            1 if dsts[byte] & mask else 0]
                    addr += 1

                    if args.interesting and not any(row[i] for i in keep_idx):
                        continue
                    if args.nonzero and not any(row[i] for i in state_idx):
                        continue
                    chunk.append(",".join(map(str, row)))
                if chunk:
                    f.write("\n".join(chunk) + "\n")
                    written += len(chunk)

    note = ""
    if meta["costBits"] != 8:
        note = "  (built with cost_bits={}, magnitudes saturated at 255)".format(
            meta["costBits"])
    print("{} -> {}".format(args.snapshot, out_path))
    print("  search {}, dim {}x{}x{}, {} of {} nodes written{}".format(
        meta["searchId"], x_dim, y_dim, z_dim, written, meta["n"], note))


if __name__ == "__main__":
    main()
