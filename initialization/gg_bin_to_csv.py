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

         python3 initialization/gg_bin_to_csv.py \
        top_gcd/ggdump/ggs_iter1_x50400_y50400_s3.bin \
        --gg  top_gcd/ggdump/gg_iter1_x50400_y50400.txt \
        --out work/ggs_iter1_x50400_y50400_s3.csv

--gg is required: the binary stores only per-search changes, and shared static
node fields and coordinate columns are reconstructed from the matching dump.

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
    SNAP_COST_ORDER,
    SNAP_MUTABLE_FLAGS,
    parse_gridgraph,
)

# These flags mirror the C++ Node declaration order and are reconstructed from
# the shared text dump plus the snapshot's override bitmap.
BYTE0_FLAGS = FLAG_BITS[:6]
BYTE1_FLAGS = FLAG_BITS[6:]
COST_ORDER = COST_BITS + NDR_COLS  # sparse cost mask bit order
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
    x_dim, y_dim, z_dim, cost_bits, cost_records, search_id = struct.unpack(
        "<6i", blob[8:SNAP_HEADER_BYTES])
    if cost_records < 0:
        die("{} has a negative sparse cost-record count".format(path))
    n = x_dim * y_dim * z_dim
    bm = (n + 7) // 8
    off = SNAP_HEADER_BYTES
    sparse_costs = {}
    prev_addr = -1
    for _ in range(cost_records):
        if off + 6 > len(blob):
            die("{} ends inside a sparse cost record".format(path))
        addr = struct.unpack("<I", blob[off:off + 4])[0]
        mask = struct.unpack("<H", blob[off + 4:off + 6])[0]
        off += 6
        n_values = bin(mask).count("1")
        if (addr >= n or addr <= prev_addr or mask == 0
                or mask >> len(SNAP_COST_ORDER)):
            die("{} has an invalid sparse cost record at address {}".format(
                path, addr))
        if off + n_values > len(blob):
            die("{} ends inside sparse cost values at address {}".format(
                path, addr))
        costs = [0] * len(SNAP_COST_ORDER)
        for field in range(len(SNAP_COST_ORDER)):
            if mask & (1 << field):
                costs[field] = blob[off]
                off += 1
        sparse_costs[addr] = costs
        prev_addr = addr
    bitmap_count = len(SNAP_MUTABLE_FLAGS) + 3
    want = off + bitmap_count * bm
    if len(blob) != want:
        die("{} is {} bytes, expected {} for dim {}x{}x{}".format(
            path, len(blob), want, x_dim, y_dim, z_dim))
    meta = {"dim": (x_dim, y_dim, z_dim), "searchId": search_id,
            "costBits": cost_bits, "costRecords": cost_records, "n": n}
    bitmaps = [blob[off + i * bm:off + (i + 1) * bm]
               for i in range(bitmap_count)]
    return meta, sparse_costs, bitmaps


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("snapshot", help="dumpSearchGraph() .bin")
    ap.add_argument("--gg", default=None,
                    help="required gg_*.txt baseline from the same route box")
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

    meta, sparse_costs, bitmaps = read_snapshot(args.snapshot)
    mutable_bitmaps = bitmaps[:len(SNAP_MUTABLE_FLAGS)]
    guides, srcs, dsts = bitmaps[len(SNAP_MUTABLE_FLAGS):]
    x_dim, y_dim, z_dim = meta["dim"]
    if args.layer is not None and not 0 <= args.layer < z_dim:
        die("--layer {} is outside 0..{}".format(args.layer, z_dim - 1))

    if not args.gg or not os.path.isfile(args.gg):
        die("--gg must name the shared grid dump for this snapshot")
    hdr, base_nodes = parse_gridgraph(
        args.gg, ignore_ndr=True, want_nodes=True, check_ndr_fields=False)
    if tuple(hdr["dim"]) != meta["dim"]:
        die("{} dim {} does not match snapshot dim {} -- not the same "
            "route box".format(args.gg, tuple(hdr["dim"]), meta["dim"]))
    xc, yc, layers = hdr["xCoords"], hdr["yCoords"], hdr["zCoords(layerNum)"]

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
                    byte, mask = addr >> 3, 1 << (addr & 7)
                    row = [addr, x, y, z,
                           xc[x], yc[y], layers[z]]
                    row += [(base_nodes[addr] >> i) & 1
                            for i in range(len(FLAG_BITS))]
                    for flag, bitmap in zip(SNAP_MUTABLE_FLAGS,
                                            mutable_bitmaps):
                        row[7 + FLAG_BITS.index(flag)] = (
                            1 if bitmap[byte] & mask else 0)
                    row += sparse_costs.get(
                        addr, [0] * len(SNAP_COST_ORDER))
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