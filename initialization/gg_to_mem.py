#!/usr/bin/env python3
"""Convert FlexGridGraph / forbidden-length dumps into $readmemh BRAM images.

Reads the dumps produced by FlexGridGraph::dumpGridGraph(),
FlexGridGraph::dumpSearchGraph() and FlexRP::dumpForbiddenTables() and emits
one .mem file per BRAM, plus a .vh of the scalar constants (which stay as
localparams -- they do not belong in a memory).

    python3 initialization/gg_to_mem.py \
        --gg        work/ggdump/gg_iter0_x0_y151200.txt \
        --forbidden work/ggdump/forbidden_tables.txt \
        --outdir    initialization/mem


NODE STATE SOURCE.  --gg alone gives the *pre-routing* snapshot: dumpGridGraph()
runs at the end of FlexGridGraph::init(), before mazeNetInit() stamps any net's
guides and before any rip-up/route moves the cost counters, so its guide column
is uniformly 0 and its src/dst/overrideShapeCostVia state does not exist yet.

Pass --snapshot to overlay the binary dumpSearchGraph() image instead.  That is
taken on entry to one search() call, which is the state the A* actually reads,
and is what an accelerator must be validated against:

    python3 initialization/gg_to_mem.py \
        --gg        work/ggdump/gg_iter1_x136800_y136800.txt \
        --snapshot  work/ggdump/ggs_iter1_x136800_y136800_s0.bin \
        --forbidden work/ggdump/forbidden_tables.txt

--gg is still required with --snapshot: the binary carries only the mutable
per-node state, while the coordinate arrays, layer directions and cost scalars
are static after init() and are read from the text dump of the same route box.
Match the iter/x/y of the two files -- the script checks dim but cannot check
that you picked the same box at the same iteration.

Word widths are sized to the data, not to a machine word: each is the narrowest
field layout that holds the values, rounded up to a multiple of 4 so one hex
digit is one nibble and $readmemh stays exact.  Every .mem opens with a comment
block giving its bit-by-bit breakdown, and gg_params.vh exports the same widths
and offsets as localparams so the RTL slices never go out of sync with the file.

Outputs (all $readmemh-ready: bare hex, one word per line, no prefixes):

    gg_nodes.mem                167*232*6 x 28b   main grid-graph memory
    gg_xcoords.mem              167       x 20b   signed DBU, indexed by x
    gg_ycoords.mem              232       x 20b   signed DBU, indexed by y
    gg_zcoords.mem              6         x 20b   zHeights, DBU, indexed by z
    gg_layers.mem               6         x 16b   {dir, minWidth}, indexed by z
    via2via_forbidden_len.mem   6*8       x 48b   N x {lo, hi}
    via_forbidden_turn_len.mem  6*4       x 48b   N x {lo, hi}
    gg_params.vh                widths, offsets, bit indices, scalar constants


NODE WORD -- 28 bits:

    [27:25]  prevAstarNodeDir   3b   A* scratch, frDirEnum, 0 = UNKNOWN
    [24]     closed             1b   A* scratch
    [23: 0]  node state        24b   loaded from the dump

The top four bits are working storage for the search, not dump data: both are 0
in every word this script emits, because on entry to search() nothing is closed
and every prevAstarNodeDir is UNKNOWN.  They are carved out of the node word so
the RTL can read state and write back its A* bookkeeping in one memory rather
than maintaining a second array in parallel -- which is what the C++ does, in
the separate prevDirs_ bit vector.

The word carries no x/y/z.  Under the canonical address mapping below the
address is a bijection onto (x, y, z), so any reader has the indices in hand
before the word comes back, and gg_{x,y,z}coords.mem turn those indices into
DBU.  Re-embedding them would only pay off for a *compacted* node memory --
only the ~41% of nodes with any state bit set -- where a word must
self-identify; add a tag field alongside the entries if that day comes.

Boolean bits, LSB first.  Bits 0..13 mirror the C++ Node byte0/byte1 flag
layout; 14..20 are the non-NDR cost fields reduced to a nonzero test; 21..23 are
the separate guides_ / srcs_ / dsts_ bit vectors.

Bits 7 (overrideShapeCostVia), 22 (src) and 23 (dst) only ever carry data from
--snapshot.  The text dump has no column for them, and all three are routing-
time state that is uniformly 0 in a pre-routing dump anyway, so a --gg-only run
zeroing them is correct rather than lossy.

Reducing the cost counters to one bit each is lossless for the A* cost model:
FlexGridGraph::getCosts() assigns every one of them to a `bool` and uses it only
to gate a fixed `gg*Cost_ * edgeLength` term -- the stored magnitude never
reaches the cost sum.  The magnitudes matter only to the addToByte/subFromByte
bookkeeping during route and rip-up, which a snapshot consumer does not do.

The 5 NDR cost fields are dropped.  getCosts() only consults them when
useNDRCosts() is true, which requires ndr_ != nullptr, so a non-NDR net never
reads them.  If a dump ever carries a nonzero NDR field the script stops rather
than drop it silently; --ignore-ndr overrides.


COORDINATE WIDTH is derived from the values present and shared by all three
coordinate files, so the getEstCost datapath needs no per-axis sign extension.
These are absolute DBU, so the width tracks the route box's position on the die:
a box at a larger y needs more bits than this one.  Pin it with --coord-bits
when generating images for several boxes that must share RTL.


NODE ADDRESS is canonical z-major, x-fastest:

    addr = (z * yDim + y) * xDim + x

This is the dump's own line order.  It deliberately does NOT match C++
FlexGridGraph::getIdx(), which swaps to y-fastest on HORIZONTAL layers as a
cache-locality trick.  Nothing outside the C++ walker depends on that layout,
and a uniform stride keeps RTL address generation to one multiply-add.
"""

import argparse
import os
import struct
import sys

# odb::dbTechLayerDir::Value.  Note the dumpGridGraph() column header says
# "layerDir(0=H,1=V,2=NONE)" -- that label is wrong; it prints the raw odb enum.
DIR_NAMES = {0: "NONE", 1: "HORIZONTAL", 2: "VERTICAL"}

# Boolean payload, LSB first.  This order is also the C++ Node byte0/byte1 bit
# order, which parse_snapshot() relies on to copy both bytes in one shift
# instead of bit by bit -- keep the two in step.
FLAG_BITS = [
    "hasEastEdge",
    "hasNorthEdge",
    "hasUpEdge",
    "isBlockedEast",
    "isBlockedNorth",
    "isBlockedUp",
    "hasSpecialVia",
    "overrideShapeCostVia",
    "hasGridCostEast",
    "hasGridCostNorth",
    "hasGridCostUp",
    "hasApCostEast",
    "hasApCostNorth",
    "hasApCostUp",
]

# Flags the text dump has no column for.  --snapshot supplies them; a --gg-only
# run leaves them 0, which is their pre-routing value anyway.
TEXT_MISSING = {"overrideShapeCostVia"}

# Non-NDR cost counters, reduced to a nonzero test.  Continue the bit numbering.
COST_BITS = [
    "routeShapeCostPlanar",
    "routeShapeCostVia",
    "markerCostPlanar",
    "markerCostVia",
    "fixedShapeCostVia",
    "fixedShapeCostPlanarHorz",
    "fixedShapeCostPlanarVert",
]

# Dropped: only read when useNDRCosts() is true, i.e. ndr_ != nullptr.
NDR_COLS = [
    "routeShapeCostPlanarNDR",
    "routeShapeCostViaNDR",
    "fixedShapeCostViaNDR",
    "fixedShapeCostPlanarHorzNDR",
    "fixedShapeCostPlanarVertNDR",
]

# Bit vectors kept outside Node in C++, appended after the cost bits.
VECTOR_BITS = ["guide", "src", "dst"]
GUIDE_BIT = len(FLAG_BITS) + len(COST_BITS)  # 21
SRC_BIT = GUIDE_BIT + 1  # 22
DST_BIT = GUIDE_BIT + 2  # 23
BOOL_BITS = GUIDE_BIT + len(VECTOR_BITS)  # 24

# Per-node A* scratch, written by the search rather than loaded from a dump.
# Both are 0 on entry to search() -- prevAstarNodeDir is UNKNOWN and nothing is
# closed yet -- so the .mem ships them zeroed and the RTL owns them from there.
# In C++ these live outside Node, in the separate prevDirs_ bit vector.
CLOSED_BIT = BOOL_BITS  # 24
DIR_LSB = BOOL_BITS + 1  # 25
DIR_BITS = 3
SCRATCH_BITS = 1 + DIR_BITS  # 4
MIN_NODE_BITS = BOOL_BITS + SCRATCH_BITS  # 28

# frDirEnum, from src/drt/src/frBaseTypes.h.  UNKNOWN = 0 is the reset value.
DIR_ENUM = [("UNKNOWN", 0), ("D", 1), ("S", 2), ("W", 3),
            ("E", 4), ("N", 5), ("U", 6)]

# gg_layers.mem: preferred direction (raw odb::dbTechLayerDir, 0..2) over the
# layer's minWidth.  Named GG_LAYER_DIR_* so they cannot collide with the
# frDirEnum GG_DIR_* above -- these are two unrelated encodings.
LAYER_DIR_BITS = 2
LAYER_DIR_ENUM = [("NONE", 0), ("HORIZONTAL", 1), ("VERTICAL", 2)]

# Column index of each name in a "node" row (token 0 is the literal "node").
# The text dump carries neither overrideShapeCostVia nor src/dst.
NODE_COLUMNS = (
    ["x", "y", "z"]
    + [n for n in FLAG_BITS if n not in TEXT_MISSING]
    + COST_BITS
    + NDR_COLS
    + ["guide"]
)
COL = {name: i + 1 for i, name in enumerate(NODE_COLUMNS)}

# ---- dumpSearchGraph() binary format (see FlexGridGraph_maze.cpp) ----
SNAP_MAGIC = b"DRTGGS01"
SNAP_HEADER_BYTES = 32  # magic + 6 int32
SNAP_STRIDE = 14  # bytes per node record
# Cost counters occupy record bytes 2..13 in this order.
SNAP_COST_ORDER = COST_BITS + NDR_COLS


def die(msg):
    sys.exit("error: " + msg)


def nibble_align(bits):
    """Round up to a multiple of 4 so one hex digit is exactly one nibble."""
    return -(-bits // 4) * 4


def signed_bits(lo, hi):
    """Narrowest two's-complement width holding every value in [lo, hi]."""
    n = 1
    while lo < -(1 << (n - 1)) or hi > (1 << (n - 1)) - 1:
        n += 1
    return n


def unsigned_bits(hi):
    return max(1, hi.bit_length())


def hex_word(value, bits):
    """Two's-complement hex, no 0x, padded to the word width."""
    return format(value & ((1 << bits) - 1), "0{}x".format(bits // 4))


def bit_range(hi, lo):
    return "[{}]".format(hi) if hi == lo else "[{}:{}]".format(hi, lo)


def write_mem(path, words, bits, title, layout, notes=()):
    """Emit a .mem with a header describing how each word is divided up.

    layout: list of (hi_bit, lo_bit, name, note) rows, MSB first.
    """
    name = os.path.basename(path)
    with open(path, "w") as f:
        f.write("// {} -- {} words x {} bits ({} hex digits per line)\n".format(
            name, len(words), bits, bits // 4))
        f.write("// {}\n//\n".format(title))
        span = max(len(bit_range(h, l)) for h, l, _, _ in layout)
        field = max(len(n) for _, _, n, _ in layout)
        for hi, lo, fname, note in layout:
            f.write("//   {:<{s}}  {:<{f}}  {:>2}b  {}\n".format(
                bit_range(hi, lo), fname, hi - lo + 1, note, s=span, f=field))
        if notes:
            f.write("//\n")
            for line in notes:
                f.write("//   {}\n".format(line) if line else "//\n")
        f.write("//\n// load with $readmemh\n")
        for w in words:
            f.write(hex_word(w, bits) + "\n")
    print("  {:<30} {:>8} x {:>3}b".format(name, len(words), bits))


def node_layout(node_bits):
    rows = []
    if node_bits > MIN_NODE_BITS:
        rows.append((node_bits - 1, MIN_NODE_BITS, "pad", "zero"))
    rows += [
        (DIR_LSB + DIR_BITS - 1, DIR_LSB, "prevAstarNodeDir",
         "frDirEnum, 0 = UNKNOWN (A* scratch, 0 here)"),
        (CLOSED_BIT, CLOSED_BIT, "closed", "A* scratch, 0 here"),
        (BOOL_BITS - 1, 0, "node state", "one bit each, see below"),
    ]
    return rows


def bool_names(from_snapshot):
    tail = "" if from_snapshot else " (--snapshot only, 0 here)"
    names = [n + (tail if n in TEXT_MISSING else "") for n in FLAG_BITS]
    names += [c + " != 0" for c in COST_BITS]
    names += ["guide", "src" + tail, "dst" + tail]
    return names


def parse_gridgraph(path, ignore_ndr, want_nodes=True):
    """Return (header dict, node words indexed by canonical address).

    With want_nodes False the per-node rows are skipped and None is returned
    in their place -- used when --snapshot supersedes them, so the 232k-line
    table is not parsed just to be thrown away.
    """
    header = {}
    nodes = None
    x_dim = y_dim = z_dim = None
    dropped_ndr = {}
    seen = 0

    with open(path) as f:
        for line in f:
            if not line or line[0] == "#":
                continue
            tok = line.split()
            if not tok:
                continue
            key = tok[0]

            if key == "node":
                if not want_nodes:
                    continue
                if nodes is None:
                    die("'node' rows appear before the 'dim' line in " + path)
                x, y, z = int(tok[1]), int(tok[2]), int(tok[3])
                if not (0 <= x < x_dim and 0 <= y < y_dim and 0 <= z < z_dim):
                    die("node ({}, {}, {}) is outside dim {}x{}x{}".format(
                        x, y, z, x_dim, y_dim, z_dim))

                word = 0
                for bit, name in enumerate(FLAG_BITS):
                    if name not in TEXT_MISSING and int(tok[COL[name]]):
                        word |= 1 << bit
                for i, name in enumerate(COST_BITS):
                    if int(tok[COL[name]]):
                        word |= 1 << (len(FLAG_BITS) + i)
                if int(tok[COL["guide"]]):
                    word |= 1 << GUIDE_BIT

                for name in NDR_COLS:
                    if int(tok[COL[name]]):
                        dropped_ndr[name] = dropped_ndr.get(name, 0) + 1

                addr = (z * y_dim + y) * x_dim + x
                if nodes[addr] is not None:
                    die("duplicate node at ({}, {}, {})".format(x, y, z))
                nodes[addr] = word
                seen += 1

            elif key == "dim":
                x_dim, y_dim, z_dim = int(tok[1]), int(tok[2]), int(tok[3])
                header["dim"] = (x_dim, y_dim, z_dim)
                if want_nodes:
                    nodes = [None] * (x_dim * y_dim * z_dim)

            elif key in ("xCoords", "yCoords", "zCoords(layerNum)", "zHeights",
                         "layerDir(0=H,1=V,2=NONE)", "layerMinWidth", "routeBox"):
                header[key] = [int(v) for v in tok[1:]]

            elif len(tok) == 2:
                header[key] = int(tok[1])

    if "dim" not in header:
        die("no 'dim' line found in " + path)
    if want_nodes:
        if seen != len(nodes):
            die("expected {} nodes, found {} in {}".format(
                len(nodes), seen, path))
        check_ndr(dropped_ndr, ignore_ndr, path)

    return header, nodes


def check_ndr(dropped_ndr, ignore_ndr, path):
    if dropped_ndr and not ignore_ndr:
        die("{} has nonzero NDR cost fields ({}) which the node word does not "
            "carry -- pass --ignore-ndr to drop them anyway".format(
                path,
                ", ".join("{}x{}".format(v, k)
                          for k, v in sorted(dropped_ndr.items()))))


def parse_snapshot(path, ignore_ndr):
    """Read a dumpSearchGraph() .bin -> (node words, meta dict).

    The binary carries only mutable per-node state; the caller still needs a
    --gg text dump for the coordinate arrays and cost scalars.
    """
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
    bm_bytes = (n + 7) // 8
    node_end = SNAP_HEADER_BYTES + n * stride
    want = node_end + 3 * bm_bytes
    if len(blob) != want:
        die("{} is {} bytes, expected {} for dim {}x{}x{}".format(
            path, len(blob), want, x_dim, y_dim, z_dim))
    guides = blob[node_end:node_end + bm_bytes]
    srcs = blob[node_end + bm_bytes:node_end + 2 * bm_bytes]
    dsts = blob[node_end + 2 * bm_bytes:node_end + 3 * bm_bytes]

    n_cost = len(COST_BITS)
    ndr_base = 2 + n_cost
    words = [0] * n
    dropped_ndr = {}
    off = SNAP_HEADER_BYTES

    # The record order is the canonical z-major, x-fastest address order, so a
    # flat walk lands each record at its own address with no index arithmetic.
    for addr in range(n):
        # Record bytes 0 and 1 are the C++ Node flag bytes, and FLAG_BITS is in
        # that same order: byte 0 supplies bits 0..5 (bits 6,7 are the C++
        # unused1/unused2) and byte 1 bits 6..13.
        word = (blob[off] & 0x3f) | (blob[off + 1] << 6)
        for i in range(n_cost):
            if blob[off + 2 + i]:
                word |= 1 << (len(FLAG_BITS) + i)
        for i, name in enumerate(NDR_COLS):
            if blob[off + ndr_base + i]:
                dropped_ndr[name] = dropped_ndr.get(name, 0) + 1
        byte, mask = addr >> 3, 1 << (addr & 7)
        if guides[byte] & mask:
            word |= 1 << GUIDE_BIT
        if srcs[byte] & mask:
            word |= 1 << SRC_BIT
        if dsts[byte] & mask:
            word |= 1 << DST_BIT
        words[addr] = word
        off += stride

    check_ndr(dropped_ndr, ignore_ndr, path)
    meta = {
        "dim": (x_dim, y_dim, z_dim),
        "searchId": search_id,
        "costBits": cost_bits,
        "srcs": sum(bin(b).count("1") for b in srcs),
        "dsts": sum(bin(b).count("1") for b in dsts),
        "guides": sum(bin(b).count("1") for b in guides),
    }
    return words, meta


def parse_forbidden(path):
    """Return (numLayers, via2via, turn) with each table {(z, entry): [(lo, hi)]}.

    An entry may hold several disjoint ranges -- e.g. metal6 prev-up/curr-up is
    [0, 600] and [800, 1600], with 601..799 legal between them -- so the list is
    kept as-is and pack_forbidden() gives every entry the same number of slots.
    """
    via2via, turn = {}, {}
    num_layers = None

    with open(path) as f:
        for line in f:
            if not line or line[0] == "#":
                continue
            tok = line.split()
            if not tok:
                continue

            if tok[0] == "numLayers":
                num_layers = int(tok[1])
                continue
            if tok[0] not in ("via2ViaForbiddenLen", "viaForbiddenTurnLen"):
                continue

            table = via2via if tok[0] == "via2ViaForbiddenLen" else turn
            entries = 8 if tok[0] == "via2ViaForbiddenLen" else 4
            z, entry, n_ranges = int(tok[1]), int(tok[2]), int(tok[3])

            if not 0 <= entry < entries:
                die("{} entry {} out of range 0..{}".format(tok[0], entry, entries - 1))
            vals = [int(v) for v in tok[4:]]
            if len(vals) != 2 * n_ranges:
                die("{} z={} entry={} claims {} ranges but has {} bounds".format(
                    tok[0], z, entry, n_ranges, len(vals)))
            table[(z, entry)] = list(zip(vals[0::2], vals[1::2]))

    if num_layers is None:
        die("no 'numLayers' line found in " + path)
    return num_layers, via2via, turn


def pack_forbidden(table, num_layers, entries, name, half, n_slots):
    """Flatten to words of n_slots {lo, hi} pairs, slot 0 in the high bits.

    Ranges are closed on both ends, so an unused slot is encoded as the
    unsatisfiable range lo = all-ones, hi = 0: the RTL compare (lo <= len <= hi)
    is false for every len, so the slots need no valid bits and an entry with
    fewer ranges than n_slots costs nothing but the OR of a dead comparator.
    """
    pair_bits = 2 * half
    empty = ((1 << half) - 1) << half
    words = []
    for z in range(num_layers):
        for entry in range(entries):
            if (z, entry) not in table:
                die("{} missing z={} entry={}".format(name, z, entry))
            ranges = table[(z, entry)]
            word = 0
            for slot in range(n_slots):
                if slot < len(ranges):
                    lo, hi = ranges[slot]
                    pair = (lo << half) | hi
                else:
                    pair = empty
                word |= pair << (pair_bits * (n_slots - 1 - slot))
            words.append(word)
    return words


def write_params(path, header, node_bits, coord_bits, forbid_half, forbid_slots,
                 minwidth_bits, layer_bits, snap):
    x_dim, y_dim, z_dim = header["dim"]
    dirs = header["layerDir(0=H,1=V,2=NONE)"]
    layer_nums = header["zCoords(layerNum)"]
    min_widths = header["layerMinWidth"]

    with open(path, "w") as f:
        f.write("// Generated by initialization/gg_to_mem.py -- do not edit.\n")
        f.write("// Widths and offsets for the .mem images, plus the scalar\n")
        f.write("// grid-graph constants (which are not worth a memory).\n//\n")
        f.write("// Route box (DBU): {} {} {} {}\n".format(*header["routeBox"]))
        if snap is None:
            f.write("// Node state: pre-routing dumpGridGraph() snapshot --\n"
                    "// no guides stamped, no src/dst set.\n\n")
        else:
            f.write("// Node state: dumpSearchGraph() snapshot of search {} --\n"
                    "// {} guide, {} src, {} dst nodes.\n\n".format(
                        snap["searchId"], snap["guides"], snap["srcs"],
                        snap["dsts"]))

        f.write("localparam integer GG_X_DIM      = {};\n".format(x_dim))
        f.write("localparam integer GG_Y_DIM      = {};\n".format(y_dim))
        f.write("localparam integer GG_Z_DIM      = {};\n".format(z_dim))
        f.write("localparam integer GG_NUM_NODES  = {};\n".format(x_dim * y_dim * z_dim))
        f.write("localparam integer GG_ADDR_BITS  = {};\n".format(
            max(1, (x_dim * y_dim * z_dim - 1).bit_length())))
        f.write("// addr = (z * GG_Y_DIM + y) * GG_X_DIM + x\n\n")

        f.write("// ---- gg_nodes.mem ----\n")
        f.write("// x/y/z are recoverable from the address above, so the word is\n")
        f.write("// state [{}:0] plus the A* scratch the search writes back.\n".format(
            BOOL_BITS - 1))
        f.write("localparam integer GG_NODE_BITS  = {};\n".format(node_bits))
        f.write("localparam integer GG_STATE_W    = {};\n".format(BOOL_BITS))
        f.write("localparam integer GG_BIT_CLOSED = {};\n".format(CLOSED_BIT))
        f.write("localparam integer GG_DIR_LSB    = {};\n".format(DIR_LSB))
        # Not GG_DIR_W -- that is West below, and Verilog would see a redeclaration.
        f.write("localparam integer GG_DIR_BITS   = {};\n".format(DIR_BITS))
        f.write("// Both scratch fields are 0 in this image: on entry to search()\n")
        f.write("// no node is closed and every prevAstarNodeDir is UNKNOWN.\n")
        for name, val in DIR_ENUM:
            f.write("localparam [{}:0] GG_DIR_{:<8} = {}'d{};\n".format(
                DIR_BITS - 1, name, DIR_BITS, val))
        f.write("\n")

        names = bool_names(snap is not None)
        keys = [n.split()[0] for n in names]
        width = max(len(k) for k in keys)
        for bit, key in enumerate(keys):
            f.write("localparam integer GG_BIT_{:<{w}} = {};\n".format(key, bit, w=width))
        f.write("\n")

        f.write("// ---- gg_{x,y,z}coords.mem ----\n")
        f.write("// Signed two's complement DBU.  One width for all three axes so\n")
        f.write("// the getEstCost datapath needs no per-axis sign extension.\n")
        f.write("localparam integer GG_COORD_BITS = {};\n\n".format(coord_bits))

        f.write("// ---- via*_forbidden_*.mem ----\n")
        f.write("// Each word is GG_FORBID_SLOTS {lo, hi} pairs, slot 0 in the high\n")
        f.write("// bits; slot s occupies [GG_FORBID_PAIR*(GG_FORBID_SLOTS-s) - 1 :\n")
        f.write("// GG_FORBID_PAIR*(GG_FORBID_SLOTS-1-s)].  An entry needs more than\n")
        f.write("// one slot when its forbidden lengths are disjoint.\n")
        f.write("localparam integer GG_FORBID_SLOTS  = {};\n".format(forbid_slots))
        f.write("localparam integer GG_FORBID_PAIR   = {};\n".format(2 * forbid_half))
        f.write("localparam integer GG_FORBID_BITS   = {};\n".format(
            2 * forbid_half * forbid_slots))
        f.write("localparam integer GG_FORBID_HALF   = {};  // within a pair: "
                "lo = [{}:{}], hi = [{}:0]\n".format(
                    forbid_half, 2 * forbid_half - 1, forbid_half, forbid_half - 1))
        f.write("localparam integer GG_V2V_ENTRIES   = {};\n".format(8 * z_dim))
        f.write("localparam integer GG_TURN_ENTRIES  = {};\n".format(4 * z_dim))
        f.write("// forbidden when lo <= len <= hi in ANY slot; unused slot is\n")
        f.write("// lo=all-ones, hi=0, which no len can satisfy\n\n")

        for name in ("ggDRCCost", "ggMarkerCost", "ggFixedShapeCost",
                     "GRIDCOST", "BLOCKCOST", "GUIDECOST"):
            f.write("localparam integer GG_{:<16} = {};\n".format(
                name.replace("gg", "").upper(), header[name]))
        f.write("\n")

        f.write("// ---- gg_layers.mem ----\n")
        f.write("// Per-layer constants, indexed by z.  A memory rather than\n")
        f.write("// localparams so the direction and minWidth are readable at\n")
        f.write("// runtime with the z the datapath already has, instead of a\n")
        f.write("// {}-way case that has to be re-elaborated per design.\n".format(z_dim))
        f.write("// Direction is the raw odb::dbTechLayerDir enum; the dump's own\n")
        f.write("// column header mislabels it as 0=H/1=V/2=NONE -- ignore it.\n")
        f.write("localparam integer GG_LAYER_BITS     = {};\n".format(layer_bits))
        f.write("localparam integer GG_MINWIDTH_BITS  = {};  // [{}:0]\n".format(
            minwidth_bits, minwidth_bits - 1))
        f.write("localparam integer GG_LAYER_DIR_LSB  = {};\n".format(minwidth_bits))
        f.write("localparam integer GG_LAYER_DIR_BITS = {};\n".format(LAYER_DIR_BITS))
        for name, val in LAYER_DIR_ENUM:
            f.write("localparam [{}:0] GG_LAYER_DIR_{:<10} = {}'d{};\n".format(
                LAYER_DIR_BITS - 1, name, LAYER_DIR_BITS, val))
        f.write("// per-z values, for reference -- read them from the .mem, not here:\n")
        f.write("//   dir      {}\n".format(
            " ".join(DIR_NAMES.get(d, "?")[:1] or "?" for d in dirs)))
        f.write("//   minWidth {}\n\n".format(" ".join(str(w) for w in min_widths)))

        f.write("// frLayerNum per z, for tracing back to the LEF; not needed at runtime\n")
        for z in range(z_dim):
            f.write("localparam integer GG_LAYERNUM_{}    = {};\n".format(z, layer_nums[z]))
    print("  {:<30} {:>8}".format(os.path.basename(path), "params"))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gg", default="work/ggdump/gg_iter1_x136800_y136800.txt",
                    help="grid-graph dump (default: %(default)s)")
    ap.add_argument("--snapshot", default=None,
                    help="dumpSearchGraph() .bin for one search; supplies the "
                         "node state (guides, src/dst, routing-time costs) in "
                         "place of the pre-routing rows in --gg")
    ap.add_argument("--forbidden", default="work/ggdump/forbidden_tables.txt",
                    help="forbidden-length dump (default: %(default)s)")
    ap.add_argument("--outdir", default="initialization/mem",
                    help="output directory (default: %(default)s)")
    ap.add_argument("--node-bits", type=int, default=nibble_align(MIN_NODE_BITS),
                    help="node word width, min {} (default: %(default)s)".format(
                        MIN_NODE_BITS))
    ap.add_argument("--coord-bits", type=int, default=None,
                    help="coordinate width; default is derived from the values")
    ap.add_argument("--forbidden-bits", type=int, default=None,
                    help="width of one {lo, hi} pair; default is derived")
    ap.add_argument("--minwidth-bits", type=int, default=None,
                    help="minWidth field width in gg_layers.mem; default is "
                         "derived from the values")
    ap.add_argument("--forbidden-slots", type=int, default=None,
                    help="ranges per table entry; default is the most any entry "
                         "in --forbidden needs")
    ap.add_argument("--ignore-ndr", action="store_true",
                    help="drop nonzero NDR cost fields instead of stopping")
    args = ap.parse_args()

    for p in (args.gg, args.forbidden, args.snapshot):
        if p is not None and not os.path.isfile(p):
            die("no such file: " + p)
    for name, val in (("--node-bits", args.node_bits),
                      ("--coord-bits", args.coord_bits),
                      ("--minwidth-bits", args.minwidth_bits),
                      ("--forbidden-bits", args.forbidden_bits)):
        if val is not None and val % 4:
            die("{} {} is not a multiple of 4, so it cannot be written as "
                "whole hex digits".format(name, val))
    if args.node_bits < MIN_NODE_BITS:
        die("--node-bits {} is too small: the node state is {} bits".format(
            args.node_bits, MIN_NODE_BITS))
    if not os.path.isdir(args.outdir):
        os.makedirs(args.outdir)

    header, nodes = parse_gridgraph(args.gg, args.ignore_ndr,
                                    want_nodes=args.snapshot is None)
    snap = None
    if args.snapshot is not None:
        nodes, snap = parse_snapshot(args.snapshot, args.ignore_ndr)
        if snap["dim"] != header["dim"]:
            die("snapshot dim {} does not match {} dim {} -- the two files are "
                "not the same route box".format(
                    snap["dim"], args.gg, header["dim"]))
        if snap["costBits"] != 8:
            print("  note: snapshot built with cost_bits={}, cost bytes "
                  "saturated at 255 (only tested for nonzero here)".format(
                      snap["costBits"]))
    num_layers, via2via, turn = parse_forbidden(args.forbidden)

    x_dim, y_dim, z_dim = header["dim"]
    xc, yc, zh = header["xCoords"], header["yCoords"], header["zHeights"]
    if (len(xc), len(yc), len(zh)) != (x_dim, y_dim, z_dim):
        die("coordinate array lengths {} do not match dim {}".format(
            (len(xc), len(yc), len(zh)), (x_dim, y_dim, z_dim)))
    if num_layers != z_dim:
        die("forbidden tables have {} layers but the grid graph has {}".format(
            num_layers, z_dim))

    # --- coordinate width: one signed width covering all three axes ---
    all_coords = xc + yc + zh
    need = signed_bits(min(all_coords), max(all_coords))
    coord_bits = args.coord_bits if args.coord_bits is not None else nibble_align(need)
    if coord_bits < need:
        die("--coord-bits {} cannot hold the range {}..{} (needs {} signed)".format(
            coord_bits, min(all_coords), max(all_coords), need))

    # --- per-layer word: {dir, minWidth}, minWidth sized from the values ---
    dirs = header["layerDir(0=H,1=V,2=NONE)"]
    min_widths = header["layerMinWidth"]
    if len(dirs) != z_dim or len(min_widths) != z_dim:
        die("layerDir/layerMinWidth have {}/{} entries, expected {}".format(
            len(dirs), len(min_widths), z_dim))
    for z, d in enumerate(dirs):
        if not 0 <= d < (1 << LAYER_DIR_BITS):
            die("layerDir[{}] = {} does not fit the {}-bit direction field".format(
                z, d, LAYER_DIR_BITS))
    if min(min_widths) < 0:
        die("layerMinWidth has a negative entry: {}".format(min(min_widths)))
    mw_need = unsigned_bits(max(min_widths))
    minwidth_bits = (args.minwidth_bits if args.minwidth_bits is not None
                     else nibble_align(mw_need))
    if minwidth_bits < mw_need:
        die("--minwidth-bits {} cannot hold the max minWidth {} (needs {})".format(
            minwidth_bits, max(min_widths), mw_need))
    layer_bits = nibble_align(minwidth_bits + LAYER_DIR_BITS)
    layer_words = [(d << minwidth_bits) | w for d, w in zip(dirs, min_widths)]

    # --- forbidden width: lo and hi share a half-word, n_slots pairs per word ---
    all_entries = list(via2via.values()) + list(turn.values())
    ranges = [r for entry in all_entries for r in entry]
    forbid_slots = max((len(entry) for entry in all_entries), default=1) or 1
    if args.forbidden_slots is not None:
        if args.forbidden_slots < forbid_slots:
            die("--forbidden-slots {} is too few: an entry in {} has {} "
                "ranges".format(args.forbidden_slots, args.forbidden, forbid_slots))
        forbid_slots = args.forbidden_slots
    max_lo = max((lo for lo, _ in ranges), default=0)
    max_hi = max((hi for _, hi in ranges), default=0)
    # The empty sentinel is lo = all-ones, so a real lo must stay below it.
    half_need = max(unsigned_bits(max_hi), unsigned_bits(max_lo + 1))
    if args.forbidden_bits is not None:
        if args.forbidden_bits % 8:
            die("--forbidden-bits {} must be a multiple of 8 so lo and hi split "
                "evenly into nibble-aligned halves".format(args.forbidden_bits))
        forbid_half = args.forbidden_bits // 2
        if forbid_half < half_need:
            die("--forbidden-bits {} gives {}-bit halves, too small for lo max {} "
                "/ hi max {} (needs {})".format(args.forbidden_bits, forbid_half,
                                                max_lo, max_hi, half_need))
    else:
        forbid_half = nibble_align(half_need)

    forbid_bits = 2 * forbid_half * forbid_slots
    v2v_words = pack_forbidden(via2via, num_layers, 8, "via2ViaForbiddenLen",
                               forbid_half, forbid_slots)
    turn_words = pack_forbidden(turn, num_layers, 4, "viaForbiddenTurnLen",
                                forbid_half, forbid_slots)

    def coord_layout(label, values):
        return [(coord_bits - 1, 0, label, "signed, range {}..{} (needs {}b)".format(
            min(values), max(values), signed_bits(min(values), max(values))))]

    def forbid_layout():
        rows = []
        for slot in range(forbid_slots):
            base = 2 * forbid_half * (forbid_slots - 1 - slot)
            rows.append((base + 2 * forbid_half - 1, base + forbid_half,
                         "lo{}".format(slot), "inclusive lower bound"))
            rows.append((base + forbid_half - 1, base,
                         "hi{}".format(slot), "inclusive upper bound"))
        return rows

    forbid_notes = [
        "forbidden when lo <= len <= hi in ANY of the {} slots".format(forbid_slots),
        "unused slot = lo {}, hi {} -- unsatisfiable, so no valid bit is needed"
        .format("0x" + "f" * (forbid_half // 4), "0x" + "0" * (forbid_half // 4)),
    ]

    out = lambda n: os.path.join(args.outdir, n)
    print("writing to {}/".format(args.outdir))

    names = bool_names(snap is not None)
    if snap is None:
        state_notes = ["state from {} (pre-routing)".format(
            os.path.basename(args.gg)), ""]
    else:
        state_notes = ["state from {} (search {}: {} guide, {} src, {} dst)"
                       .format(os.path.basename(args.snapshot),
                               snap["searchId"], snap["guides"], snap["srcs"],
                               snap["dsts"]), ""]
    state_notes += ["node state bits, LSB first:", ""]
    state_notes += ["bit {:>2}  {}".format(i, n) for i, n in enumerate(names)]
    write_mem(out("gg_nodes.mem"), nodes, args.node_bits,
              "grid-graph nodes, addr = (z*{} + y)*{} + x".format(y_dim, x_dim),
              node_layout(args.node_bits), state_notes)

    write_mem(out("gg_xcoords.mem"), xc, coord_bits,
              "xCoords in DBU, indexed by x", coord_layout("xCoords[x]", xc))
    write_mem(out("gg_ycoords.mem"), yc, coord_bits,
              "yCoords in DBU, indexed by y", coord_layout("yCoords[y]", yc))
    write_mem(out("gg_zcoords.mem"), zh, coord_bits,
              "zHeights (accumulated z coordinate) in DBU, indexed by z",
              coord_layout("zHeights[z]", zh))

    layer_layout = []
    if layer_bits > minwidth_bits + LAYER_DIR_BITS:
        layer_layout.append((layer_bits - 1, minwidth_bits + LAYER_DIR_BITS,
                             "pad", "zero"))
    layer_layout += [
        (minwidth_bits + LAYER_DIR_BITS - 1, minwidth_bits, "dir",
         "odb::dbTechLayerDir: " + ", ".join(
             "{}={}".format(v, n) for n, v in LAYER_DIR_ENUM)),
        (minwidth_bits - 1, 0, "minWidth",
         "DBU, range {}..{} (needs {}b)".format(
             min(min_widths), max(min_widths), mw_need)),
    ]
    write_mem(out("gg_layers.mem"), layer_words, layer_bits,
              "per-layer direction and minWidth, indexed by z", layer_layout,
              ["the dump's 'layerDir(0=H,1=V,2=NONE)' header is mislabelled;",
               "the values are the raw odb enum, so 1 = HORIZONTAL, 2 = VERTICAL"])

    write_mem(out("via2via_forbidden_len.mem"), v2v_words, forbid_bits,
              "via2ViaForbiddenLen, addr = z*8 + prevViaUp*4 + currViaUp*2 + isDirY",
              forbid_layout(), forbid_notes)
    write_mem(out("via_forbidden_turn_len.mem"), turn_words, forbid_bits,
              "viaForbiddenTurnLen, addr = z*4 + viaUp*2 + isDirY",
              forbid_layout(), forbid_notes)

    write_params(out("gg_params.vh"), header, args.node_bits, coord_bits,
                 forbid_half, forbid_slots, minwidth_bits, layer_bits, snap)


if __name__ == "__main__":
    main()
