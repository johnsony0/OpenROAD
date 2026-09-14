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
        --search    work/ggdump/search_iter1_x136800_y136800_s0.txt \
        --forbidden work/ggdump/forbidden_tables.txt

--search is the text dump of the same search() call.  It is what supplies the
goal box; see THE GOAL BOX below.

python3 initialization/gg_to_mem.py --gg top_gcd/ggdump/gg_iter1_x50400_y50400.txt --snapshot  top_gcd/ggdump/ggs_iter1_x50400_y50400_s3.bin --forbidden top_gcd/ggdump/forbidden_tables.txt

python3 initialization/gg_to_mem.py --gg top_gcd/ggdump/gg_iter1_x50400_y50400.txt --snapshot top_gcd/ggdump/ggs_iter1_x50400_y50400_s3.bin --forbidden top_gcd/ggdump/forbidden_tables.txt

--gg is still required with --snapshot: the binary carries only the mutable
per-node state, while the coordinate arrays, layer directions and cost scalars
are static after init() and are read from the text dump of the same route box.
Match the iter/x/y of the two files -- the script checks dim but cannot check
that you picked the same box at the same iteration.

Word widths are sized to the data, not to a machine word: each is the narrowest
field layout that holds the values, rounded up to a multiple of 4 so one hex
digit is one nibble and $readmemh stays exact.  Every .mem opens with a comment
block giving its bit-by-bit breakdown -- that header is the only description of
the word, so the RTL slices are written against it.

Outputs (all $readmemh-ready: bare hex, one word per line, no prefixes):

    gg_nodes.mem                167*232*6 x 32b   main grid-graph memory
    gg_xcoords.mem              167       x 20b   signed DBU, indexed by x
    gg_ycoords.mem              232       x 20b   signed DBU, indexed by y
    gg_zcoords.mem              6         x 20b   zHeights, DBU, indexed by z
    gg_layers.mem               6         x 16b   {unidir, dir, minWidth}, by z
    via2via_forbidden_len.mem   6*8       x 48b   N x {lo, hi}
    via2via_prl_len.mem         6*8       x 12b   PRL threshold, same addressing
    via_forbidden_turn_len.mem  6*4       x 48b   N x {lo, hi}
    params.sv                   grid dims, goal box and the cost scalars

Two more come out of --search, as plain text rather than memory images -- they
describe one search() call, not the grid it ran on:

    src_dst_pins.txt            its src and dst node sets, maze idx + DBU
    path_cost.txt               the cost it paid, then the path it took


NODE WORD -- 32 bits:

    [31:28]  scratchpad         4b   spare, zero here -- free for the RTL
    [27:25]  prevAstarNodeDir   3b   A* scratch, frDirEnum, 0 = UNKNOWN
    [24]     closed             1b   A* scratch
    [23: 0]  node state        24b   loaded from the dump

Bits 24..27 are working storage for the search, not dump data: both fields are 0
in every word this script emits, because on entry to search() nothing is closed
and every prevAstarNodeDir is UNKNOWN.  They are carved out of the node word so
the RTL can read state and write back its A* bookkeeping in one memory rather
than maintaining a second array in parallel -- which is what the C++ does, in
the separate prevDirs_ bit vector.

Bits 28..31 are the same idea taken one step further: a nibble of uncommitted
per-node storage for whatever else the RTL wants to keep alongside a node (a
visited/queued marker, a small tag, a bucket index).  Nothing in this script or
in the C++ assigns them any meaning, so they ship zeroed.  Widen or narrow the
field with --node-bits; the gg_nodes.mem header follows whatever width you
pick, and 28 (--node-bits 28) removes it entirely.

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


THE GOAL BOX in params.sv is getEstCost's dstMazeIdx1/dstMazeIdx2, the box its
Manhattan heuristic is measured to, written out in DBU as
{LOWER,UPPER}_GOAL_{X,Y,Z}.  Three sources, in falling order of precedence:

    --goal-box x1 y1 z1 x2 y2 z2   maze indices given by hand
    --search   search_iter*.txt    the dstBox row, i.e. what search() used
    --snapshot                     bounding box of the dst bits

--search is the one to use: dumpSearch() records dstMazeIdx1/dstMazeIdx2 as
passed, so no reconstruction is involved.  Match its _s<id> to the snapshot's.

The --snapshot fallback is a reconstruction, exact only while one unconnected
pin is left: routeNet_setDst flags every access pattern of every unconnected
pin and routeNet_setSrc clears them pin by pin, so with several still pending
dsts_ is their union and its box comes out larger than the one search() used.
A --gg-only run has no dst bits at all and stops.

Only the maze indices are read off the dstBox row.  Its DBU columns say the
same thing through the coordinate arrays, and taking those from --gg instead
keeps every coordinate in the run consistent with the .mem images -- and works
on a version 3 search dump, whose third DBU column is the frLayerNum rather
than the zHeight.


THE getEstCost FORBIDDEN CHECK needs four values beyond the range tables, all
of them run-constant and all read from --forbidden (version 2 or newer):

    useNonPrefTracks     USE_NONPREF_TRACKS       scalar
    isUnidirectional[z]  gg_layers.mem unidir bit per z
    BOTTOM_ROUTING_LAYER BOTTOM_ROUTING_LAYER     scalar
    getTopLayerNum()     TOP_ROUTING_LAYER        scalar

The C++ (FlexGridGraph_maze.cpp getEstCost) enters the check only when
`!useNonPrefTracks || isUnidirectional[z]`, then bounds each neighbouring-layer
table lookup with `layerNum - 2 < BOTTOM_ROUTING_LAYER` /
`layerNum + 2 > getTopLayerNum()`, where a bound that is out of range makes its
half of the `&&` vacuously true.  The C++ compares absolute frLayerNum;
params.sv gives both bounds as z instead, since getEstCost hardcodes
`layerNum = (z + 1) * 2` and `layerNum -+ 2` is then just `z -+ 1`.  Each
parameter's comment carries its frLayerNum for tracing back to the LEF.

The top bound is frTechObject::getTopLayerNum(), i.e. layers_.size() - 1 over
*all* layers including cut and masterslice, mapped down to the highest routing
layer at or below it.

isUnidirectional is emitted per layer because it is not derivable from the
preferred direction: frLayer::isUnidirectional() is
`numMasks > 1 || lef58RectOnly || unidirectional_`, none of which the layerDir
column records.
"""

import argparse
import os
import struct
import sys

# params.sv wraps its parameters in a package, so the RTL imports the set by
# name instead of `include-ing the file into every scope that needs a constant.
PARAMS_PACKAGE = "DRT_PARAMS"

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

# Uncommitted per-node storage above the A* scratch, for RTL variables this
# script knows nothing about.  One nibble by default so the node word is a round
# 32 bits; --node-bits resizes it and --node-bits 28 removes it.
SCRATCHPAD_BITS = 4
DEFAULT_NODE_BITS = MIN_NODE_BITS + SCRATCHPAD_BITS  # 32

# frDirEnum, from src/drt/src/frBaseTypes.h.  UNKNOWN = 0 is the reset value.
DIR_ENUM = [("UNKNOWN", 0), ("D", 1), ("S", 2), ("W", 3),
            ("E", 4), ("N", 5), ("U", 6)]

# gg_layers.mem: isUnidirectional over the preferred direction (raw
# odb::dbTechLayerDir, 0..2) over the layer's minWidth.  Named GG_LAYER_DIR_* so
# they cannot collide with the frDirEnum GG_DIR_* above -- these are two
# unrelated encodings.
LAYER_DIR_BITS = 2
LAYER_DIR_ENUM = [("NONE", 0), ("HORIZONTAL", 1), ("VERTICAL", 2)]
# One bit above the direction field.  Separate from dir because
# frLayer::isUnidirectional() is numMasks/rectOnly/unidirectional_, which the
# direction does not encode -- a HORIZONTAL layer may be either.
LAYER_UNIDIR_BITS = 1

# Minimum --forbidden version carrying the getEstCost scalars and the
# per-layer unidirectional column.
FORBIDDEN_MIN_VERSION = 2

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
        rows.append((node_bits - 1, MIN_NODE_BITS, "scratchpad",
                     "spare, 0 here -- RTL-owned, no meaning to this script"))
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
    """Return (numLayers, via2via, turn, prl, scalars).

    via2via and turn are {(z, entry): [(lo, hi)]}.  An entry may hold several
    disjoint ranges -- e.g. metal6 prev-up/curr-up is [0, 600] and [800, 1600],
    with 601..799 legal between them -- so the list is kept as-is and
    pack_forbidden() gives every entry the same number of slots.

    prl is {(z, entry): threshold}, a single frCoord per entry rather than a
    range: frTechObject::isVia2ViaPRL() is `len <= via2ViaPrlLen_[z][entry]`.
    It shares via2ViaForbiddenLen's entry encoding, so one address serves both.

    scalars carries the run-constant getEstCost inputs -- useNonPrefTracks,
    bottomRoutingLayer, topLayerNum -- plus the per-z lists layerNums and
    unidirectional read off the "layer" lines.  These arrived in dump version 2;
    a version 1 file is rejected rather than defaulted, because guessing
    useNonPrefTracks would silently bake a wrong forbidden branch into the RTL
    images instead of failing where it can be seen.
    """
    via2via, turn, prl = {}, {}, {}
    num_layers = None
    version = None
    scalars = {"layerNums": {}, "unidirectional": {}}
    SCALAR_LINES = ("useNonPrefTracks", "bottomRoutingLayer", "topLayerNum")

    with open(path) as f:
        for line in f:
            if not line or line[0] == "#":
                continue
            tok = line.split()
            if not tok:
                continue

            if tok[0] == "version":
                version = int(tok[1])
                continue
            if tok[0] == "numLayers":
                num_layers = int(tok[1])
                continue
            if tok[0] in SCALAR_LINES:
                scalars[tok[0]] = int(tok[1])
                continue
            if tok[0] == "layer":
                # version 1: layer <z> <layerNum> <name>
                # version 2: layer <z> <layerNum> <name> <unidirectional>
                z = int(tok[1])
                scalars["layerNums"][z] = int(tok[2])
                if len(tok) > 4:
                    scalars["unidirectional"][z] = int(tok[4])
                continue
            if tok[0] == "via2ViaPrlLen":
                z, entry, value = int(tok[1]), int(tok[2]), int(tok[3])
                if not 0 <= entry < 8:
                    die("via2ViaPrlLen entry {} out of range 0..7".format(entry))
                prl[(z, entry)] = value
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
    if version is None:
        die("no 'version' line found in " + path)
    if version < FORBIDDEN_MIN_VERSION:
        die("{} is a version {} dump, but the getEstCost forbidden check needs "
            "version {}+ for useNonPrefTracks / bottomRoutingLayer / "
            "topLayerNum and the per-layer unidirectional column.  Re-run the "
            "router with DRT_DUMP_GG_DIR set to regenerate it -- these are not "
            "defaulted, because a wrong useNonPrefTracks silently flips the "
            "forbidden branch.".format(path, version, FORBIDDEN_MIN_VERSION))
    for name in SCALAR_LINES:
        if name not in scalars:
            die("no '{}' line found in {} (version {} dump)".format(
                name, path, version))
    for z in range(num_layers):
        for field in ("layerNums", "unidirectional"):
            if z not in scalars[field]:
                die("{} has no {} for z={}; expected a 'layer' line per "
                    "routing layer 0..{}".format(path, field, z, num_layers - 1))
    return num_layers, via2via, turn, prl, scalars


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


def pack_prl(table, num_layers):
    """Flatten via2ViaPrlLen to one word per (z, entry), same order as via2via.

    The values are half-sums of two via-box extents (FlexRP::prep_via2viaPRL),
    so they cannot be negative and the field is unsigned; an entry whose via
    defs are missing keeps the caller's 0, which makes `len <= prl` false for
    every positive len.
    """
    words = []
    for z in range(num_layers):
        for entry in range(8):
            if (z, entry) not in table:
                die("via2ViaPrlLen missing z={} entry={}".format(z, entry))
            value = table[(z, entry)]
            if value < 0:
                die("via2ViaPrlLen z={} entry={} is negative ({}), which the "
                    "unsigned PRL field cannot hold".format(z, entry, value))
            words.append(value)
    return words


def parse_search(path):
    """Read a dumpSearch() search_iter*.txt -> the fields params.sv needs.

    Only the header and the dstBox row are of interest here; the cc/src/dst/p
    rows repeat state the snapshot already carries.  Returns a dict with dim,
    iter, searchId and the two dstBox corners as maze indices, so the caller
    can check the file against --gg / --snapshot before trusting the box.
    """
    meta = {"srcs": [], "dsts": [], "path": []}
    counts = {}
    rows = {"src": "srcs", "dst": "dsts", "p": "path"}
    with open(path) as f:
        for line in f:
            tok = line.split()
            if not tok or tok[0][0] == "#":
                continue
            if tok[0] == "dim":
                meta["dim"] = (int(tok[1]), int(tok[2]), int(tok[3]))
            elif tok[0] in ("version", "iter", "searchId"):
                meta[tok[0]] = int(tok[1])
            elif tok[0] in ("srcs", "dsts", "path"):
                counts[tok[0]] = int(tok[1])
            elif tok[0] in rows:
                # <x> <y> <z> then the same point in DBU, which is dropped for
                # the same reason as on the dstBox row.
                meta[rows[tok[0]]].append(tuple(int(v) for v in tok[1:4]))
            elif tok[0] == "finalCost":
                # g then f of the node the search stopped on; -1 -1 on failure.
                meta["finalCost"] = (int(tok[1]), int(tok[2]))
            elif tok[0] == "dstBox":
                # <x1> <y1> <z1> <xdbu1> <ydbu1> <zdbu1> then the same for the
                # upper corner.  The maze indices are what gets used -- see the
                # GOAL BOX section of the module docstring -- but the x and y
                # DBU columns are kept to check the file against --gg.
                if len(tok) != 13:
                    die("{} has a dstBox row with {} values, expected 12".format(
                        path, len(tok) - 1))
                meta["dstLo"] = tuple(int(v) for v in tok[1:4])
                meta["dstLoDbu"] = tuple(int(v) for v in tok[4:6])
                meta["dstHi"] = tuple(int(v) for v in tok[7:10])
                meta["dstHiDbu"] = tuple(int(v) for v in tok[10:12])

    for name in ("version", "dim"):
        if name not in meta:
            die("no '{}' line found in {}".format(name, path))
    # Each block states its own length; a mismatch means a truncated file.
    for name, n in counts.items():
        if len(meta[name]) != n:
            die("{} declares {} {} but has {} rows".format(
                path, n, name, len(meta[name])))
    if "dstLo" not in meta:
        die("{} is a version {} search dump with no 'dstBox' row, so it does "
            "not carry the goal box.  Re-run the router with a build that "
            "dumps it, or pass --goal-box.".format(path, meta["version"]))
    return meta


def check_goal_box(lo, hi, dim, what):
    for label, box in (("lower", lo), ("upper", hi)):
        for axis, (v, n) in enumerate(zip(box, dim)):
            if not 0 <= v < n:
                die("{} {} corner index {} on axis {} is outside dim "
                    "{}x{}x{}".format(what, label, v, "xyz"[axis], *dim))
    if any(a > b for a, b in zip(lo, hi)):
        die("{} lower corner {} is not <= upper corner {}".format(
            what, list(lo), list(hi)))


def dst_goal_box(nodes, dim):
    """Maze-index bounding box over the nodes carrying the dst bit.

    Returns ((x1, y1, z1), (x2, y2, z2), count), or (None, None, 0) when no
    node is flagged -- which is every --gg-only run, since the text dump has
    no dst column and a pre-routing snapshot has no dst set anyway.

    This reconstructs getEstCost's dstMazeIdx1/dstMazeIdx2, which no dump
    records directly.  It is exact whenever one unconnected pin is left:
    routeNet_setDst flags every access pattern of every unconnected pin and
    routeNet_setSrc clears a pin's flags as it is absorbed into the connected
    component, so with a single pin remaining dsts_ holds exactly the access
    patterns search() bounds its heuristic to.  With several pins left the box
    is the union over all of them, which is larger than the box search() used
    -- pass --goal-box to override it there.
    """
    x_dim, y_dim, _ = dim
    plane = x_dim * y_dim
    lo = hi = None
    count = 0
    for addr, word in enumerate(nodes):
        if not (word >> DST_BIT) & 1:
            continue
        z, rem = divmod(addr, plane)
        y, x = divmod(rem, x_dim)
        if lo is None:
            lo, hi = [x, y, z], [x, y, z]
        else:
            lo = [min(lo[i], v) for i, v in enumerate((x, y, z))]
            hi = [max(hi[i], v) for i, v in enumerate((x, y, z))]
        count += 1
    return lo, hi, count


def write_src_dst_pins(path, search, dim, xc, yc, zh, source):
    """Emit the search's source and destination node sets, one row per node.

    The srcs/dsts blocks of the search dump, kept in its own file so a consumer
    that only needs the endpoints does not have to walk past the path and the
    header to find them.  The maze indices are copied; the DBU columns are
    re-derived from --gg, so every coordinate this run writes comes from the
    same arrays as gg_{x,y,z}coords.mem.
    """
    # No header or comment lines: the file is exactly the two blocks, so a
    # reader can take the count off line 1 and then read that many rows.
    with open(path, "w") as f:
        for label, pts in (("srcs", search["srcs"]), ("dsts", search["dsts"])):
            f.write("{} {}\n".format(label, len(pts)))
            for x, y, z in pts:
                if any(not 0 <= v < n for v, n in zip((x, y, z), dim)):
                    die("{} has a {} row at {} {} {}, outside dim "
                        "{}x{}x{}".format(source, label[:-1], x, y, z, *dim))
                f.write("{} {} {} {} {} {} {}\n".format(
                    label[:-1], x, y, z, xc[x], yc[y], zh[z]))
    print("  {:<30} {:>8}".format(
        os.path.basename(path),
        "{}+{}".format(len(search["srcs"]), len(search["dsts"]))))


def write_path_cost(path, search, dim, source):
    """Emit what the search routed and what it paid: cost first, then points.

    Line 1 is the total cost of the path -- the pathCost of the node search()
    stopped on, which is every edge charge the A* accumulated getting there.
    Every line after it is one path point as a maze index.

    The points are in the search dump's own order, dst-first back toward src,
    and they are corner points only: traceBackPath() records a node only where
    the direction changes, so consecutive rows differ on a single axis and the
    nodes between them are implied by the run.
    """
    if "finalCost" not in search:
        die("{} has no 'finalCost' row, so the path's cost is not in it.  "
            "Re-run the router with a build that dumps it.".format(source))
    cost = search["finalCost"][0]
    if cost < 0:
        print("  note: {} is a failed search (finalCost -1); writing the cost "
              "through as -1 with no path".format(source))
    with open(path, "w") as f:
        f.write("{}\n".format(cost))
        for x, y, z in search["path"]:
            if any(not 0 <= v < n for v, n in zip((x, y, z), dim)):
                die("{} has a path point at {} {} {}, outside dim "
                    "{}x{}x{}".format(source, x, y, z, *dim))
            f.write("{} {} {}\n".format(x, y, z))
    print("  {:<30} {:>8}".format(
        os.path.basename(path), "{} pts".format(len(search["path"]))))


def write_params(path, header, fb, goal_lo, goal_hi, goal_note):
    """Emit the scalar constants the RTL needs, as plain `parameter` lines.

    Only the run constants live here; every width, bit offset and table layout
    is documented in the header of the .mem file it describes.
    """
    x_dim, y_dim, z_dim = header["dim"]
    xc, yc, zh = header["xCoords"], header["yCoords"], header["zHeights"]
    layer_nums = header["zCoords(layerNum)"]
    if "iter" not in header:
        die("no 'iter' line in the grid-graph dump, so ITER cannot be set")

    def to_z(layer_num, what):
        """frLayerNum -> z.  getTopLayerNum() counts cut and masterslice layers
        too, so it need not be a routing layer's own number; fall back to the
        highest routing layer at or below it."""
        if layer_num in layer_nums:
            return layer_nums.index(layer_num)
        below = [z for z, ln in enumerate(layer_nums) if ln <= layer_num]
        if not below:
            die("{} is frLayerNum {}, which is below every routing layer "
                "{}".format(what, layer_num, layer_nums))
        return below[-1]

    bottom_z = to_z(fb["bottomRoutingLayer"], "bottomRoutingLayer")
    top_z = to_z(fb["topLayerNum"], "topLayerNum")

    def goal(box):
        return xc[box[0]], yc[box[1]], zh[box[2]]

    with open(path, "w") as f:
        f.write("// Generated by initialization/gg_to_mem.py -- do not edit.\n")
        f.write("// Route box (DBU): {} {} {} {}\n".format(*header["routeBox"]))
        f.write("// Goal box (maze idx): {} {} {} .. {} {} {} -- {}\n".format(
            *(list(goal_lo) + list(goal_hi) + [goal_note])))
        f.write("// Widths, bit offsets and table layouts are not here; each\n")
        f.write("// .mem file documents its own word in its header.\n\n")

        f.write("// Coordinates for grid and cost size, "
                "change when grid graph changes\n")
        f.write("package {};\n".format(PARAMS_PACKAGE))
        f.write("  parameter GRID_I = {};\n".format(x_dim))
        f.write("  parameter GRID_J = {};\n".format(y_dim))
        f.write("  parameter GRID_K = {};\n".format(z_dim))
        f.write("  parameter ITER = {};\n\n".format(header["iter"]))

        # DBU, the same units getEstCost measures its Manhattan distance in.
        for bound, box in (("LOWER", goal_lo), ("UPPER", goal_hi)):
            gx, gy, gz = goal(box)
            f.write("  parameter {}_GOAL_X = {};\n".format(bound, gx))
            f.write("  parameter {}_GOAL_Y = {};\n".format(bound, gy))
            f.write("  parameter {}_GOAL_Z = {};\n\n".format(bound, gz))

        f.write("  // Constants for Tritonroute\n")
        for name, key in (("DRC_COST", "ggDRCCost"),
                          ("MARKER_COST", "ggMarkerCost"),
                          ("FIXEDSHAPECOST", "ggFixedShapeCost"),
                          ("GRID_COST", "GRIDCOST"),
                          ("BLOCK_COST", "BLOCKCOST"),
                          ("GUIDE_COST", "GUIDECOST")):
            f.write("  parameter {} = {};\n".format(name, header[key]))
        f.write("\n")

        # Both bounds are z here, not frLayerNum: getEstCost compares
        # layerNum -+ 2 against them, which is the z -+ 1 neighbour under the
        # layerNum = (z + 1) * 2 mapping it hardcodes.
        f.write("  parameter USE_NONPREF_TRACKS = {};\n".format(
            fb["useNonPrefTracks"]))
        f.write("  parameter BOTTOM_ROUTING_LAYER = {}; "
                "//layer_num of {} or layer z of {}\n".format(
                    bottom_z, fb["bottomRoutingLayer"], bottom_z))
        f.write("  parameter TOP_ROUTING_LAYER = {}; "
                "//layer_num of {} or layer z of {}\n".format(
                    top_z, fb["topLayerNum"], top_z))
        f.write("endpackage\n")
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
    ap.add_argument("--node-bits", type=int, default=DEFAULT_NODE_BITS,
                    help="node word width, min {} (default: %(default)s -- the "
                         "{}b state + A* scratch plus a {}b RTL scratchpad in "
                         "the top bits)".format(
                             MIN_NODE_BITS, MIN_NODE_BITS, SCRATCHPAD_BITS))
    ap.add_argument("--coord-bits", type=int, default=None,
                    help="coordinate width; default is derived from the values")
    ap.add_argument("--forbidden-bits", type=int, default=None,
                    help="width of one {lo, hi} pair; default is derived")
    ap.add_argument("--prl-bits", type=int, default=None,
                    help="via2ViaPrlLen word width; default is derived from "
                         "the values")
    ap.add_argument("--minwidth-bits", type=int, default=None,
                    help="minWidth field width in gg_layers.mem; default is "
                         "derived from the values")
    ap.add_argument("--forbidden-slots", type=int, default=None,
                    help="ranges per table entry; default is the most any entry "
                         "in --forbidden needs")
    ap.add_argument("--search", default=None,
                    help="dumpSearch() search_iter*.txt for the same search as "
                         "--snapshot; its dstBox row is the goal box search() "
                         "actually used")
    ap.add_argument("--goal-box", type=int, nargs=6, default=None,
                    metavar=("X1", "Y1", "Z1", "X2", "Y2", "Z2"),
                    help="goal box as two maze indices, lower then upper; "
                         "overrides --search and the --snapshot dst bits")
    ap.add_argument("--ignore-ndr", action="store_true",
                    help="drop nonzero NDR cost fields instead of stopping")
    args = ap.parse_args()

    for p in (args.gg, args.forbidden, args.snapshot, args.search):
        if p is not None and not os.path.isfile(p):
            die("no such file: " + p)
    for name, val in (("--node-bits", args.node_bits),
                      ("--coord-bits", args.coord_bits),
                      ("--minwidth-bits", args.minwidth_bits),
                      ("--prl-bits", args.prl_bits),
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
    search = None
    if args.search is not None:
        search = parse_search(args.search)
        if search["dim"] != header["dim"]:
            die("{} dim {} does not match {} dim {} -- the two files are not "
                "the same route box".format(
                    args.search, search["dim"], args.gg, header["dim"]))
        if "iter" in search and "iter" in header \
                and search["iter"] != header["iter"]:
            die("{} is from DR iteration {} but {} is from {} -- the same route "
                "box is a different grid graph each iteration".format(
                    args.search, search["iter"], args.gg, header["iter"]))
        # The goal box belongs to one search() call, so pairing it with another
        # call's node state would put the heuristic and the grid out of step.
        if snap is not None and "searchId" in search \
                and search["searchId"] != snap["searchId"]:
            die("{} is search {} but {} is search {} -- match the _s<id> "
                "suffixes".format(args.search, search["searchId"],
                                  args.snapshot, snap["searchId"]))
    num_layers, via2via, turn, prl, fb = parse_forbidden(args.forbidden)

    x_dim, y_dim, z_dim = header["dim"]
    xc, yc, zh = header["xCoords"], header["yCoords"], header["zHeights"]
    if (len(xc), len(yc), len(zh)) != (x_dim, y_dim, z_dim):
        die("coordinate array lengths {} do not match dim {}".format(
            (len(xc), len(yc), len(zh)), (x_dim, y_dim, z_dim)))
    if num_layers != z_dim:
        die("forbidden tables have {} layers but the grid graph has {}".format(
            num_layers, z_dim))
    # Both files number routing layers from the bottom one, so their z ->
    # frLayerNum maps must agree.  A mismatch means the two dumps came from
    # different designs, which every table lookup below would silently mis-index.
    fb_layer_nums = [fb["layerNums"][z] for z in range(z_dim)]
    if fb_layer_nums != header["zCoords(layerNum)"]:
        die("layer numbering disagrees: {} has {} but {} has {} -- the two "
            "dumps are not from the same design".format(
                args.forbidden, fb_layer_nums, args.gg,
                header["zCoords(layerNum)"]))

    # --- goal box: what getEstCost measures its heuristic to ---
    if args.goal_box is not None:
        goal_lo, goal_hi = tuple(args.goal_box[:3]), tuple(args.goal_box[3:])
        check_goal_box(goal_lo, goal_hi, header["dim"], "--goal-box")
        goal_note = "from --goal-box"
    elif search is not None:
        goal_lo, goal_hi = search["dstLo"], search["dstHi"]
        where = "the dstBox row of " + os.path.basename(args.search)
        check_goal_box(goal_lo, goal_hi, header["dim"], where)
        # The row states each corner twice, as a maze index and in DBU. Only
        # the index is used, so disagreement means the two files are not the
        # same route box (or the dump predates the current dstBox print) and
        # the index this run trusts is the one that is wrong.
        for label, box, dbu in (("lower", goal_lo, search["dstLoDbu"]),
                                ("upper", goal_hi, search["dstHiDbu"])):
            want = (xc[box[0]], yc[box[1]])
            if want != dbu:
                die("{} disagrees with itself: its {} corner is maze index "
                    "{} {}, which is {} {} DBU in {}, but the row says {} {}"
                    .format(where, label, box[0], box[1], want[0], want[1],
                            os.path.basename(args.gg), dbu[0], dbu[1]))
        goal_note = "dstBox of search {} in {}".format(
            search.get("searchId", "?"), os.path.basename(args.search))
    else:
        goal_lo, goal_hi, n_dst = dst_goal_box(nodes, header["dim"])
        if goal_lo is None:
            die("no node carries the dst bit, so the goal box cannot be "
                "recovered{} -- pass --search for the dumpSearch() text file "
                "of the same search, --snapshot for a dump taken on entry to "
                "search(), or set it by hand with --goal-box".format(
                    " (a --gg text dump has no dst column)"
                    if snap is None else ""))
        goal_note = "bounding box of the {} dst node{} in {}".format(
            n_dst, "" if n_dst == 1 else "s",
            os.path.basename(args.snapshot))

    # --- coordinate width: one signed width covering all three axes ---
    all_coords = xc + yc + zh
    need = signed_bits(min(all_coords), max(all_coords))
    coord_bits = args.coord_bits if args.coord_bits is not None else nibble_align(need)
    if coord_bits < need:
        die("--coord-bits {} cannot hold the range {}..{} (needs {} signed)".format(
            coord_bits, min(all_coords), max(all_coords), need))

    # --- per-layer word: {unidir, dir, minWidth}, minWidth sized from the
    # values.  dir comes from the grid-graph dump, unidir from the forbidden
    # dump -- the two files describe the same layer stack, checked above.
    dirs = header["layerDir(0=H,1=V,2=NONE)"]
    min_widths = header["layerMinWidth"]
    unidirs = [fb["unidirectional"][z] for z in range(z_dim)]
    if len(dirs) != z_dim or len(min_widths) != z_dim:
        die("layerDir/layerMinWidth have {}/{} entries, expected {}".format(
            len(dirs), len(min_widths), z_dim))
    for z, d in enumerate(dirs):
        if not 0 <= d < (1 << LAYER_DIR_BITS):
            die("layerDir[{}] = {} does not fit the {}-bit direction field".format(
                z, d, LAYER_DIR_BITS))
    for z, u in enumerate(unidirs):
        if u not in (0, 1):
            die("unidirectional[{}] = {} is not a 0/1 flag".format(z, u))
    if min(min_widths) < 0:
        die("layerMinWidth has a negative entry: {}".format(min(min_widths)))
    mw_need = unsigned_bits(max(min_widths))
    minwidth_bits = (args.minwidth_bits if args.minwidth_bits is not None
                     else nibble_align(mw_need))
    if minwidth_bits < mw_need:
        die("--minwidth-bits {} cannot hold the max minWidth {} (needs {})".format(
            minwidth_bits, max(min_widths), mw_need))
    unidir_lsb = minwidth_bits + LAYER_DIR_BITS
    layer_bits = nibble_align(unidir_lsb + LAYER_UNIDIR_BITS)
    layer_words = [(u << unidir_lsb) | (d << minwidth_bits) | w
                   for u, d, w in zip(unidirs, dirs, min_widths)]

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

    # --- PRL width: unsigned, one threshold per via2ViaForbiddenLen entry ---
    prl_words = pack_prl(prl, num_layers)
    prl_need = unsigned_bits(max(prl_words))
    prl_bits = args.prl_bits if args.prl_bits is not None else nibble_align(prl_need)
    if prl_bits < prl_need:
        die("--prl-bits {} cannot hold the max via2ViaPrlLen {} (needs {})".format(
            prl_bits, max(prl_words), prl_need))

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
    state_notes += ["", "prevAstarNodeDir is frDirEnum: " + ", ".join(
        "{}={}".format(v, n) for n, v in DIR_ENUM)]
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
    if layer_bits > unidir_lsb + LAYER_UNIDIR_BITS:
        layer_layout.append((layer_bits - 1, unidir_lsb + LAYER_UNIDIR_BITS,
                             "pad", "zero"))
    layer_layout += [
        (unidir_lsb + LAYER_UNIDIR_BITS - 1, unidir_lsb, "unidir",
         "frLayer::isUnidirectional(), from " + os.path.basename(args.forbidden)),
        (unidir_lsb - 1, minwidth_bits, "dir",
         "odb::dbTechLayerDir: " + ", ".join(
             "{}={}".format(v, n) for n, v in LAYER_DIR_ENUM)),
        (minwidth_bits - 1, 0, "minWidth",
         "DBU, range {}..{} (needs {}b)".format(
             min(min_widths), max(min_widths), mw_need)),
    ]
    write_mem(out("gg_layers.mem"), layer_words, layer_bits,
              "per-layer unidir, direction and minWidth, indexed by z",
              layer_layout,
              ["the dump's 'layerDir(0=H,1=V,2=NONE)' header is mislabelled;",
               "the values are the raw odb enum, so 1 = HORIZONTAL, 2 = VERTICAL",
               "",
               "per-z dir      " + " ".join(
                   DIR_NAMES.get(d, "?")[:1] or "?" for d in dirs),
               "per-z minWidth " + " ".join(str(w) for w in min_widths),
               "per-z unidir   " + " ".join(str(u) for u in unidirs),
               "",
               "unidir is not a function of dir -- it is numMasks > 1 ||",
               "lef58RectOnly || unidirectional_, so a HORIZONTAL layer may be",
               "either.  getEstCost takes its forbidden branch when",
               "!GG_USE_NONPREF_TRACKS || unidir[z]."])

    write_mem(out("via2via_forbidden_len.mem"), v2v_words, forbid_bits,
              "via2ViaForbiddenLen, addr = z*8 + prevViaUp*4 + currViaUp*2 + isDirY",
              forbid_layout(), forbid_notes)
    write_mem(out("via2via_prl_len.mem"), prl_words, prl_bits,
              "via2ViaPrlLen, addr = z*8 + prevViaUp*4 + currViaUp*2 + isDirY",
              [(prl_bits - 1, 0, "prl", "unsigned DBU, range {}..{} (needs {}b)"
                .format(min(prl_words), max(prl_words), prl_need))],
              ["a threshold, not a range: isVia2ViaPRL() is len <= prl",
               "same address as via2via_forbidden_len.mem -- one generator",
               "feeds both",
               "",
               "read only when the via2via check has both currVLengthX and",
               "currVLengthY nonzero, where it picks OR (either axis passes",
               "the PRL test) or AND (neither does) over the two",
               "isVia2ViaForbiddenLen results; a single-axis move never",
               "consults it",
               "",
               "no NDR variant exists -- isVia2ViaPRL() always reads the tech",
               "table, even for a net with a non-default rule"])

    write_mem(out("via_forbidden_turn_len.mem"), turn_words, forbid_bits,
              "viaForbiddenTurnLen, addr = z*4 + viaUp*2 + isDirY",
              forbid_layout(), forbid_notes)

    if search is not None:
        src_name = os.path.basename(args.search)
        write_src_dst_pins(out("src_dst_pins.txt"), search, header["dim"],
                           xc, yc, zh, src_name)
        write_path_cost(out("path_cost.txt"), search, header["dim"], src_name)

    write_params(out("params.sv"), header, fb, goal_lo, goal_hi, goal_note)


if __name__ == "__main__":
    main()