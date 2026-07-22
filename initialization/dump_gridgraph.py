# dump_gridgraph.py
#
# gdb-Python script that dumps ONE route box's full FlexGridGraph node data at
# runtime, WITHOUT modifying/rebuilding OpenROAD source. It reads the grid
# graph's members directly out of inferior memory (it does NOT call printNode /
# getIdx, so it works even in optimized builds where those inline functions were
# never emitted). Output format is identical to the C++ FlexGridGraph::dumpGridGraph
# dumper, so the same parser handles both.
#
# REQUIREMENT: the openroad binary must have debug symbols (a Debug or
# RelWithDebInfo build, or an unstripped package). Verify first:
#     file $(which openroad)                      # want "not stripped"
#     gdb -q openroad -ex 'b drt::FlexGridGraph::search' -ex quit
#         -> must resolve to FlexGridGraph_maze.cpp:<line>
# If it is stripped/optimized-out, you need a debug build (which is itself a
# rebuild) -- at which point the source dumper is the easier path.
#
# ---------------------------------------------------------------------------
# USAGE (recommended: the tiny single-net top_level_term design)
#
#   cd src/drt/test
#   gdb -q -batch \
#       -ex "set pagination off" \
#       -x ../../../initialization/dump_gridgraph.py \
#       -ex run \
#       --args openroad -no_init -exit top_level_term.tcl
#
#   -> writes the dump to $GG_DUMP_OUT (default: ./gridgraph_dump.txt)
#
# Prefer a SINGLE-THREADED run so the breakpoint doesn't fire on many OpenMP
# worker threads at once. Add `set_thread_count 1` near the top of the .tcl
# (before detailed_route), or the results are still fine but noisier.
#
# ENV VARS (all optional):
#   GG_DUMP_OUT   output file path             (default ./gridgraph_dump.txt)
#   GG_DUMP_BOX   "<xlo> <ylo>" of the route box to dump; if unset, dumps the
#                 FIRST search() hit and then stops dumping.
#   GG_DUMP_BP    breakpoint function; default "drt::FlexGridGraph::search"
#                 (state exactly as A* sees it). Use
#                 "drt::FlexGridGraph::init" for the pristine pre-routing graph
#                 -- but note `this` is valid there only after the graph is
#                 built, so search() is the reliable capture point.
# ---------------------------------------------------------------------------

import os
import gdb

OUT_PATH = os.environ.get("GG_DUMP_OUT", "gridgraph_dump.txt")
BP_FUNC = os.environ.get("GG_DUMP_BP", "drt::FlexGridGraph::search")
_want_box = os.environ.get("GG_DUMP_BOX")
TARGET_BOX = None
if _want_box:
    parts = _want_box.split()
    if len(parts) == 2:
        TARGET_BOX = (int(parts[0]), int(parts[1]))

HORIZONTAL = 1  # odb::dbTechLayerDir::Value { NONE=0, HORIZONTAL=1, VERTICAL=2 }


# ---- helpers to read libstdc++ containers straight from memory --------------
def strip_ref(v):
    """Normalize a reference/pointer-to-object into the object Value."""
    t = v.type.strip_typedefs()
    if t.code in (gdb.TYPE_CODE_REF, getattr(gdb, "TYPE_CODE_RVALUE_REF", -99)):
        return v.referenced_value()
    if t.code == gdb.TYPE_CODE_PTR:
        return v.dereference()
    return v


def vec_ptr_size(v):
    """(start_ptr_Value, size) for a std::vector<T> gdb.Value."""
    v = strip_ref(v)
    impl = v["_M_impl"]
    start = impl["_M_start"]
    finish = impl["_M_finish"]
    size = int(finish - start)
    return start, size


def vec_to_list(v):
    start, n = vec_ptr_size(v)
    return [start[i] for i in range(n)]


def vec_int_list(v):
    start, n = vec_ptr_size(v)
    return [int(start[i]) for i in range(n)]


def vbool_at(vb, i):
    """Read element i of a std::vector<bool> (bit-packed libstdc++ layout)."""
    vb = strip_ref(vb)
    start = vb["_M_impl"]["_M_start"]
    p = start["_M_p"]
    off = int(start["_M_offset"])
    bits = 8 * p.dereference().type.sizeof
    total = off + i
    word = int(p[total // bits])
    return (word >> (total % bits)) & 1


# Node bitfield columns, in the same order the C++ dumper prints them.
NODE_FIELDS = [
    "hasEastEdge", "hasNorthEdge", "hasUpEdge",
    "isBlockedEast", "isBlockedNorth", "isBlockedUp",
    "hasSpecialVia",
    "hasGridCostEast", "hasGridCostNorth", "hasGridCostUp",
    "hasApCostEast", "hasApCostNorth", "hasApCostUp",
    "routeShapeCostPlanar", "routeShapeCostVia",
    "markerCostPlanar", "markerCostVia",
    "fixedShapeCostVia", "fixedShapeCostPlanarHorz", "fixedShapeCostPlanarVert",
    "routeShapeCostPlanarNDR", "routeShapeCostViaNDR",
    "fixedShapeCostViaNDR", "fixedShapeCostPlanarHorzNDR",
    "fixedShapeCostPlanarVertNDR",
]


def compute_idx(x, y, z, xsize, ysize, zdirs):
    # Mirrors FlexGridGraph::getIdx().
    if zdirs[z] == HORIZONTAL:
        zmod = x + y * xsize
    else:
        zmod = y + x * ysize
    return zmod + z * xsize * ysize


def dump(gg, out_path):
    gg = strip_ref(gg)  # `this` is FlexGridGraph*; work on the object
    worker = gg["drWorker_"]
    rb = worker.dereference()["routeBox_"]
    xlo, ylo = int(rb["xlo_"]), int(rb["ylo_"])
    xhi, yhi = int(rb["xhi_"]), int(rb["yhi_"])
    it = int(worker.dereference()["drIter_"])

    xcoords = vec_int_list(gg["xCoords_"])
    ycoords = vec_int_list(gg["yCoords_"])
    zcoords = vec_int_list(gg["zCoords_"])
    zheights = vec_int_list(gg["zHeights_"])
    zdirs = [int(d["value_"]) for d in vec_to_list(gg["layerRouteDirections_"])]

    xdim, ydim, zdim = len(xcoords), len(ycoords), len(zcoords)
    if xdim == 0 or ydim == 0 or zdim == 0:
        print("[dump_gridgraph] grid not built yet (empty dims); skipping")
        return False

    nodes_start, _ = vec_ptr_size(gg["nodes_"])
    guides = gg["guides_"]

    with open(out_path, "w") as os_:
        os_.write("version 1\n")
        os_.write("iter %d\n" % it)
        os_.write("routeBox %d %d %d %d\n" % (xlo, ylo, xhi, yhi))
        os_.write("ggDRCCost %d\n" % int(gg["ggDRCCost_"]))
        os_.write("ggMarkerCost %d\n" % int(gg["ggMarkerCost_"]))
        os_.write("ggFixedShapeCost %d\n" % int(gg["ggFixedShapeCost_"]))
        os_.write("dim %d %d %d\n" % (xdim, ydim, zdim))
        os_.write("xCoords " + " ".join(map(str, xcoords)) + "\n")
        os_.write("yCoords " + " ".join(map(str, ycoords)) + "\n")
        os_.write("zCoords(layerNum) " + " ".join(map(str, zcoords)) + "\n")
        os_.write("zHeights " + " ".join(map(str, zheights)) + "\n")
        os_.write("layerDir(0=H,1=V,2=NONE) " + " ".join(map(str, zdirs)) + "\n")

        # Seeds available at search() entry (comment lines: ignored by a node
        # parser, but give you the A* source points for this net).
        try:
            frame = gdb.selected_frame()
            cc = frame.read_var("connComps")
            start, n = vec_ptr_size(cc)
            for i in range(n):
                mi = start[i]
                os_.write("# src %d %d %d\n"
                          % (int(mi["xIdx_"]), int(mi["yIdx_"]), int(mi["zIdx_"])))
        except Exception as e:  # noqa: BLE001 - seeds are best-effort
            os_.write("# (seeds unavailable: %s)\n" % e)

        os_.write("# node x y z " + " ".join(
            ["eE", "eN", "eU", "bE", "bN", "bU", "sVia",
             "gcE", "gcN", "gcU", "apE", "apN", "apU",
             "rsPlanar", "rsVia", "mkPlanar", "mkVia",
             "fxVia", "fxPlanarH", "fxPlanarV",
             "rsPlanarNDR", "rsViaNDR", "fxViaNDR",
             "fxPlanarHNDR", "fxPlanarVNDR", "guide"]) + "\n")

        for z in range(zdim):
            for y in range(ydim):
                for x in range(xdim):
                    idx = compute_idx(x, y, z, xdim, ydim, zdirs)
                    node = nodes_start[idx]
                    vals = [int(node[f]) for f in NODE_FIELDS]
                    try:
                        g = vbool_at(guides, idx)
                    except Exception:  # noqa: BLE001
                        g = -1
                    os_.write("node %d %d %d %s %d\n"
                              % (x, y, z, " ".join(map(str, vals)), g))

    print("[dump_gridgraph] wrote route box (%d,%d)-(%d,%d) iter %d -> %s"
          % (xlo, ylo, xhi, yhi, it, os.path.abspath(out_path)))
    return True


class GridGraphDump(gdb.Breakpoint):
    def __init__(self):
        super().__init__(BP_FUNC, gdb.BP_BREAKPOINT, internal=False)
        self.done = False

    def stop(self):
        # Return False => auto-continue (program runs to completion, DEF still
        # gets written). We do all the work here, then stop dumping.
        if self.done:
            return False
        try:
            gg = strip_ref(gdb.parse_and_eval("this"))
            worker = gg["drWorker_"]
            rb = worker.dereference()["routeBox_"]
            xlo, ylo = int(rb["xlo_"]), int(rb["ylo_"])
            if TARGET_BOX is not None and (xlo, ylo) != TARGET_BOX:
                return False  # not the box we want; keep going
            if dump(gg, OUT_PATH):
                self.done = True
                self.enabled = False
        except Exception as e:  # noqa: BLE001
            print("[dump_gridgraph] ERROR: %s" % e)
        return False


GridGraphDump()
print("[dump_gridgraph] armed breakpoint at %s; output -> %s%s"
      % (BP_FUNC, OUT_PATH,
         "" if TARGET_BOX is None else "  (box filter %s)" % (TARGET_BOX,)))
