// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "dr/FlexMazeTypes.h"
#include "odb/geom.h"

namespace drt {

class drNet;
class frDesign;
class frMarker;

// A* hardware trial: every routed net is first routed with paths from the A*
// accelerator (a VCS simulation driven through DRT_ASTAR_HW_CMD), measured,
// and then undone, after which the normal CPU route runs and is kept. The
// committed routing is therefore exactly that of a CPU-only run; the trial
// only adds CSV rows comparing the two routes of each net.
//
// Environment:
//   DRT_ASTAR_HW_OUT        output directory; the trial is off when unset
//   DRT_ASTAR_HW_MODE       "trial" (default) or "record": record only writes
//                           iters.csv timing/quality rows of a CPU-only run,
//                           the baseline astar_hw_score.py --baseline reads
//   DRT_ASTAR_HW_CMD        job command, called as
//                             <cmd> --job D --prebuilt
//                           (or, with DRT_ASTAR_HW_CONVERT=python,
//                             <cmd> --job D --gg G --snapshot S --search T
//                                   --forbidden F)
//                           and expected to write D/hw_result.txt
//                           (a-star-accelerator/tests/hw_job.py)
//   DRT_ASTAR_HW_CONVERT    how a job gets its accelerator inputs:
//                           "cpp" (default) FlexGridGraph::writeHwMem() writes
//                           the $readmemh images, params.sv, search_cfg.txt
//                           and src_dst_pins.txt straight into the job;
//                           "python" writes the gg.txt / search dumps and the
//                           job converts them with gg_to_mem.py;
//                           "both" runs the job on the cpp images and also
//                           writes the dumps, to check one against the other
//   DRT_ASTAR_HW_CLK_MHZ    accelerator clock for hw_ms (default 100)
//   DRT_ASTAR_HW_NET_PROB   fraction of routed nets trialed (default 1)
//   DRT_ASTAR_HW_MIN_ITER / DRT_ASTAR_HW_MAX_ITER  iteration window
//   DRT_ASTAR_HW_SEED       seed of the net sampling hash (default 1)
//   DRT_ASTAR_HW_KEEP_JOBS  0 delete job dirs, 1 keep mismatching, 2 keep all
//   DRT_ASTAR_HW_RUN_ID     run_id column (default: start timestamp)
//
// Outputs in DRT_ASTAR_HW_OUT: searches.csv, nets.csv, iters.csv (one row per
// DR iteration), forbidden_tables.txt and jobs/ (per-box scratch).
namespace astar_hw {

struct Config
{
  bool enabled = false;
  bool recordOnly = false;
  std::string outDir;
  std::string cmd;
  double clkMhz = 100.0;
  double netProb = 1.0;
  int minIter = 0;
  int maxIter = std::numeric_limits<int>::max();
  uint64_t seed = 1;
  int keepJobs = 0;
  std::string runId;
  bool writeMem = true;     // cpp images, job runs --prebuilt
  bool writeDumps = false;  // gg.txt / search.bin / search.txt
};

const Config& config();
// Iteration is recorded in iters.csv.
bool iterEnabled(int iter);
// Nets of this iteration are trial-routed on the accelerator.
bool trialEnabled(int iter);
// Scratch directory of one route box, holding its gg.txt and search jobs.
std::string boxDir(int iter, const odb::Rect& routeBox);
std::string forbiddenTablesPath();
bool sampleNet(int iter,
               const odb::Rect& routeBox,
               const std::string& netName,
               int numReroutes);

struct HwResult
{
  std::string status = "not_run";
  int64_t cycles = -1;
  int64_t nodes = -1;
  int64_t cost = -1;
  int64_t wirelength = -1;
  int64_t forwardCycles = -1;
  int64_t evalCycles = -1;
  int64_t evalStallCycles = -1;
  int64_t searchCycles = -1;
  double convertS = 0;
  double compileS = 0;
  double simS = 0;
  double jobS = 0;
  // Every node of the path, destination first, as maze indices.
  std::vector<FlexMazeIdx> path;
};

// Runs one job synchronously and parses <jobDir>/hw_result.txt. With
// config().writeMem the job's inputs are already in memDir(jobDir) and the
// dump paths are unused.
HwResult runJob(const std::string& jobDir,
                const std::string& ggPath,
                const std::string& snapshotPath,
                const std::string& searchPath);
// Where the simulator reads its inputs, relative to the job directory it runs
// in (typedef.sv: a-star-accelerator/tests/<TESTBENCH_NAME>).
std::string memDir(const std::string& jobDir);

// The accelerator's forbidden-length tables, packed once per run by FlexRP
// from the tech (the gg_to_mem.py layout; see writeHwMem()). Each image is the
// .mem data, one hex word per line, numLayers * 4 words deep.
struct TechImages
{
  bool valid = false;
  int numLayers = 0;
  std::vector<int> layerNums;    // frLayerNum of each z
  int forbidHalf = 0;            // bits of one lo or hi bound
  int prlBits = 0;               // bits of one via2ViaPrlLen threshold
  std::string via2viaForbidden;  // 8 * forbidHalf bits per word
  std::string via2viaPrl;        // 2 * prlBits
  std::string viaForbiddenTurn;  // 4 * forbidHalf
};
void setTechImages(TechImages images);
const TechImages& techImages();

// $readmemh word helpers. Every field is a multiple of 4 bits, so a word is
// its fields' hex digits side by side, MSB first.
int nibbleAlign(int bits);
// Narrowest two's-complement width holding every value in [lo, hi].
int signedBits(int64_t lo, int64_t hi);
int unsignedBits(uint64_t hi);
// Appends the low `bits` bits of value as bits / 4 hex digits.
void appendHex(std::string& out, uint64_t value, int bits);

// ISPD18 contest scoring categories, measured on routed geometry. Lengths and
// areas are DBU / DBU^2; astar_hw_score.py applies the contest weights.
struct Quality
{
  int64_t wlDbu = 0;
  int64_t vias = 0;
  int64_t offGuideWireDbu = 0;
  int64_t offGuideVias = 0;
  int64_t offTrackWireDbu = 0;
  int64_t offTrackVias = 0;
  int64_t wrongWayDbu = 0;
  int64_t patches = 0;
  int64_t markers = 0;
  int64_t shorts = 0;
  double shortArea = 0;
  int64_t minArea = 0;
  int64_t spacing = 0;
  int64_t otherDrc = 0;

  void add(const Quality& o);
  void addMarker(const frMarker& marker);
};

// Geometry categories of one worker net (pathsegs, vias, patches).
Quality measureNet(const frDesign* design, const drNet* net);
// Geometry and DRC categories of the whole committed design.
Quality measureDesign(const frDesign* design);

struct SearchRow
{
  int iter = 0;
  odb::Rect box;
  std::string net;
  int reroute = 0;
  int searchIdx = 0;
  int dimX = 0, dimY = 0, dimZ = 0;
  bool cpuOk = false;
  int64_t cpuCost = -1;
  int64_t cpuNodes = -1;
  int64_t cpuWlDbu = 0;
  int64_t cpuVias = 0;
  double cpuMs = 0;
  HwResult hw;
  int64_t hwWlDbu = 0;
  int64_t hwVias = 0;
  bool hwUsed = false;  // hw path valid and used to continue the trial
  bool pathMatch = false;
  bool costMatch = false;
};

struct NetRow
{
  int iter = 0;
  odb::Rect box;
  std::string net;
  int reroute = 0;
  std::string status;  // ok / partial / unsupported_ndr / ...
  int searches = 0;
  int hwSearchesUsed = 0;
  int pathMatches = 0;
  bool hwRouted = false;
  Quality cpu;
  Quality hw;
  double cpuAstarMs = 0;  // CPU A* on the hardware trial's inputs
  double hwMs = 0;
  int64_t hwCycles = 0;
  double trialMs = 0;
};

void recordSearch(const SearchRow& row);
void recordNet(const NetRow& row);
void recordWorker(int iter,
                  double cpuRouteAstarMs,
                  int cpuRouteSearches,
                  double trialMs);
// Writes the iters.csv row; called once per DR iteration by FlexDR.
void endIteration(const frDesign* design,
                  int iter,
                  int threads,
                  double iterWallMs);

}  // namespace astar_hw
}  // namespace drt
