// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// A* hardware trial on the worker side: route a net with accelerator paths,
// measure it, undo it. See AstarHwTrial.h for the overall flow.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "db/drObj/drNet.h"
#include "db/drObj/drPin.h"
#include "db/obj/frNet.h"
#include "dr/AstarHwTrial.h"
#include "dr/FlexDR.h"
#include "dr/FlexGridGraph.h"
#include "dr/FlexMazeTypes.h"
#include "frBaseTypes.h"
#include "gc/FlexGC.h"
#include "odb/geom.h"

namespace drt {

namespace {

// Direction of the unit step a -> b, or UNKNOWN if the two nodes are not
// neighbors.
frDirEnum stepDir(const FlexMazeIdx& a, const FlexMazeIdx& b)
{
  const int dx = b.x() - a.x();
  const int dy = b.y() - a.y();
  const int dz = b.z() - a.z();
  if (std::abs(dx) + std::abs(dy) + std::abs(dz) != 1) {
    return frDirEnum::UNKNOWN;
  }
  if (dx != 0) {
    return dx > 0 ? frDirEnum::E : frDirEnum::W;
  }
  if (dy != 0) {
    return dy > 0 ? frDirEnum::N : frDirEnum::S;
  }
  return dz > 0 ? frDirEnum::U : frDirEnum::D;
}

// Every node of a corner-point path, in the same order.
std::vector<FlexMazeIdx> expandPath(const std::vector<FlexMazeIdx>& corners)
{
  std::vector<FlexMazeIdx> nodes;
  for (size_t i = 0; i < corners.size(); ++i) {
    if (i == 0) {
      nodes.push_back(corners[0]);
      continue;
    }
    FlexMazeIdx cur = corners[i - 1];
    const FlexMazeIdx& to = corners[i];
    while (!(cur == to)) {
      cur.set(cur.x() + (to.x() > cur.x()) - (to.x() < cur.x()),
              cur.y() + (to.y() > cur.y()) - (to.y() < cur.y()),
              cur.z() + (to.z() > cur.z()) - (to.z() < cur.z()));
      nodes.push_back(cur);
    }
  }
  return nodes;
}

}  // namespace

void FlexDRWorker::routeNet_hwTrial(drNet* net)
{
  const auto start = std::chrono::steady_clock::now();
  HwTrialState trial;
  astar_hw::NetRow& row = trial.row;
  row.iter = getDRIter();
  row.box = getRouteBox();
  row.net = net->getFrNet()->getName();
  row.reroute = net->getNumReroutes();
  trial.boxDir = astar_hw::boxDir(row.iter, row.box);
  trial.ggPath = trial.boxDir + "/gg.txt";
  trial.netTag
      = "n" + std::to_string(net->getId()) + "_r" + std::to_string(row.reroute);
  // Jumper nets are supported: dumpSearch() records route_with_jumpers and the
  // accelerator takes it at runtime (search_cfg.txt jumper_route).
  if (net->getFrNet()->getNondefaultRule() != nullptr) {
    trial.unsupported = "unsupported_ndr";
  }

  // The net was just ripped up and cleared by route_queue_main(). Route it
  // the normal way, with routeNet_hwSearch() standing in for the A*.
  FlexGridGraph::SavedState saved;
  gridGraph_.saveState(saved);
  hwTrial_ = &trial;
  mazeNetInit(net);
  std::vector<FlexMazeIdx> paths;
  row.hwRouted = routeNet(net, paths);
  mazeNetEnd(net);
  hwTrial_ = nullptr;

  // Same check route_queue_main() runs on the CPU route, minus writing the
  // gc patches back.
  if (gcWorker_->setTargetNet(net)) {
    gcWorker_->updateDRNet(net);
    gcWorker_->setEnableSurgicalFix(true);
    gcWorker_->main();
    row.hw = astar_hw::measureNet(getDesign(), net);
    for (const auto& marker : gcWorker_->getMarkers()) {
      row.hw.addMarker(*marker);
    }
    gcWorker_->clearPWires();
  }

  // Undo: drop the trial geometry and put every grid-graph cost back. The
  // CPU route's own updateDRNet() resyncs the gc worker.
  auto& workerRegionQuery = getWorkerRegionQuery();
  for (auto& connFig : net->getRouteConnFigs()) {
    workerRegionQuery.remove(connFig.get());
  }
  net->clear();
  gridGraph_.restoreState(saved);

  if (!trial.unsupported.empty()) {
    row.status = trial.unsupported;
  } else if (row.searches == 0) {
    row.status = "no_search";
  } else if (row.pqFullSearches > 0) {
    row.status = "pq_full";
  } else if (!row.hwRouted) {
    row.status = "no_route";
  } else if (row.hwSearchesUsed == row.searches) {
    row.status = "ok";
  } else if (row.hwSearchesUsed > 0) {
    row.status = "partial";
  } else {
    row.status = "cpu_only";
  }
  row.trialMs = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - start)
                    .count();
  hwTrialMs_ += row.trialMs;
  hwPendingNet_ = std::make_unique<astar_hw::NetRow>(row);
}

bool FlexDRWorker::routeNet_hwSearch(
    std::vector<FlexMazeIdx>& connComps,
    drPin* nextPin,
    std::vector<FlexMazeIdx>& path,
    FlexMazeIdx& ccMazeIdx1,
    FlexMazeIdx& ccMazeIdx2,
    const odb::Point& centerPt,
    std::map<FlexMazeIdx, frBox3D*>& mazeIdx2TaperBox,
    bool route_with_jumpers)
{
  HwTrialState& trial = *hwTrial_;
  astar_hw::NetRow& netRow = trial.row;
  astar_hw::SearchRow row;
  row.iter = netRow.iter;
  row.box = netRow.box;
  row.net = netRow.net;
  row.reroute = netRow.reroute;
  row.searchIdx = netRow.searches++;
  frMIdx xDim, yDim, zDim;
  gridGraph_.getDim(xDim, yDim, zDim);
  row.dimX = xDim;
  row.dimY = yDim;
  row.dimZ = zDim;
  const std::string jobDir = trial.boxDir + "/" + trial.netTag + "_s"
                             + std::to_string(row.searchIdx);
  std::filesystem::create_directories(jobDir);
  const std::string prefix = jobDir + "/search";

  // CPU A* on exactly the input the accelerator gets. search() writes its
  // outputs into connComps/ccMazeIdx*, so it runs on copies; before the A*
  // starts it also writes the job's inputs (the accelerator's memory images
  // and/or the dumps gg_to_mem.py converts, see DRT_ASTAR_HW_CONVERT).
  std::vector<FlexMazeIdx> cpuConnComps = connComps;
  FlexMazeIdx cpuCc1 = ccMazeIdx1;
  FlexMazeIdx cpuCc2 = ccMazeIdx2;
  std::vector<FlexMazeIdx> cpuPath;
  double cpuMs = 0.0;
  const auto& cfg = astar_hw::config();
  const bool writeMem = cfg.writeMem && trial.unsupported.empty();
  if (cfg.writeDumps) {
    gridGraph_.setHwExportPrefix(prefix);
  }
  if (writeMem) {
    gridGraph_.setHwMemDir(astar_hw::memDir(jobDir));
  }
  row.cpuOk = gridGraph_.search(cpuConnComps,
                                nextPin,
                                cpuPath,
                                cpuCc1,
                                cpuCc2,
                                centerPt,
                                mazeIdx2TaperBox,
                                route_with_jumpers,
                                cpuMs);
  gridGraph_.setHwExportPrefix("");
  gridGraph_.setHwMemDir("");
  row.cpuMs = cpuMs;
  row.cpuCost = row.cpuOk ? gridGraph_.getLastPathCost() : -1;
  row.cpuNodes = gridGraph_.getLastExploredNodes();

  auto pathLength = [this](const std::vector<FlexMazeIdx>& corners,
                           int64_t& wl,
                           int64_t& vias) {
    for (size_t i = 1; i < corners.size(); ++i) {
      wl += std::abs(gridGraph_.xCoord(corners[i].x())
                     - gridGraph_.xCoord(corners[i - 1].x()))
            + std::abs(gridGraph_.yCoord(corners[i].y())
                       - gridGraph_.yCoord(corners[i - 1].y()));
      vias += std::abs(corners[i].z() - corners[i - 1].z());
    }
  };
  pathLength(cpuPath, row.cpuWlDbu, row.cpuVias);

  if (!trial.unsupported.empty()) {
    row.hw.status = trial.unsupported;
  } else if (writeMem && !gridGraph_.getLastHwMemError().empty()) {
    row.hw.status = "mem_" + gridGraph_.getLastHwMemError();
  } else {
    row.hw = astar_hw::runJob(
        jobDir, trial.ggPath, prefix + ".bin", prefix + ".txt");
    if (writeMem) {
      // The job skipped conversion; its input was written here instead.
      row.hw.convertS = gridGraph_.getLastHwMemSeconds();
    }
  }

  // Check the accelerator path the way traceBackPath() would have produced
  // it: destination first, unit steps along existing edges, ending on a
  // source. Compress it to the corner points traceBackPath() returns.
  const std::vector<FlexMazeIdx>& nodes = row.hw.path;
  std::vector<FlexMazeIdx> corners;
  bool valid = row.hw.status == "solved" && !nodes.empty();
  for (const FlexMazeIdx& n : nodes) {
    valid = valid && n.x() >= 0 && n.x() < xDim && n.y() >= 0 && n.y() < yDim
            && n.z() >= 0 && n.z() < zDim;
  }
  valid = valid && gridGraph_.isDstNode(nodes.front())
          && gridGraph_.isSrcNode(nodes.back());
  for (size_t i = 1; valid && i < nodes.size(); ++i) {
    const frDirEnum dir = stepDir(nodes[i - 1], nodes[i]);
    valid = dir != frDirEnum::UNKNOWN
            && gridGraph_.hasEdge(
                nodes[i - 1].x(), nodes[i - 1].y(), nodes[i - 1].z(), dir);
  }
  if (valid) {
    corners.push_back(nodes.front());
    for (size_t i = 1; i + 1 < nodes.size(); ++i) {
      if (stepDir(nodes[i - 1], nodes[i]) != stepDir(nodes[i], nodes[i + 1])) {
        corners.push_back(nodes[i]);
      }
    }
    if (nodes.size() > 1) {
      corners.push_back(nodes.back());
    }
  }
  row.hwUsed = valid;

  if (valid) {
    row.pathMatch = row.cpuOk && expandPath(cpuPath) == nodes;
    row.costMatch = row.cpuOk && row.hw.cost == row.cpuCost;
    pathLength(corners, row.hwWlDbu, row.hwVias);
    path = corners;
    // As traceBackPath(): every path node except the source joins the
    // connected component, and the component box grows over the corners.
    connComps.insert(connComps.end(), nodes.begin(), nodes.end() - 1);
    for (const FlexMazeIdx& mi : corners) {
      ccMazeIdx1.set(std::min(ccMazeIdx1.x(), mi.x()),
                     std::min(ccMazeIdx1.y(), mi.y()),
                     std::min(ccMazeIdx1.z(), mi.z()));
      ccMazeIdx2.set(std::max(ccMazeIdx2.x(), mi.x()),
                     std::max(ccMazeIdx2.y(), mi.y()),
                     std::max(ccMazeIdx2.z(), mi.z()));
    }
    ++netRow.hwSearchesUsed;
    netRow.pathMatches += row.pathMatch;
  } else {
    // Keep the trial going on the CPU path so later pins are still tried.
    path = cpuPath;
    connComps = cpuConnComps;
    ccMazeIdx1 = cpuCc1;
    ccMazeIdx2 = cpuCc2;
  }
  if (valid) {
    // Timed only over searches the accelerator completed, like iters.csv.
    netRow.cpuAstarMs += row.cpuMs;
    netRow.hwCycles += std::max<int64_t>(row.hw.cycles, 0);
    netRow.hwMs += std::max<int64_t>(row.hw.cycles, 0)
                   / (astar_hw::config().clkMhz * 1000.0);
  }
  netRow.pqFullSearches += row.hw.status == "pq_full";
  astar_hw::recordSearch(row);

  const int keep = astar_hw::config().keepJobs;
  if (keep >= 2 || (keep == 1 && !(row.hwUsed && row.pathMatch))) {
    hwKeptJobs_ = true;
  } else {
    std::error_code ec;
    std::filesystem::remove_all(jobDir, ec);
  }
  return valid || row.cpuOk;
}

void FlexDRWorker::hwTrialRecordCpuRoute(drNet* net)
{
  if (!hwPendingNet_) {
    return;
  }
  astar_hw::NetRow& row = *hwPendingNet_;
  row.cpu = astar_hw::measureNet(getDesign(), net);
  for (const auto& marker : gcWorker_->getMarkers()) {
    row.cpu.addMarker(*marker);
  }
  if (row.pqFullSearches > 0) {
    // An open-list overflow means the accelerator could not route the net;
    // score it as the software route it falls back to.
    row.hw = row.cpu;
  }
  astar_hw::recordNet(row);
  hwPendingNet_.reset();
}

void FlexDRWorker::hwTrialEndWorker()
{
  if (!astar_hw::iterEnabled(getDRIter())) {
    return;
  }
  astar_hw::recordWorker(
      getDRIter(), astarRuntimeMs_, astarSearchCount_, hwTrialMs_);
  const int keep = astar_hw::config().keepJobs;
  if (astar_hw::trialEnabled(getDRIter())
      && (keep == 0 || (keep == 1 && !hwKeptJobs_))) {
    std::error_code ec;
    std::filesystem::remove_all(astar_hw::boxDir(getDRIter(), getRouteBox()),
                                ec);
  }
}

}  // namespace drt
