// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// A* hardware trial on the worker side: route a net with accelerator paths,
// measure it, undo it. See AstarHwTrial.h for the overall flow.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "db/drObj/drNet.h"
#include "db/drObj/drPin.h"
#include "db/drObj/drVia.h"
#include "db/gcObj/gcNet.h"
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

uint64_t mixHash(uint64_t h, uint64_t v)
{
  return h ^ (v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2));
}

// Hash of a multiset: sort, then fold, so iteration order does not matter.
uint64_t foldSorted(std::vector<uint64_t>& v)
{
  std::sort(v.begin(), v.end());
  uint64_t h = v.size();
  for (const uint64_t x : v) {
    h = mixHash(h, x);
  }
  return h;
}

uint64_t rectHash(const odb::Rect& r)
{
  uint64_t h = 0;
  for (const int c : {r.xMin(), r.yMin(), r.xMax(), r.yMax()}) {
    h = mixHash(h, static_cast<uint32_t>(c));
  }
  return h;
}

}  // namespace

std::map<std::string, uint64_t> FlexDRWorker::hwTrialFingerprint()
{
  std::map<std::string, uint64_t> fp;
  std::vector<uint64_t> v;
  const auto numLayers = getTech()->getLayers().size();

  // Worker region query, every layer over the ext box.
  for (frLayerNum l = 0; l < static_cast<frLayerNum>(numLayers); ++l) {
    std::vector<drConnFig*> figs;
    rq_.query(getExtBox(), l, figs);
    for (const drConnFig* f : figs) {
      v.push_back(mixHash(
          mixHash(rectHash(f->getBBox()), static_cast<uint64_t>(f->typeId())),
          mixHash(static_cast<uint64_t>(l),
                  f->getNet() ? f->getNet()->getId() : ~0ULL)));
    }
  }
  fp["region_query"] = foldSorted(v);
  // Same shapes in query() order, which callers depend on.
  uint64_t ordered = 0;
  for (frLayerNum l = 0; l < static_cast<frLayerNum>(numLayers); ++l) {
    std::vector<drConnFig*> figs;
    rq_.query(getExtBox(), l, figs);
    for (const drConnFig* f : figs) {
      ordered = mixHash(ordered, mixHash(rectHash(f->getBBox()), l));
    }
  }
  fp["region_query_order"] = ordered;

  // gc worker: route and fixed rectangles of every gc net.
  v.clear();
  std::vector<uint64_t> fixedV;
  for (const auto& gnet : gcWorker_->getNets()) {
    const uint64_t id = gnet->getOwner() ? gnet->getOwner()->getId() : ~0ULL;
    for (const bool fixed : {false, true}) {
      const auto& byLayer = gnet->getRectangles(fixed);
      for (size_t l = 0; l < byLayer.size(); ++l) {
        for (const auto& r : byLayer[l]) {
          (fixed ? fixedV : v)
              .push_back(mixHash(
                  mixHash(id, l),
                  rectHash(odb::Rect(
                      gtl::xl(r), gtl::yl(r), gtl::xh(r), gtl::yh(r)))));
        }
      }
    }
  }
  fp["gc_route_rects"] = foldSorted(v);
  fp["gc_fixed_rects"] = foldSorted(fixedV);

  // Markers and history.
  v.clear();
  for (const auto& m : markers_) {
    v.push_back(mixHash(rectHash(m.getBBox()), m.getLayerNum()));
  }
  fp["markers"] = foldSorted(v);
  auto idxHash = [](const FlexMazeIdx& mi) {
    return mixHash(mixHash(mi.x(), mi.y()), mi.z());
  };
  for (const auto& [name, set] :
       {std::pair{"planar_history", &planarHistoryMarkers_},
        std::pair{"via_history", &viaHistoryMarkers_}}) {
    v.clear();
    for (const auto& mi : *set) {
      v.push_back(idxHash(mi));
    }
    fp[name] = foldSorted(v);
  }
  v.clear();
  for (size_t z = 0; z < historyMarkers_.size(); ++z) {
    for (const auto& mi : historyMarkers_[z]) {
      v.push_back(mixHash(idxHash(mi), z));
    }
  }
  fp["history_markers"] = foldSorted(v);
  v.clear();
  for (const auto& [mi, ap] : apSVia_) {
    v.push_back(
        mixHash(idxHash(mi),
                std::hash<std::string>{}(ap->getAccessViaDef()
                                             ? ap->getAccessViaDef()->getName()
                                             : std::string())));
  }
  fp["ap_svia"] = foldSorted(v);
  v.clear();
  for (const auto& p : specialAccessAPs_) {
    v.push_back(mixHash(mixHash(p.x(), p.y()), p.z()));
  }
  fp["special_access_aps"] = foldSorted(v);

  // Every net: its routes, ext figs, counters and access patterns.
  std::vector<uint64_t> routes, ext, counters, aps;
  for (const auto& n : nets_) {
    const uint64_t id = n->getId();
    for (const auto& f : n->getRouteConnFigs()) {
      routes.push_back(mixHash(
          id,
          mixHash(rectHash(f->getBBox()), static_cast<uint64_t>(f->typeId()))));
    }
    for (const auto& f : n->getExtConnFigs()) {
      uint64_t h
          = mixHash(rectHash(f->getBBox()), static_cast<uint64_t>(f->typeId()));
      if (f->typeId() == drcVia) {
        const auto* via = static_cast<const drVia*>(f.get());
        h = mixHash(h, via->isBottomConnected() * 2 + via->isTopConnected());
      }
      ext.push_back(mixHash(id, h));
    }
    uint64_t c = mixHash(mixHash(n->getNumMarkers(), n->getNumReroutes()),
                         mixHash(n->isModified(), n->hasExtFigUpdates()));
    for (const uint64_t x : {static_cast<uint64_t>(n->getNumPinsIn()),
                             static_cast<uint64_t>(n->getMarkerDist()),
                             static_cast<uint64_t>(n->isRipup()),
                             static_cast<uint64_t>(n->isInQueue()),
                             static_cast<uint64_t>(n->isRouted()),
                             static_cast<uint64_t>(n->getPriority()),
                             static_cast<uint64_t>(n->getNRipupAvoids()),
                             static_cast<uint64_t>(n->getMaxRipupAvoids())}) {
      c = mixHash(c, x);
    }
    counters.push_back(mixHash(id, c));
    for (const auto& f : n->getBestRouteConnFigs()) {
      routes.push_back(mixHash(
          ~id,
          mixHash(rectHash(f->getBBox()), static_cast<uint64_t>(f->typeId()))));
    }
    for (const auto& pin : n->getPins()) {
      for (const auto& ap : pin->getAccessPatterns()) {
        uint64_t h
            = mixHash(id, mixHash(ap->getPoint().x(), ap->getPoint().y()));
        h = mixHash(h, ap->hasMazeIdx() ? idxHash(ap->getMazeIdx()) : 0);
        for (const auto dir : frDirEnumAll) {
          h = mixHash(h, ap->hasValidAccess(dir));
        }
        h = mixHash(h,
                    std::hash<std::string>{}(
                        ap->getAccessViaDef() ? ap->getAccessViaDef()->getName()
                                              : std::string()));
        aps.push_back(h);
      }
    }
  }
  fp["net_routes"] = foldSorted(routes);
  fp["net_ext_figs"] = foldSorted(ext);
  fp["net_counters"] = foldSorted(counters);
  fp["access_patterns"] = foldSorted(aps);
  fp["grid_graph"] = gridGraph_.stateHash();
  return fp;
}

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
  static const bool checkRestore = std::getenv("DRT_ASTAR_HW_CHECK_RESTORE");
  // DRT_ASTAR_HW_FINGERPRINT_LOG: the same fingerprint at the start of every
  // trialed net, to diff two runs and find the first structure to diverge.
  static const bool logFingerprint
      = std::getenv("DRT_ASTAR_HW_FINGERPRINT_LOG");
  std::map<std::string, uint64_t> before;
  if (checkRestore || logFingerprint) {
    before = hwTrialFingerprint();
  }
  if (logFingerprint) {
    static std::mutex fpMutex;
    std::lock_guard<std::mutex> lock(fpMutex);
    static std::ofstream fpLog(astar_hw::config().outDir + "/fingerprints.log");
    fpLog << row.iter << " " << row.box.xMin() << " " << row.box.yMin() << " "
          << row.net << " " << row.reroute;
    for (const auto& [name, h] : before) {
      fpLog << " " << name << "=" << h;
    }
    fpLog << "\n" << std::flush;
  }
  FlexGridGraph::SavedState saved;
  gridGraph_.saveState(saved);
  rq_.saveSnapshot();
  // The trial routes and checks against its own gc worker, built like the
  // worker's (init()), so the worker's never holds trial geometry. Its copy
  // of this net is the route before the rip-up, which cannot be rebuilt
  // from the cleared drNet, and later rip-up costs (modEolCosts_poly) read it.
  auto trialGc
      = std::make_unique<FlexGCWorker>(getTech(), logger_, router_cfg_, this);
  trialGc->setExtBox(getExtBox());
  trialGc->setDrcBox(getDrcBox());
  trialGc->init(getDesign());
  trialGc->setEnableSurgicalFix(true);
  std::swap(gcWorker_, trialGc);
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
  std::swap(gcWorker_, trialGc);

  // Undo: drop the trial geometry, put the worker region query back exactly
  // (its trees hold no shape of this net: the rip-up removed them) and every
  // grid-graph cost.
  rq_.restoreSnapshot();
  net->clear();
  gridGraph_.restoreState(saved);
  if (checkRestore) {
    // One line per changed structure in restore_check.log (the logger
    // caps a message id at 1000 lines).
    static std::mutex checkMutex;
    const auto after = hwTrialFingerprint();
    std::lock_guard<std::mutex> lock(checkMutex);
    static std::ofstream checkLog(astar_hw::config().outDir
                                  + "/restore_check.log");
    for (const auto& [name, h] : before) {
      if (after.at(name) != h) {
        checkLog << name << " iter " << row.iter << " box " << row.box.xMin()
                 << " " << row.box.yMin() << " net " << row.net << " reroute "
                 << row.reroute << "\n";
      }
    }
    checkLog << std::flush;
  }

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
