// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#include "dr/AstarHwTrial.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "db/drObj/drNet.h"
#include "db/drObj/drShape.h"
#include "db/drObj/drVia.h"
#include "db/obj/frMarker.h"
#include "db/obj/frNet.h"
#include "db/obj/frShape.h"
#include "db/obj/frTrackPattern.h"
#include "db/obj/frVia.h"
#include "db/tech/frConstraint.h"
#include "db/tech/frLayer.h"
#include "db/tech/frViaDef.h"
#include "frBaseTypes.h"
#include "frDesign.h"
#include "frRegionQuery.h"
#include "odb/dbTypes.h"
#include "odb/geom.h"

namespace drt::astar_hw {

namespace {

std::string envString(const char* name)
{
  const char* v = std::getenv(name);
  return v == nullptr ? std::string() : std::string(v);
}

double envDouble(const char* name, double def)
{
  const std::string v = envString(name);
  return v.empty() ? def : std::strtod(v.c_str(), nullptr);
}

int envInt(const char* name, int def)
{
  const std::string v = envString(name);
  return v.empty() ? def : std::atoi(v.c_str());
}

Config loadConfig()
{
  Config c;
  c.outDir = envString("DRT_ASTAR_HW_OUT");
  c.enabled = !c.outDir.empty();
  if (!c.enabled) {
    return c;
  }
  c.recordOnly = envString("DRT_ASTAR_HW_MODE") == "record";
  c.cmd = envString("DRT_ASTAR_HW_CMD");
  c.clkMhz = envDouble("DRT_ASTAR_HW_CLK_MHZ", 100.0);
  c.netProb = envDouble("DRT_ASTAR_HW_NET_PROB", 1.0);
  c.minIter = envInt("DRT_ASTAR_HW_MIN_ITER", 0);
  c.maxIter = envInt("DRT_ASTAR_HW_MAX_ITER", std::numeric_limits<int>::max());
  c.seed = std::strtoull(envString("DRT_ASTAR_HW_SEED").empty()
                             ? "1"
                             : envString("DRT_ASTAR_HW_SEED").c_str(),
                         nullptr,
                         10);
  c.keepJobs = envInt("DRT_ASTAR_HW_KEEP_JOBS", 0);
  c.runId = envString("DRT_ASTAR_HW_RUN_ID");
  if (c.runId.empty()) {
    char buf[32];
    const std::time_t now = std::time(nullptr);
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&now));
    c.runId = buf;
  }
  std::filesystem::create_directories(c.recordOnly ? c.outDir
                                                   : c.outDir + "/jobs");
  return c;
}

// splitmix64 finalizer, as used by the gg-dump sampler.
uint64_t mix64(uint64_t x)
{
  x += 0x9e3779b97f4a7c15ULL;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
  return x ^ (x >> 31);
}

std::string shellQuote(const std::string& s)
{
  std::string out = "'";
  for (const char c : s) {
    if (c == '\'') {
      out += "'\\''";
    } else {
      out += c;
    }
  }
  return out + "'";
}

// ---------------------------------------------------------------------------
// Geometry measurement

// Track coordinates per routing layer: xTracks are the x locations of
// vertical tracks (frTrackPattern::isHorizontal(), DEF "TRACKS X"), yTracks
// the y locations of horizontal ones.
struct TrackIndex
{
  std::vector<std::vector<const frTrackPattern*>> xTracks;
  std::vector<std::vector<const frTrackPattern*>> yTracks;
};

const TrackIndex& trackIndex(const frDesign* design)
{
  static std::once_flag once;
  static TrackIndex index;
  std::call_once(once, [design]() {
    const auto numLayers = design->getTech()->getLayers().size();
    index.xTracks.resize(numLayers);
    index.yTracks.resize(numLayers);
    for (size_t l = 0; l < numLayers; ++l) {
      for (const auto& tp : design->getTopBlock()->getTrackPatterns(l)) {
        (tp->isHorizontal() ? index.xTracks : index.yTracks)[l].push_back(
            tp.get());
      }
    }
  });
  return index;
}

bool onTrack(const std::vector<const frTrackPattern*>& patterns, frCoord c)
{
  for (const frTrackPattern* tp : patterns) {
    const frCoord rel = c - tp->getStartCoord();
    if (rel < 0 || rel % static_cast<frCoord>(tp->getTrackSpacing()) != 0) {
      continue;
    }
    if (rel / static_cast<frCoord>(tp->getTrackSpacing())
        < static_cast<frCoord>(tp->getNumTracks())) {
      return true;
    }
  }
  return false;
}

// Original (unprocessed) route guides of `net` on `layer` touching `box`.
std::vector<odb::Rect> netGuides(const frDesign* design,
                                 const frNet* net,
                                 frLayerNum layer,
                                 const odb::Rect& box)
{
  frRegionQuery::Objects<frNet> found;
  design->getRegionQuery()->queryOrigGuide(box, layer, found);
  std::vector<odb::Rect> rects;
  for (auto& [rect, owner] : found) {
    if (owner == net) {
      rects.push_back(rect);
    }
  }
  return rects;
}

void addWire(Quality& q,
             const frDesign* design,
             const frNet* net,
             frLayerNum layerNum,
             odb::Point b,
             odb::Point e)
{
  const frCoord len = odb::Point::manhattanDistance(b, e);
  if (len == 0) {
    return;
  }
  q.wlDbu += len;
  const frLayer* layer = design->getTech()->getLayer(layerNum);
  const bool horzSeg = b.y() == e.y();
  if (horzSeg != (layer->getDir() == odb::dbTechLayerDir::HORIZONTAL)) {
    q.wrongWayDbu += len;
  }
  const TrackIndex& tracks = trackIndex(design);
  if (horzSeg ? !onTrack(tracks.yTracks[layerNum], b.y())
              : !onTrack(tracks.xTracks[layerNum], b.x())) {
    q.offTrackWireDbu += len;
  }
  // Out-of-guide length: the part of the centerline not covered by any guide
  // of this net on the same layer.
  const frCoord lo = horzSeg ? std::min(b.x(), e.x()) : std::min(b.y(), e.y());
  const frCoord hi = horzSeg ? std::max(b.x(), e.x()) : std::max(b.y(), e.y());
  const frCoord fixed = horzSeg ? b.y() : b.x();
  std::vector<std::pair<frCoord, frCoord>> cover;
  for (const odb::Rect& g : netGuides(design, net, layerNum, odb::Rect(b, e))) {
    const bool spans = horzSeg ? (g.yMin() <= fixed && fixed <= g.yMax())
                               : (g.xMin() <= fixed && fixed <= g.xMax());
    if (!spans) {
      continue;
    }
    const frCoord glo = std::max(lo, horzSeg ? g.xMin() : g.yMin());
    const frCoord ghi = std::min(hi, horzSeg ? g.xMax() : g.yMax());
    if (glo < ghi) {
      cover.emplace_back(glo, ghi);
    }
  }
  std::sort(cover.begin(), cover.end());
  frCoord covered = 0;
  frCoord reach = lo;
  for (const auto& [clo, chi] : cover) {
    const frCoord start = std::max(clo, reach);
    if (chi > start) {
      covered += chi - start;
      reach = chi;
    }
  }
  q.offGuideWireDbu += len - covered;
}

void addVia(Quality& q,
            const frDesign* design,
            const frNet* net,
            const frViaDef* viaDef,
            odb::Point origin)
{
  ++q.vias;
  const frLayerNum l1 = viaDef->getLayer1Num();
  const frLayerNum l2 = viaDef->getLayer2Num();
  const TrackIndex& tracks = trackIndex(design);
  const bool xOn = onTrack(tracks.xTracks[l1], origin.x())
                   || onTrack(tracks.xTracks[l2], origin.x());
  const bool yOn = onTrack(tracks.yTracks[l1], origin.y())
                   || onTrack(tracks.yTracks[l2], origin.y());
  if (!xOn || !yOn) {
    ++q.offTrackVias;
  }
  // In guide when both metal layers have a guide of this net at the origin.
  const odb::Rect pt(origin, origin);
  if (netGuides(design, net, l1, pt).empty()
      || netGuides(design, net, l2, pt).empty()) {
    ++q.offGuideVias;
  }
}

// ---------------------------------------------------------------------------
// CSV recorder

struct IterAgg
{
  int netsTrialed = 0;
  int netsHwRouted = 0;
  int searches = 0;
  int hwUsed = 0;
  int pathMatch = 0;
  int costMatch = 0;
  double trialCpuMs = 0;
  double trialHwMs = 0;
  int64_t trialHwCycles = 0;
  double trialMs = 0;
  double cpuRouteAstarMs = 0;
  int64_t cpuRouteSearches = 0;
  Quality cpu;
  Quality hw;
};

const char* const kQualityCols[] = {"wl_dbu",
                                    "vias",
                                    "offguide_wire_dbu",
                                    "offguide_vias",
                                    "offtrack_wire_dbu",
                                    "offtrack_vias",
                                    "wrongway_dbu",
                                    "patches",
                                    "drc",
                                    "shorts",
                                    "short_area",
                                    "minarea",
                                    "spacing",
                                    "other_drc"};

std::string qualityHeader(const std::string& prefix)
{
  std::string out;
  for (const char* col : kQualityCols) {
    out += "," + prefix + col;
  }
  return out;
}

std::string qualityValues(const Quality& q)
{
  std::ostringstream os;
  os << "," << q.wlDbu << "," << q.vias << "," << q.offGuideWireDbu << ","
     << q.offGuideVias << "," << q.offTrackWireDbu << "," << q.offTrackVias
     << "," << q.wrongWayDbu << "," << q.patches << "," << q.markers << ","
     << q.shorts << "," << q.shortArea << "," << q.minArea << "," << q.spacing
     << "," << q.otherDrc;
  return os.str();
}

std::string boxValues(const odb::Rect& b)
{
  std::ostringstream os;
  os << "," << b.xMin() << "," << b.yMin() << "," << b.xMax() << ","
     << b.yMax();
  return os.str();
}

double hwMs(const HwResult& hw)
{
  return hw.cycles < 0 ? 0.0 : hw.cycles / (config().clkMhz * 1000.0);
}

class Recorder
{
 public:
  static Recorder& get()
  {
    static Recorder r;
    return r;
  }

  void search(const SearchRow& r)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::ofstream& os = open(searches_,
                             "searches.csv",
                             "run_id,iter,box_xmin,box_ymin,box_xmax,box_ymax,"
                             "net,reroute,search_idx,dim_x,dim_y,dim_z,"
                             "hw_status,hw_used,path_match,cost_match,cpu_ok,"
                             "cpu_cost,hw_cost,cpu_nodes,hw_nodes,cpu_wl_dbu,"
                             "hw_wl_dbu,cpu_vias,hw_vias,cpu_ms,hw_cycles,"
                             "hw_ms,speedup,forward_cycles,eval_cycles,"
                             "eval_stall_cycles,search_cycles,convert_s,"
                             "compile_s,sim_s,job_s");
    const double hms = hwMs(r.hw);
    os << config().runId << "," << r.iter << boxValues(r.box) << "," << r.net
       << "," << r.reroute << "," << r.searchIdx << "," << r.dimX << ","
       << r.dimY << "," << r.dimZ << "," << r.hw.status << "," << r.hwUsed
       << "," << r.pathMatch << "," << r.costMatch << "," << r.cpuOk << ","
       << r.cpuCost << "," << r.hw.cost << "," << r.cpuNodes << ","
       << r.hw.nodes << "," << r.cpuWlDbu << "," << r.hwWlDbu << ","
       << r.cpuVias << "," << r.hwVias << "," << r.cpuMs << "," << r.hw.cycles
       << "," << hms << "," << (hms > 0 ? r.cpuMs / hms : 0.0) << ","
       << r.hw.forwardCycles << "," << r.hw.evalCycles << ","
       << r.hw.evalStallCycles << "," << r.hw.searchCycles << ","
       << r.hw.convertS << "," << r.hw.compileS << "," << r.hw.simS << ","
       << r.hw.jobS << "\n";
    os.flush();
    IterAgg& a = iters_[r.iter];
    ++a.searches;
    a.hwUsed += r.hwUsed;
    a.pathMatch += r.pathMatch;
    a.costMatch += r.costMatch;
    a.trialCpuMs += r.cpuMs;
    a.trialHwMs += hms;
    a.trialHwCycles += std::max<int64_t>(r.hw.cycles, 0);
  }

  void net(const NetRow& r)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::ofstream& os = open(nets_,
                             "nets.csv",
                             "run_id,iter,box_xmin,box_ymin,box_xmax,box_ymax,"
                             "net,reroute,status,searches,hw_searches_used,"
                             "path_matches,hw_routed"
                                 + qualityHeader("cpu_") + qualityHeader("hw_")
                                 + ",cpu_astar_ms,hw_ms,hw_cycles,trial_ms");
    os << config().runId << "," << r.iter << boxValues(r.box) << "," << r.net
       << "," << r.reroute << "," << r.status << "," << r.searches << ","
       << r.hwSearchesUsed << "," << r.pathMatches << "," << r.hwRouted
       << qualityValues(r.cpu) << qualityValues(r.hw) << "," << r.cpuAstarMs
       << "," << r.hwMs << "," << r.hwCycles << "," << r.trialMs << "\n";
    os.flush();
    IterAgg& a = iters_[r.iter];
    ++a.netsTrialed;
    a.netsHwRouted += r.hwRouted;
    a.cpu.add(r.cpu);
    a.hw.add(r.hw);
  }

  void worker(int iter, double astarMs, int searches, double trialMs)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    IterAgg& a = iters_[iter];
    a.cpuRouteAstarMs += astarMs;
    a.cpuRouteSearches += searches;
    a.trialMs += trialMs;
  }

  void iteration(const frDesign* design, int iter, int threads, double wallMs)
  {
    const Quality design_q = measureDesign(design);
    std::lock_guard<std::mutex> lock(mutex_);
    const IterAgg a = iters_[iter];
    iters_.erase(iter);
    std::ofstream& os = open(
        iters_file_,
        "iters.csv",
        "run_id,iter,threads,iter_wall_ms,trial_ms,cpu_route_astar_ms,"
        "cpu_route_searches,nets_trialed,nets_hw_routed,searches,hw_used,"
        "path_match,cost_match,trial_cpu_ms,trial_hw_ms,trial_hw_cycles,"
        "clk_mhz,dbu_per_micron,m2_pitch_dbu"
            + qualityHeader("cpu_") + qualityHeader("hw_")
            + qualityHeader("design_"));
    frCoord m2Pitch = 0;
    int routing = 0;
    for (const auto& layer : design->getTech()->getLayers()) {
      if (layer->getType() == odb::dbTechLayerType::ROUTING && ++routing == 2) {
        m2Pitch = layer->getPitch();
        break;
      }
    }
    os << config().runId << "," << iter << "," << threads << "," << wallMs
       << "," << a.trialMs << "," << a.cpuRouteAstarMs << ","
       << a.cpuRouteSearches << "," << a.netsTrialed << "," << a.netsHwRouted
       << "," << a.searches << "," << a.hwUsed << "," << a.pathMatch << ","
       << a.costMatch << "," << a.trialCpuMs << "," << a.trialHwMs << ","
       << a.trialHwCycles << "," << config().clkMhz << ","
       << design->getTopBlock()->getDBUPerUU() << "," << m2Pitch
       << qualityValues(a.cpu) << qualityValues(a.hw) << qualityValues(design_q)
       << "\n";
    os.flush();
  }

 private:
  std::ofstream& open(std::ofstream& os,
                      const char* name,
                      const std::string& header)
  {
    if (!os.is_open()) {
      const std::string path = config().outDir + "/" + name;
      const bool fresh = !std::filesystem::exists(path)
                         || std::filesystem::file_size(path) == 0;
      os.open(path, std::ios::app);
      if (fresh) {
        os << header << "\n";
      }
    }
    return os;
  }

  std::mutex mutex_;
  std::ofstream searches_;
  std::ofstream nets_;
  std::ofstream iters_file_;
  std::map<int, IterAgg> iters_;
};

}  // namespace

const Config& config()
{
  static const Config c = loadConfig();
  return c;
}

bool iterEnabled(int iter)
{
  const Config& c = config();
  return c.enabled && iter >= c.minIter && iter <= c.maxIter;
}

bool trialEnabled(int iter)
{
  return iterEnabled(iter) && !config().recordOnly;
}

std::string boxDir(int iter, const odb::Rect& routeBox)
{
  return config().outDir + "/jobs/iter" + std::to_string(iter) + "_x"
         + std::to_string(routeBox.xMin()) + "_y"
         + std::to_string(routeBox.yMin());
}

std::string forbiddenTablesPath()
{
  return config().outDir + "/forbidden_tables.txt";
}

bool sampleNet(int iter,
               const odb::Rect& routeBox,
               const std::string& netName,
               int numReroutes)
{
  const double p = config().netProb;
  if (p >= 1.0) {
    return true;
  }
  uint64_t h = mix64(config().seed);
  for (const uint64_t key :
       {static_cast<uint64_t>(iter),
        static_cast<uint64_t>(routeBox.xMin()),
        static_cast<uint64_t>(routeBox.yMin()),
        static_cast<uint64_t>(std::hash<std::string>{}(netName)),
        static_cast<uint64_t>(numReroutes)}) {
    h = mix64(h ^ key);
  }
  return static_cast<double>(h >> 11) * 0x1.0p-53 < p;
}

HwResult runJob(const std::string& jobDir,
                const std::string& ggPath,
                const std::string& snapshotPath,
                const std::string& searchPath)
{
  HwResult res;
  if (config().cmd.empty()) {
    res.status = "no_cmd";
    return res;
  }
  const std::string resultPath = jobDir + "/hw_result.txt";
  std::filesystem::remove(resultPath);
  const std::string cmd = config().cmd + " --job " + shellQuote(jobDir)
                          + " --gg " + shellQuote(ggPath) + " --snapshot "
                          + shellQuote(snapshotPath) + " --search "
                          + shellQuote(searchPath) + " --forbidden "
                          + shellQuote(forbiddenTablesPath()) + " > "
                          + shellQuote(jobDir + "/hw_job.log") + " 2>&1";
  if (std::system(cmd.c_str()) != 0) {
    res.status = "cmd_failed";
  }
  std::ifstream in(resultPath);
  if (!in.is_open()) {
    if (res.status == "not_run") {
      res.status = "no_result";
    }
    return res;
  }
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream ls(line);
    std::string key;
    ls >> key;
    if (key == "status") {
      ls >> res.status;
    } else if (key == "cycles") {
      ls >> res.cycles;
    } else if (key == "nodes") {
      ls >> res.nodes;
    } else if (key == "hw_cost") {
      ls >> res.cost;
    } else if (key == "hw_wirelength") {
      ls >> res.wirelength;
    } else if (key == "forward_cycles") {
      ls >> res.forwardCycles;
    } else if (key == "eval_cycles") {
      ls >> res.evalCycles;
    } else if (key == "eval_stall_cycles") {
      ls >> res.evalStallCycles;
    } else if (key == "search_cycles") {
      ls >> res.searchCycles;
    } else if (key == "convert_s") {
      ls >> res.convertS;
    } else if (key == "compile_s") {
      ls >> res.compileS;
    } else if (key == "sim_s") {
      ls >> res.simS;
    } else if (key == "job_s") {
      ls >> res.jobS;
    } else if (key == "path") {
      std::string rest;
      std::getline(ls, rest);
      std::replace(rest.begin(), rest.end(), ';', ' ');
      std::istringstream ps(rest);
      int x, y, z;
      while (ps >> x >> y >> z) {
        res.path.emplace_back(x, y, z);
      }
    }
  }
  return res;
}

void Quality::add(const Quality& o)
{
  wlDbu += o.wlDbu;
  vias += o.vias;
  offGuideWireDbu += o.offGuideWireDbu;
  offGuideVias += o.offGuideVias;
  offTrackWireDbu += o.offTrackWireDbu;
  offTrackVias += o.offTrackVias;
  wrongWayDbu += o.wrongWayDbu;
  patches += o.patches;
  markers += o.markers;
  shorts += o.shorts;
  shortArea += o.shortArea;
  minArea += o.minArea;
  spacing += o.spacing;
  otherDrc += o.otherDrc;
}

void Quality::addMarker(const frMarker& marker)
{
  ++markers;
  const frConstraint* con = marker.getConstraint();
  if (con == nullptr) {
    ++otherDrc;
    return;
  }
  std::ostringstream os;
  os << con->typeId();
  const std::string name = os.str();
  if (name == "frcShortConstraint") {
    ++shorts;
    shortArea += static_cast<double>(marker.getBBox().area());
  } else if (name == "frcAreaConstraint" || name == "frcLef58AreaConstraint") {
    ++minArea;
  } else if (name.find("Spacing") != std::string::npos
             || name.find("Spc") != std::string::npos
             || name.find("Eol") != std::string::npos) {
    ++spacing;
  } else {
    ++otherDrc;
  }
}

Quality measureNet(const frDesign* design, const drNet* net)
{
  Quality q;
  const frNet* fnet = net->getFrNet();
  for (const auto& fig : net->getRouteConnFigs()) {
    switch (fig->typeId()) {
      case drcPathSeg: {
        const auto* ps = static_cast<const drPathSeg*>(fig.get());
        auto [b, e] = ps->getPoints();
        addWire(q, design, fnet, ps->getLayerNum(), b, e);
        break;
      }
      case drcVia: {
        const auto* via = static_cast<const drVia*>(fig.get());
        addVia(q, design, fnet, via->getViaDef(), via->getOrigin());
        break;
      }
      case drcPatchWire:
        ++q.patches;
        break;
      default:
        break;
    }
  }
  return q;
}

Quality measureDesign(const frDesign* design)
{
  Quality q;
  for (const auto& net : design->getTopBlock()->getNets()) {
    for (const auto& shape : net->getShapes()) {
      if (shape->typeId() == frcPathSeg) {
        const auto* ps = static_cast<const frPathSeg*>(shape.get());
        auto [b, e] = ps->getPoints();
        addWire(q, design, net.get(), ps->getLayerNum(), b, e);
      }
    }
    for (const auto& via : net->getVias()) {
      addVia(q, design, net.get(), via->getViaDef(), via->getOrigin());
    }
    q.patches += net->getPatchWires().size();
  }
  for (const auto& marker : design->getTopBlock()->getMarkers()) {
    q.addMarker(*marker);
  }
  return q;
}

void recordSearch(const SearchRow& row)
{
  Recorder::get().search(row);
}

void recordNet(const NetRow& row)
{
  Recorder::get().net(row);
}

void recordWorker(int iter,
                  double cpuRouteAstarMs,
                  int cpuRouteSearches,
                  double trialMs)
{
  Recorder::get().worker(iter, cpuRouteAstarMs, cpuRouteSearches, trialMs);
}

void endIteration(const frDesign* design,
                  int iter,
                  int threads,
                  double iterWallMs)
{
  Recorder::get().iteration(design, iter, threads, iterWallMs);
}

}  // namespace drt::astar_hw
