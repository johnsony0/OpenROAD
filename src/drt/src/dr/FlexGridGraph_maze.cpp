// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include <algorithm>
#include <bitset>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "db/drObj/drPin.h"
#include "db/tech/frLayer.h"
#include "dr/FlexDR.h"
#include "dr/FlexGridGraph.h"
#include "dr/FlexMazeTypes.h"
#include "dr/FlexWavefront.h"
#include "frBaseTypes.h"
#include "global.h"
#include "odb/dbTypes.h"
#include "odb/geom.h"

using odb::dbTechLayerDir;

namespace drt {
const int debugMazeIter = std::numeric_limits<int>::max();
// Flip to true to dump every maze search without setting DRT_DUMP_EXP_DIR; the
// files then land in expDumpDefaultDir. See openExpansionDump().
const bool expDumpAlways = false;
// Used when DRT_DUMP_EXP_DIR is unset. Relative to the cwd of the openroad
// process, and it must already exist.
const char* const expDumpDefaultDir = ".";

// Companion to dumpGridGraph()/dumpSearch() recording which expansions the maze
// search was allowed to make: the inputs and verdict of every isExpandable()
// call. Gated by DRT_DUMP_EXP_DIR (kept separate from DRT_DUMP_GG_DIR because
// this dump is much larger than the other two; point both at the same directory
// to have the files sit together). DRT_DUMP_EXP_ITER=<n> restricts it to a
// single DR iteration. Split like the grid-graph dump: one file per grid graph
// per iteration, covering every search() the worker runs.
void FlexGridGraph::openExpansionDump()
{
  if (exp_file_.is_open() || expDumpTried_) {
    return;
  }
  expDumpTried_ = true;
  if (drWorker_ == nullptr) {
    return;
  }
  const char* env_dir = std::getenv("DRT_DUMP_EXP_DIR");
  const bool have_env_dir = env_dir != nullptr && env_dir[0] != '\0';
  if (!have_env_dir && !expDumpAlways) {
    return;
  }
  const char* dir = have_env_dir ? env_dir : expDumpDefaultDir;
  const int iter = drWorker_->getDRIter();
  const char* iter_filter = std::getenv("DRT_DUMP_EXP_ITER");
  if (iter_filter != nullptr && iter_filter[0] != '\0'
      && std::atoi(iter_filter) != iter) {
    return;
  }
  const odb::Rect& rb = drWorker_->getRouteBox();
  const std::string path = std::string(dir) + "/exp_iter" + std::to_string(iter)
                           + "_x" + std::to_string(rb.xMin()) + "_y"
                           + std::to_string(rb.yMin()) + ".txt";
  exp_file_.open(path.c_str());
  if (!exp_file_.is_open()) {
    logger_->warn(utl::DRT, 620, "Could not open expansion dump file {}", path);
    return;
  }
  frMIdx xDim, yDim, zDim;
  getDim(xDim, yDim, zDim);
  exp_file_ << "version 1\n";
  exp_file_ << fmt::format("iter {}\n", iter);
  exp_file_ << fmt::format(
      "routeBox {} {} {} {}\n", rb.xMin(), rb.yMin(), rb.xMax(), rb.yMax());
  exp_file_ << fmt::format("dim {} {} {}\n", xDim, yDim, zDim);
}

void FlexGridGraph::openCostDump()
{
  if (cost_file_.is_open() || costDumpTried_) {
    return;
  }
  costDumpTried_ = true;
  if (drWorker_ == nullptr) {
    return;
  }
  const char* env_dir = std::getenv("DRT_DUMP_COST_DIR");
  const bool have_env_dir = env_dir != nullptr && env_dir[0] != '\0';
  if (!have_env_dir && !expDumpAlways) {
    return;
  }
  const char* dir = have_env_dir ? env_dir : expDumpDefaultDir;
  const int iter = drWorker_->getDRIter();
  const char* iter_filter = std::getenv("DRT_DUMP_EXP_ITER");
  if (iter_filter != nullptr && iter_filter[0] != '\0'
      && std::atoi(iter_filter) != iter) {
    return;
  }
  const odb::Rect& rb = drWorker_->getRouteBox();
  const std::string path = std::string(dir) + "/cost_iter" + std::to_string(iter)
                           + "_x" + std::to_string(rb.xMin()) + "_y"
                           + std::to_string(rb.yMin()) + ".txt";
  cost_file_.open(path.c_str());
  if (!cost_file_.is_open()) {
    logger_->warn(utl::DRT, 621, "Could not open cost dump file {}", path);
    return;
  }
  frMIdx xDim, yDim, zDim;
  getDim(xDim, yDim, zDim);
  cost_file_ << "version 1\n";
  cost_file_ << fmt::format("iter {}\n", iter);
  cost_file_ << fmt::format(
      "routeBox {} {} {} {}\n", rb.xMin(), rb.yMin(), rb.xMax(), rb.yMax());
  cost_file_ << fmt::format("dim {} {} {}\n", xDim, yDim, zDim);
}

void FlexGridGraph::printExpansion(const FlexWavefrontGrid& currGrid,
                                   const std::string& keyword)
{
  auto dir = currGrid.getLastDir();
  auto gridX = currGrid.x();
  auto gridY = currGrid.y();
  auto gridZ = currGrid.z();
  dir = (frDirEnum) (OPPOSITEDIR - (int) dir);
  bool gridCost = hasGridCost(gridX, gridY, gridZ, dir);
  bool apCost = hasApCost(gridX, gridY, gridZ, dir);
  bool drcCost = hasRouteShapeCostAdj(gridX, gridY, gridZ, dir, false);
  bool markerCost = hasMarkerCostAdj(gridX, gridY, gridZ, dir);
  bool shapeCost = hasFixedShapeCostAdj(gridX, gridY, gridZ, dir, false);
  bool blockCost = isBlocked(gridX, gridY, gridZ, dir);
  bool guideCost = hasGuide(gridX, gridY, gridZ, dir);
  frCoord edgeLength = getEdgeLength(gridX, gridY, gridZ, dir);
  odb::Point pt;
  getPoint(pt, currGrid.x(), currGrid.y());
  dump_file_ << fmt::format(
      "{} {} pt {} cost {} pathCost {} lastDir {} estCost {} gridX {} gridY "
      "{} ",
      keyword,
      currGrid.z(),
      pt,
      currGrid.getCost(),
      currGrid.getPathCost(),
      currGrid.getLastDir(),
      currGrid.getCost() - currGrid.getPathCost(),
      gridX,
      gridY);
  dump_file_ << fmt::format(
      "gridCost {} apCost {} drcCost {} markerCost {} shapeCost {} blockCost "
      "{} "
      "guideCost {} edgeLength {} ",
      gridCost,
      apCost,
      drcCost,
      markerCost,
      shapeCost,
      blockCost,
      guideCost,
      edgeLength);
  dump_file_ << fmt::format(
      "id {} parent_id {}\n", currGrid.getId(), currGrid.getParentId());
  dump_file_.flush();
}
void FlexGridGraph::expand(FlexWavefrontGrid& currGrid,
                           const frDirEnum& dir,
                           const FlexMazeIdx& dstMazeIdx1,
                           const FlexMazeIdx& dstMazeIdx2,
                           const odb::Point& centerPt,
                           bool route_with_jumpers)
{
  frCost nextEstCost, nextPathCost;
  int gridX = currGrid.x();
  int gridY = currGrid.y();
  int gridZ = currGrid.z();

  getNextGrid(gridX, gridY, gridZ, dir);

  FlexMazeIdx nextIdx(gridX, gridY, gridZ);
  // get cost
  nextEstCost
      = getEstCost(FlexMazeIdx(currGrid.x(), currGrid.y(), currGrid.z()),
                   dstMazeIdx1,
                   dstMazeIdx2,
                   dir);
  nextPathCost = getNextPathCost(currGrid, dir, route_with_jumpers);
  odb::Point currPt;
  getPoint(currPt, gridX, gridY);
  frCoord currDist = odb::Point::manhattanDistance(currPt, centerPt);

  // vlength calculation
  frCoord currVLengthX = 0;
  frCoord currVLengthY = 0;
  currGrid.getVLength(currVLengthX, currVLengthY);
  auto nextVLengthX = currVLengthX;
  auto nextVLengthY = currVLengthY;
  bool nextIsPrevViaUp = currGrid.isPrevViaUp();
  if (dir == frDirEnum::U || dir == frDirEnum::D) {
    nextVLengthX = 0;
    nextVLengthY = 0;
    nextIsPrevViaUp
        = (dir == frDirEnum::D);  // up via if current path goes down
  } else {
    if (currVLengthX != std::numeric_limits<frCoord>::max()
        && currVLengthY != std::numeric_limits<frCoord>::max()) {
      if (dir == frDirEnum::W || dir == frDirEnum::E) {
        nextVLengthX
            += getEdgeLength(currGrid.x(), currGrid.y(), currGrid.z(), dir);
      } else {
        nextVLengthY
            += getEdgeLength(currGrid.x(), currGrid.y(), currGrid.z(), dir);
      }
    }
  }

  // tlength calculation
  auto currTLength = currGrid.getTLength();
  auto nextTLength = currTLength;
  // if there was a turn, then add tlength
  if (currTLength != std::numeric_limits<frCoord>::max()) {
    nextTLength += getEdgeLength(currGrid.x(), currGrid.y(), currGrid.z(), dir);
  }
  // if current is a turn, then reset tlength
  if (currGrid.getLastDir() != frDirEnum::UNKNOWN
      && currGrid.getLastDir() != dir) {
    nextTLength = getEdgeLength(currGrid.x(), currGrid.y(), currGrid.z(), dir);
  }
  // if current is a via, then reset tlength
  if (dir == frDirEnum::U || dir == frDirEnum::D) {
    nextTLength = std::numeric_limits<frCoord>::max();
  }

  FlexWavefrontGrid nextWavefrontGrid(gridX,
                                      gridY,
                                      gridZ,
                                      nextVLengthX,
                                      nextVLengthY,
                                      nextIsPrevViaUp,
                                      nextTLength,
                                      currDist,
                                      nextPathCost,
                                      nextPathCost + nextEstCost,
                                      currGrid.getBackTraceBuffer());
  if (dir == frDirEnum::U || dir == frDirEnum::D) {
    nextWavefrontGrid.resetLength();
    if (dir == frDirEnum::U) {
      nextWavefrontGrid.setPrevViaUp(false);
    } else {
      nextWavefrontGrid.setPrevViaUp(true);
    }
  }
  if (currGrid.getSrcTaperBox()
      && currGrid.getSrcTaperBox()->contains(nextWavefrontGrid.x(),
                                             nextWavefrontGrid.y(),
                                             nextWavefrontGrid.z())) {
    nextWavefrontGrid.setSrcTaperBox(currGrid.getSrcTaperBox());
  }
  // update wavefront buffer
  auto tailDir = nextWavefrontGrid.shiftAddBuffer(dir);
  // non-buffer enablement is faster for ripup all
  // commit grid prev direction if needed
  auto tailIdx = getTailIdx(nextIdx, nextWavefrontGrid);
  if (tailDir != frDirEnum::UNKNOWN) {
    if (getPrevAstarNodeDir(tailIdx) == frDirEnum::UNKNOWN
        || getPrevAstarNodeDir(tailIdx) == tailDir) {
      setPrevAstarNodeDir(tailIdx.x(), tailIdx.y(), tailIdx.z(), tailDir);
      if (debug_) {
        nextWavefrontGrid.setId(curr_id_++);
        nextWavefrontGrid.setParentId(currGrid.getId());
        printExpansion(nextWavefrontGrid, "Pushing");
      }
      wavefront_.push(nextWavefrontGrid);
    }
  } else {
    // add to wavefront
    if (debug_) {
      nextWavefrontGrid.setId(curr_id_++);
      nextWavefrontGrid.setParentId(currGrid.getId());
      printExpansion(nextWavefrontGrid, "Pushing");
    }
    wavefront_.push(nextWavefrontGrid);
  }
  if (drWorker_->getDRIter() >= debugMazeIter) {
    std::cout << "Creating " << nextWavefrontGrid.x() << " "
              << nextWavefrontGrid.y() << " " << nextWavefrontGrid.z()
              << " coords: " << xCoords_[nextWavefrontGrid.x()] << " "
              << yCoords_[nextWavefrontGrid.y()] << " cost "
              << nextWavefrontGrid.getCost() << " g "
              << nextWavefrontGrid.getPathCost() << "\n";
  }
}

void FlexGridGraph::expandWavefront(FlexWavefrontGrid& currGrid,
                                    const FlexMazeIdx& dstMazeIdx1,
                                    const FlexMazeIdx& dstMazeIdx2,
                                    const odb::Point& centerPt,
                                    bool route_with_jumpers)
{
  if (dumpingExpansion()) {
    // Owner of the isExpandable lines that follow. Uses only wavefront getters
    // and the coordinate arrays, so it is safe for any popped node.
    exp_file_ << fmt::format(
        "expanding {} {} {} pt {} {} cost {} pathCost {} lastDir {}\n",
        currGrid.x(),
        currGrid.y(),
        currGrid.z(),
        xCoords_[currGrid.x()],
        yCoords_[currGrid.y()],
        currGrid.getCost(),
        currGrid.getPathCost(),
        currGrid.getLastDir());
  }
  if (cost_file_.is_open()) {
    cost_file_ << fmt::format(
      "expanding {} {} {} pt {} {} cost {} pathCost {} lastDir {}\n",
      currGrid.x(),
      currGrid.y(),
      currGrid.z(),
      xCoords_[currGrid.x()],
      yCoords_[currGrid.y()],
      currGrid.getCost(),
      currGrid.getPathCost(),
      currGrid.getLastDir());
  }
  for (const auto dir : frDirEnumAll) {
    if (isExpandable(currGrid, dir)) {
      expand(currGrid,
             dir,
             dstMazeIdx1,
             dstMazeIdx2,
             centerPt,
             route_with_jumpers);
    }
  }
}

frCost FlexGridGraph::getEstCost(const FlexMazeIdx& src,
                                 const FlexMazeIdx& dstMazeIdx1,
                                 const FlexMazeIdx& dstMazeIdx2,
                                 const frDirEnum& dir) const
{
  int gridX = src.x();
  int gridY = src.y();
  int gridZ = src.z();
  auto edgeLength = getEdgeLength(gridX, gridY, gridZ, dir);
  getNextGrid(gridX, gridY, gridZ, dir);
  // bend cost
  int bendCnt = 0;
  int forbiddenPenalty = 0;
  odb::Point srcPoint, dstPoint1, dstPoint2;
  getPoint(srcPoint, gridX, gridY);
  getPoint(dstPoint1, dstMazeIdx1.x(), dstMazeIdx1.y());
  getPoint(dstPoint2, dstMazeIdx2.x(), dstMazeIdx2.y());
  frCoord minCostX = std::max(
      {dstPoint1.x() - srcPoint.x(), srcPoint.x() - dstPoint2.x(), 0});
  frCoord minCostY = std::max(
      {dstPoint1.y() - srcPoint.y(), srcPoint.y() - dstPoint2.y(), 0});
  frCoord minCostZ = std::max({getZHeight(dstMazeIdx1.z()) - getZHeight(gridZ),
                               getZHeight(gridZ) - getZHeight(dstMazeIdx2.z()),
                               0});

  bendCnt += (minCostX && dir != frDirEnum::UNKNOWN && dir != frDirEnum::E
              && dir != frDirEnum::W)
                 ? 1
                 : 0;
  bendCnt += (minCostY && dir != frDirEnum::UNKNOWN && dir != frDirEnum::S
              && dir != frDirEnum::N)
                 ? 1
                 : 0;
  bendCnt += (minCostZ && dir != frDirEnum::UNKNOWN && dir != frDirEnum::U
              && dir != frDirEnum::D)
                 ? 1
                 : 0;

  // If we are on the destination layer we will have to wrong way jog or
  // via up/down or down/up so add the cheapest of those to the estimate
  if (src.z() == dstMazeIdx1.z() && dstMazeIdx1.z() == dstMazeIdx2.z()) {
  }

  odb::Point nextPoint;
  getPoint(nextPoint, gridX, gridY);
  // avoid propagating to location that will cause forbidden via spacing to
  // boundary pin
  bool isForbidden = false;
  if (dstMazeIdx1.z() == dstMazeIdx2.z() && gridZ == dstMazeIdx1.z()) {
    auto layerNum = (gridZ + 1) * 2;
    auto layer = getTech()->getLayer(layerNum);
    if (!router_cfg_->USENONPREFTRACKS || layer->isUnidirectional()) {
      bool isH = layer->isHorizontal();
      if (isH && dstMazeIdx1.y() == dstMazeIdx2.y()) {
        auto gap = abs(nextPoint.y() - dstPoint1.y());
        if (gap
            && (layerNum - 2 < router_cfg_->BOTTOM_ROUTING_LAYER
                || getTech()->isVia2ViaForbiddenLen(
                    gridZ - 1, false, false, false, gap, ndr_))
            && (layerNum + 2 > getTech()->getTopLayerNum()
                || getTech()->isVia2ViaForbiddenLen(
                    gridZ + 1, true, true, false, gap, ndr_))) {
          isForbidden = true;
        }
      } else if (!isH && dstMazeIdx1.x() == dstMazeIdx2.x()) {
        auto gap = abs(nextPoint.x() - dstPoint1.x());
        if (gap
            && (layerNum - 2 < router_cfg_->BOTTOM_ROUTING_LAYER
                || getTech()->isVia2ViaForbiddenLen(
                    gridZ - 1, false, false, true, gap, ndr_))
            && (layerNum + 2 > getTech()->getTopLayerNum()
                || getTech()->isVia2ViaForbiddenLen(
                    gridZ + 1, true, true, true, gap, ndr_))) {
          isForbidden = true;
        }
      }
    }
  }
  if (isForbidden) {
    if (drWorker_->getDRIter() >= 3) {
      forbiddenPenalty = 2 * ggMarkerCost_ * edgeLength;
    } else {
      forbiddenPenalty = 2 * ggDRCCost_ * edgeLength;
    }
  }
  return (minCostX + minCostY + minCostZ + bendCnt + forbiddenPenalty);
}

frDirEnum FlexGridGraph::getLastDir(
    const std::bitset<WAVEFRONTBITSIZE>& buffer) const
{
  auto currDirVal = buffer.to_ulong() & 0b111u;
  return static_cast<frDirEnum>(currDirVal);
}

void FlexGridGraph::getNextGrid(frMIdx& gridX,
                                frMIdx& gridY,
                                frMIdx& gridZ,
                                const frDirEnum dir) const
{
  switch (dir) {
    case frDirEnum::E:
      ++gridX;
      break;
    case frDirEnum::S:
      --gridY;
      break;
    case frDirEnum::W:
      --gridX;
      break;
    case frDirEnum::N:
      ++gridY;
      break;
    case frDirEnum::U:
      ++gridZ;
      break;
    case frDirEnum::D:
      --gridZ;
      break;
    case frDirEnum::UNKNOWN:
      break;
  }
}

void FlexGridGraph::getPrevGrid(frMIdx& gridX,
                                frMIdx& gridY,
                                frMIdx& gridZ,
                                const frDirEnum dir) const
{
  switch (dir) {
    case frDirEnum::E:
      --gridX;
      break;
    case frDirEnum::S:
      ++gridY;
      break;
    case frDirEnum::W:
      ++gridX;
      break;
    case frDirEnum::N:
      --gridY;
      break;
    case frDirEnum::U:
      --gridZ;
      break;
    case frDirEnum::D:
      ++gridZ;
      break;
    case frDirEnum::UNKNOWN:
      break;
  }
}

frCost FlexGridGraph::getNextPathCost(const FlexWavefrontGrid& currGrid,
                                      const frDirEnum& dir,
                                      bool route_with_jumpers) const
{
  frMIdx gridX = currGrid.x();
  frMIdx gridY = currGrid.y();
  frMIdx gridZ = currGrid.z();
  frCost nextPathCost = currGrid.getPathCost();

  frCost initialCost = currGrid.getPathCost();
  frCost turnCost = 0;
  frCost isForbiddenVia2ViaCost = 0;
  frCost isForbiddenViaTLenCost = 0;

  frCoord edgeLength = getEdgeLength(gridX, gridY, gridZ, dir);
  // bending cost
  auto currDir = currGrid.getLastDir();
  auto lNum = getLayerNum(currGrid.z());
  auto layer = getTech()->getLayer(lNum);

  if (currDir != dir && currDir != frDirEnum::UNKNOWN) {
    // original
    ++nextPathCost;
    ++turnCost;
  }

  // via2viaForbiddenLen enablement
  if (dir == frDirEnum::U || dir == frDirEnum::D) {
    frCoord currVLengthX = 0;
    frCoord currVLengthY = 0;
    currGrid.getVLength(currVLengthX, currVLengthY);
    bool isCurrViaUp = (dir == frDirEnum::U);
    bool isForbiddenVia2Via = false;
    // check only y
    if (currVLengthX == 0 && currVLengthY > 0) {
      isForbiddenVia2Via
          = getTech()->isVia2ViaForbiddenLen(gridZ,
                                             !(currGrid.isPrevViaUp()),
                                             !isCurrViaUp,
                                             false,
                                             currVLengthY,
                                             ndr_);
      // check only x
    } else if (currVLengthX > 0 && currVLengthY == 0) {
      isForbiddenVia2Via
          = getTech()->isVia2ViaForbiddenLen(gridZ,
                                             !(currGrid.isPrevViaUp()),
                                             !isCurrViaUp,
                                             true,
                                             currVLengthX,
                                             ndr_);
      // check both x and y
    } else {
      if (getTech()->isVia2ViaPRL(gridZ,
                                  !(currGrid.isPrevViaUp()),
                                  !isCurrViaUp,
                                  false,
                                  currVLengthY)
          || getTech()->isVia2ViaPRL(gridZ,
                                     !(currGrid.isPrevViaUp()),
                                     !isCurrViaUp,
                                     true,
                                     currVLengthX)) {
        isForbiddenVia2Via
            = getTech()->isVia2ViaForbiddenLen(gridZ,
                                               !(currGrid.isPrevViaUp()),
                                               !isCurrViaUp,
                                               false,
                                               currVLengthY,
                                               ndr_)
              || getTech()->isVia2ViaForbiddenLen(gridZ,
                                                  !(currGrid.isPrevViaUp()),
                                                  !isCurrViaUp,
                                                  true,
                                                  currVLengthX,
                                                  ndr_);
      } else {
        isForbiddenVia2Via
            = getTech()->isVia2ViaForbiddenLen(gridZ,
                                               !(currGrid.isPrevViaUp()),
                                               !isCurrViaUp,
                                               false,
                                               currVLengthY,
                                               ndr_)
              && getTech()->isVia2ViaForbiddenLen(gridZ,
                                                  !(currGrid.isPrevViaUp()),
                                                  !isCurrViaUp,
                                                  true,
                                                  currVLengthX,
                                                  ndr_);
      }
    }

    if (isForbiddenVia2Via) {
      if (drWorker_) {
        if (drWorker_->getDRIter() >= debugMazeIter) {
          std::cout << "isForbiddenVia2Via\n";
        }
        if (drWorker_->getDRIter() >= 3) {
          nextPathCost += 2 * ggMarkerCost_ * edgeLength;
          isForbiddenVia2ViaCost += 2 * ggMarkerCost_ * edgeLength;
        } else {
          nextPathCost += 2 * ggDRCCost_ * edgeLength;
          isForbiddenVia2ViaCost += 2 * ggDRCCost_ * edgeLength;
        }
      }
    }
  }

  // via2turn forbidden len enablement
  frCoord tLength = std::numeric_limits<frCoord>::max();
  frCoord tLengthDummy = 0;
  bool isTLengthViaUp = false;
  bool isForbiddenTLen = false;
  if (currDir != frDirEnum::UNKNOWN && currDir != dir) {
    // next dir is a via
    if (dir == frDirEnum::U || dir == frDirEnum::D) {
      isTLengthViaUp = (dir == frDirEnum::U);
      // if there was a turn before
      if (currDir == frDirEnum::W || currDir == frDirEnum::E) {
        tLength = currGrid.getTLength();
        if (getTech()->isViaForbiddenTurnLen(
                gridZ, !isTLengthViaUp, true, tLength, ndr_)) {
          isForbiddenTLen = true;
        }
      } else if (currDir == frDirEnum::S || currDir == frDirEnum::N) {
        tLength = currGrid.getTLength();
        if (getTech()->isViaForbiddenTurnLen(
                gridZ, !isTLengthViaUp, false, tLength, ndr_)) {
          isForbiddenTLen = true;
        }
      }
      // curr is a planar turn
    } else {
      isTLengthViaUp = currGrid.isPrevViaUp();
      if (currDir == frDirEnum::W || currDir == frDirEnum::E) {
        currGrid.getVLength(tLength, tLengthDummy);
        if (getTech()->isViaForbiddenTurnLen(
                gridZ, !isTLengthViaUp, true, tLength, ndr_)) {
          isForbiddenTLen = true;
        }
      } else if (currDir == frDirEnum::S || currDir == frDirEnum::N) {
        currGrid.getVLength(tLengthDummy, tLength);
        if (getTech()->isViaForbiddenTurnLen(
                gridZ, !isTLengthViaUp, false, tLength, ndr_)) {
          isForbiddenTLen = true;
        }
      }
    }
    if (isForbiddenTLen) {
      if (drWorker_) {
        if (drWorker_->getDRIter() >= debugMazeIter) {
          std::cout << "isForbiddenTLen\n";
        }
        if (drWorker_->getDRIter() >= 3) {
          nextPathCost += 2 * ggDRCCost_ * edgeLength;
          isForbiddenViaTLenCost += 2 * ggDRCCost_ * edgeLength;
        } else {
          nextPathCost += 2 * ggMarkerCost_ * edgeLength;
          isForbiddenViaTLenCost += 2 * ggMarkerCost_ * edgeLength;
        }
      }
    }
  }

  nextPathCost += getCosts(gridX,
                           gridY,
                           gridZ,
                           dir,
                           layer,
                           useNDRCosts(currGrid),
                           route_with_jumpers);

  if (cost_file_.is_open()) {
    cost_file_ << fmt::format(" coords {} {} {} currPathCosts {} currDir {} nextDir {} edgeLength {} turnCost {} v2v {} vtlen {} finalNextCost {} \n",
                              gridX,
                              gridY,
                              gridZ,
                              initialCost,              
                              currDir,           
                              dir,               
                              edgeLength,      
                              turnCost,                    
                              isForbiddenVia2ViaCost,        
                              isForbiddenViaTLenCost, 
                              nextPathCost          
                              );           
  }

  return nextPathCost;
}

frCost FlexGridGraph::getCosts(frMIdx gridX,
                               frMIdx gridY,
                               frMIdx gridZ,
                               frDirEnum dir,
                               frLayer* layer,
                               bool considerNDR,
                               bool route_with_jumpers) const
{
  bool gridCost = hasGridCost(gridX, gridY, gridZ, dir);
  bool apCost = hasApCost(gridX, gridY, gridZ, dir);
  bool drcCost = hasRouteShapeCostAdj(gridX, gridY, gridZ, dir, considerNDR);
  bool markerCost = hasMarkerCostAdj(gridX, gridY, gridZ, dir);
  bool shapeCost = hasFixedShapeCostAdj(gridX, gridY, gridZ, dir, considerNDR);
  bool blockCost = isBlocked(gridX, gridY, gridZ, dir);
  bool guideCost = hasGuide(gridX, gridY, gridZ, dir);
  frCoord edgeLength = getEdgeLength(gridX, gridY, gridZ, dir);

  // Cost components
  frCost c_wire   = edgeLength;
  frCost c_grid   = (gridCost || apCost) ? (router_cfg_->GRIDCOST * edgeLength) : 0;
  frCost c_drc    = drcCost ? (ggDRCCost_ * edgeLength) : 0;
  frCost c_marker = markerCost ? (ggMarkerCost_ * edgeLength) : 0;
  frCost c_shape  = shapeCost ? (ggFixedShapeCost_ * edgeLength) : 0;
  
  frCoord minWidth = layer ? layer->getMinWidth() : 0;
  frCost c_block  = blockCost ? (router_cfg_->BLOCKCOST * minWidth * 20) : 0;

  // Increase cost when a net has jumper
  frUInt4 jumper_cost = route_with_jumpers ? 10 : 1;
  frCost c_guide  = !guideCost ? ((router_cfg_->GUIDECOST * jumper_cost) * edgeLength) : 0;

  frCost totalBaseCost = c_wire + c_grid + c_drc + c_marker + c_shape + c_block + c_guide;

  if (cost_file_.is_open()) {
    cost_file_ << fmt::format(
        "coords {} {} {} dir {} edgeLen {} "
        "flags[grid:{} ap:{} drc:{} marker:{} shape:{} block:{} out_guide:{}] "
        "costs[wire:{} grid:{} drc:{} marker:{} shape:{} block:{} guide:{}] "
        "totalBaseCost {}\n",
        gridX,
        gridY,
        gridZ,
        static_cast<int>(dir),
        edgeLength,
        gridCost ? 1 : 0,
        apCost ? 1 : 0,
        drcCost ? 1 : 0,
        markerCost ? 1 : 0,
        shapeCost ? 1 : 0,
        blockCost ? 1 : 0,
        !guideCost ? 1 : 0,
        c_wire,
        c_grid,
        c_drc,
        c_marker,
        c_shape,
        c_block,
        c_guide,
        totalBaseCost);
  }

  // temporarily disable guideCost
  return getEdgeLength(gridX, gridY, gridZ, dir)
         + (gridCost || apCost ? router_cfg_->GRIDCOST * edgeLength : 0)
         + (drcCost ? ggDRCCost_ * edgeLength : 0)
         + (markerCost ? ggMarkerCost_ * edgeLength : 0)
         + (shapeCost ? ggFixedShapeCost_ * edgeLength : 0)
         + (blockCost ? router_cfg_->BLOCKCOST * layer->getMinWidth() * 20 : 0)
         + (!guideCost ? (router_cfg_->GUIDECOST * jumper_cost) * edgeLength
                       : 0);
}

bool FlexGridGraph::useNDRCosts(const FlexWavefrontGrid& p) const
{
  if (ndr_) {
    if (p.getSrcTaperBox()
        && p.getSrcTaperBox()->contains(p.x(), p.y(), p.z())) {
      return false;
    }
    if (dstTaperBox_ && dstTaperBox_->contains(p.x(), p.y(), p.z())) {
      return false;
    }
    return true;
  }
  return false;
}

frMIdx FlexGridGraph::getLowerBoundIndex(const frVector<frCoord>& tracks,
                                         frCoord v) const
{
  return std::ranges::lower_bound(tracks, v) - tracks.begin();
}

frMIdx FlexGridGraph::getUpperBoundIndex(const frVector<frCoord>& tracks,
                                         frCoord v) const
{
  auto it = std::ranges::upper_bound(tracks, v);
  if (it == tracks.end()) {
    it = std::prev(it);
  }
  return it - tracks.begin();
}

FlexMazeIdx FlexGridGraph::getTailIdx(const FlexMazeIdx& currIdx,
                                      const FlexWavefrontGrid& currGrid) const
{
  int gridX = currIdx.x();
  int gridY = currIdx.y();
  int gridZ = currIdx.z();
  auto backTraceBuffer = currGrid.getBackTraceBuffer();
  for (int i = 0; i < WAVEFRONTBUFFERSIZE; ++i) {
    int currDirVal
        = backTraceBuffer.to_ulong()
          - ((backTraceBuffer.to_ulong() >> DIRBITSIZE) << DIRBITSIZE);
    frDirEnum currDir = static_cast<frDirEnum>(currDirVal);
    backTraceBuffer >>= DIRBITSIZE;
    getPrevGrid(gridX, gridY, gridZ, currDir);
  }
  return FlexMazeIdx(gridX, gridY, gridZ);
}

bool FlexGridGraph::isExpandable(const FlexWavefrontGrid& currGrid,
                                 frDirEnum dir) const
{
  frMIdx gridX = currGrid.x();
  frMIdx gridY = currGrid.y();
  frMIdx gridZ = currGrid.z();
  const frDirEnum expandDir = dir;  // reverse() flips dir in place below
  bool hg = hasEdge(gridX, gridY, gridZ, dir);
  reverse(gridX, gridY, gridZ, dir);

  if (dumpingExpansion()) {
    exp_file_ << fmt::format(
        "  isExpandable dir {} from {} {} {} to {} {} {} hasEdge {} nextIsSrc "
        "{} nextPrevDir {} lastDir {} revDir {} cameFromNext {}\n",
        expandDir,
        currGrid.x(),
        currGrid.y(),
        currGrid.z(),
        gridX,
        gridY,
        gridZ,
        hg,
        hg && isSrc(gridX, gridY, gridZ),
        hg ? getPrevAstarNodeDir({gridX, gridY, gridZ}) : frDirEnum::UNKNOWN,
        currGrid.getLastDir(),
        dir,
        currGrid.getLastDir() == dir);
  }
  
  if (!hg || isSrc(gridX, gridY, gridZ)
      || (getPrevAstarNodeDir({gridX, gridY, gridZ}) != frDirEnum::UNKNOWN)
      ||  // comment out for non-buffer enablement
      currGrid.getLastDir() == dir) {
    if (dumpingExpansion()) {
      exp_file_ << fmt::format(
          "  isExpandable result 0 reason {}\n",
          !hg                              ? "noEdge"
          : isSrc(gridX, gridY, gridZ)     ? "nextIsSrc"
          : (getPrevAstarNodeDir({gridX, gridY, gridZ}) != frDirEnum::UNKNOWN)
              ? "alreadyExpanded"
              : "cameFromNext");
    }
    return false;
  }
  if (ndr_) {
    frCoord halfWidth
        = (frCoord) getTech()->getLayer(getLayerNum(currGrid.z()))->getWidth()
          / 2;
    if (ndr_->getWidth(currGrid.z()) > 2 * halfWidth
        && !isSrc(currGrid.x(), currGrid.y(), currGrid.z())) {
      halfWidth = ndr_->getWidth(currGrid.z()) / 2;
      // if the expansion goes parallel to a die border and the wire goes out of
      // the die box, forbid expansion
      if (dir == frDirEnum::N || dir == frDirEnum::S) {
        if (xCoords_[currGrid.x()] - halfWidth < dieBox_.xMin()
            || xCoords_[currGrid.x()] + halfWidth > dieBox_.xMax()) {
          if (dumpingExpansion()) {
            exp_file_ << fmt::format(
                "  isExpandable result 0 reason ndrOutOfDieX halfWidth {}\n",
                halfWidth);
          }
          return false;
        }
      } else if (dir == frDirEnum::E || dir == frDirEnum::W) {
        if (yCoords_[currGrid.y()] - halfWidth < dieBox_.yMin()
            || yCoords_[currGrid.y()] + halfWidth > dieBox_.yMax()) {
          if (dumpingExpansion()) {
            exp_file_ << fmt::format(
                "  isExpandable result 0 reason ndrOutOfDieY halfWidth {}\n",
                halfWidth);
          }
          return false;
        }
      }
    }
  }

  if (dumpingExpansion()) {
    exp_file_ << "  isExpandable result 1 reason ok\n";
  }
  return true;
}

void FlexGridGraph::traceBackPath(const FlexWavefrontGrid& currGrid,
                                  std::vector<FlexMazeIdx>& path,
                                  std::vector<FlexMazeIdx>& root,
                                  FlexMazeIdx& ccMazeIdx1,
                                  FlexMazeIdx& ccMazeIdx2) const
{
  frDirEnum prevDir = frDirEnum::UNKNOWN, currDir = frDirEnum::UNKNOWN;
  int currX = currGrid.x(), currY = currGrid.y(), currZ = currGrid.z();
  // pop content in buffer
  auto backTraceBuffer = currGrid.getBackTraceBuffer();
  for (int i = 0; i < WAVEFRONTBUFFERSIZE; ++i) {
    // current grid is src
    if (isSrc(currX, currY, currZ)) {
      break;
    }
    // get last direction
    currDir = getLastDir(backTraceBuffer);
    backTraceBuffer >>= DIRBITSIZE;
    if (currDir == frDirEnum::UNKNOWN) {
      std::cout << "Warning: unexpected direction in tracBackPath\n";
      break;
    }
    root.emplace_back(currX, currY, currZ);
    // push point to path
    if (currDir != prevDir) {
      path.emplace_back(currX, currY, currZ);
    }
    getPrevGrid(currX, currY, currZ, currDir);
    prevDir = currDir;
  }
  // trace back according to grid prev dir
  while (!isSrc(currX, currY, currZ)) {
    // get last direction
    currDir = getPrevAstarNodeDir({currX, currY, currZ});
    root.emplace_back(currX, currY, currZ);
    if (currDir == frDirEnum::UNKNOWN) {
      std::cout << "Warning: unexpected direction in tracBackPath\n";
      break;
    }
    if (currDir != prevDir) {
      path.emplace_back(currX, currY, currZ);
    }
    getPrevGrid(currX, currY, currZ, currDir);
    prevDir = currDir;
  }
  // add final path to src, only add when path exists; no path exists (src =
  // dst)
  if (!path.empty()) {
    path.emplace_back(currX, currY, currZ);
  }
  for (auto& mi : path) {
    ccMazeIdx1.set(std::min(ccMazeIdx1.x(), mi.x()),
                   std::min(ccMazeIdx1.y(), mi.y()),
                   std::min(ccMazeIdx1.z(), mi.z()));
    ccMazeIdx2.set(std::max(ccMazeIdx2.x(), mi.x()),
                   std::max(ccMazeIdx2.y(), mi.y()),
                   std::max(ccMazeIdx2.z(), mi.z()));
  }
}

// Dumps the source/destination node sets and the traceback path for one
// search() call. No-op unless DRT_DUMP_GG_DIR points to an existing directory.
// One file per search:
//   <DRT_DUMP_GG_DIR>/search_iter<iter>_x<xMin>_y<yMin>_s<searchId>.txt
// Unlike dumpGridGraph() (a pre-routing snapshot), this fires during
// route_queue() and so captures the routing-time state (srcs_, dsts_, path)
// that the snapshot omits. srcs_/dsts_ are scanned from the live bitvectors;
// connComps is the source component maze indices as passed to search().
// Also records the remaining search() inputs (centerPt, route_with_jumpers,
// input cc bounding box) and outputs (cc bounding box after traceback).
void FlexGridGraph::dumpSearch(const std::vector<FlexMazeIdx>& connComps,
                               drPin* nextPin,
                               const std::vector<FlexMazeIdx>& path,
                               bool success,
                               const FlexMazeIdx& ccMazeIdx1In,
                               const FlexMazeIdx& ccMazeIdx2In,
                               const FlexMazeIdx& ccMazeIdx1Out,
                               const FlexMazeIdx& ccMazeIdx2Out,
                               const odb::Point& centerPt,
                               bool routeWithJumpers) const
{
  const char* dir = std::getenv("DRT_DUMP_GG_DIR");
  if (dir == nullptr || dir[0] == '\0') {
    return;
  }
  const int id = searchDumpId_++;
  const odb::Rect& rb = drWorker_->getRouteBox();
  const int iter = drWorker_->getDRIter();
  const std::string path_str = std::string(dir) + "/search_iter"
                               + std::to_string(iter) + "_x"
                               + std::to_string(rb.xMin()) + "_y"
                               + std::to_string(rb.yMin()) + "_s"
                               + std::to_string(id) + ".txt";
  std::ofstream os(path_str.c_str());
  if (!os.is_open()) {
    logger_->warn(
        utl::DRT, 618, "Could not open search dump file {}", path_str);
    return;
  }

  frMIdx xDim, yDim, zDim;
  getDim(xDim, yDim, zDim);

  os << "version 2\n";
  os << "iter " << iter << "\n";
  os << "routeBox " << rb.xMin() << " " << rb.yMin() << " " << rb.xMax() << " "
     << rb.yMax() << "\n";
  os << "searchId " << id << "\n";
  os << "pin " << (nextPin != nullptr ? nextPin->getName() : "null") << "\n";
  os << "success " << (success ? 1 : 0) << "\n";
  os << "dim " << xDim << " " << yDim << " " << zDim << "\n";
  // Remaining search() inputs: the A* tie-break center point (DBU), the
  // jumper-routing flag, and the connected-component bounding box (maze
  // indices, ll then ur) as passed in. ccBoxOut is the same box after
  // traceBackPath() grew it over the new path; it differs from ccBoxIn only
  // on success.
  os << "centerPt " << centerPt.x() << " " << centerPt.y() << "\n";
  os << "routeWithJumpers " << (routeWithJumpers ? 1 : 0) << "\n";
  os << "ccBoxIn " << ccMazeIdx1In.x() << " " << ccMazeIdx1In.y() << " "
     << ccMazeIdx1In.z() << " " << ccMazeIdx2In.x() << " " << ccMazeIdx2In.y()
     << " " << ccMazeIdx2In.z() << "\n";
  os << "ccBoxOut " << ccMazeIdx1Out.x() << " " << ccMazeIdx1Out.y() << " "
     << ccMazeIdx1Out.z() << " " << ccMazeIdx2Out.x() << " "
     << ccMazeIdx2Out.y() << " " << ccMazeIdx2Out.z() << "\n";

  // Source component maze indices as handed to search() (the wavefront seeds).
  os << "connComps " << connComps.size() << "\n";
  for (const auto& mi : connComps) {
    os << "cc " << mi.x() << " " << mi.y() << " " << mi.z() << " "
       << xCoords_[mi.x()] << " " << yCoords_[mi.y()] << " "
       << getLayerNum(mi.z()) << "\n";
  }

  // All nodes flagged as source / destination in the live bitvectors. Each row
  // carries both maze index and DBU coords (+ layerNum) for path matching.
  int srcCount = 0, dstCount = 0;
  for (frMIdx z = 0; z < zDim; ++z) {
    for (frMIdx y = 0; y < yDim; ++y) {
      for (frMIdx x = 0; x < xDim; ++x) {
        if (isSrc(x, y, z)) {
          ++srcCount;
        }
        if (isDst(x, y, z)) {
          ++dstCount;
        }
      }
    }
  }
  os << "srcs " << srcCount << "\n";
  for (frMIdx z = 0; z < zDim; ++z) {
    for (frMIdx y = 0; y < yDim; ++y) {
      for (frMIdx x = 0; x < xDim; ++x) {
        if (isSrc(x, y, z)) {
          os << "src " << x << " " << y << " " << z << " " << xCoords_[x] << " "
             << yCoords_[y] << " " << getLayerNum(z) << "\n";
        }
      }
    }
  }
  os << "dsts " << dstCount << "\n";
  for (frMIdx z = 0; z < zDim; ++z) {
    for (frMIdx y = 0; y < yDim; ++y) {
      for (frMIdx x = 0; x < xDim; ++x) {
        if (isDst(x, y, z)) {
          os << "dst " << x << " " << y << " " << z << " " << xCoords_[x] << " "
             << yCoords_[y] << " " << getLayerNum(z) << "\n";
        }
      }
    }
  }

  // Traceback path (corner points, dst-first back toward src) produced by this
  // search. Empty when no path was found or src == dst.
  os << "path " << path.size() << "\n";
  for (const auto& mi : path) {
    os << "p " << mi.x() << " " << mi.y() << " " << mi.z() << " "
       << xCoords_[mi.x()] << " " << yCoords_[mi.y()] << " "
       << getLayerNum(mi.z()) << "\n";
  }
  os.close();
}

bool FlexGridGraph::search(std::vector<FlexMazeIdx>& connComps,
                           drPin* nextPin,
                           std::vector<FlexMazeIdx>& path,
                           FlexMazeIdx& ccMazeIdx1,
                           FlexMazeIdx& ccMazeIdx2,
                           const odb::Point& centerPt,
                           std::map<FlexMazeIdx, frBox3D*>& mazeIdx2TaperBox,
                           bool route_with_jumpers)
{
  if (debug_) {
    dump_file_.open("expansions.dump");
  }
  openExpansionDump();
  openCostDump();
  if (dumpingExpansion()) {
    // One block per search() call in this grid graph. searchId matches the
    // _s<id> suffix of the dumpSearch() file for the same search.
    exp_file_ << fmt::format(
        "search {} pin {} routeWithJumpers {} connComps {}\n",
        expSearchId_,
        nextPin != nullptr ? nextPin->getName() : std::string("null"),
        route_with_jumpers,
        connComps.size());
  }
  ++expSearchId_;
  curr_id_ = 1;
  // Snapshot the input cc bounding box for dumpSearch(); traceBackPath()
  // grows ccMazeIdx1/2 in place on success, so the values at dump time are
  // the outputs.
  const FlexMazeIdx ccMazeIdx1In = ccMazeIdx1;
  const FlexMazeIdx ccMazeIdx2In = ccMazeIdx2;
  if (drWorker_->getDRIter() >= debugMazeIter) {
    std::cout << "INIT search: target pin " << nextPin->getName()
              << "\nsource points:\n";
    for (auto& idx : connComps) {
      std::cout << idx.x() << " " << idx.y() << " " << idx.z()
                << " coords: " << xCoords_[idx.x()] << " " << yCoords_[idx.y()]
                << "\n";
    }
  }
  // prep nextPinBox
  frMIdx xDim, yDim, zDim;
  getDim(xDim, yDim, zDim);
  FlexMazeIdx dstMazeIdx1(xDim - 1, yDim - 1, zDim - 1);
  FlexMazeIdx dstMazeIdx2(0, 0, 0);
  for (auto& ap : nextPin->getAccessPatterns()) {
    FlexMazeIdx mi = ap->getMazeIdx();
    dstMazeIdx1.set(std::min(dstMazeIdx1.x(), mi.x()),
                    std::min(dstMazeIdx1.y(), mi.y()),
                    std::min(dstMazeIdx1.z(), mi.z()));
    dstMazeIdx2.set(std::max(dstMazeIdx2.x(), mi.x()),
                    std::max(dstMazeIdx2.y(), mi.y()),
                    std::max(dstMazeIdx2.z(), mi.z()));
  }

  wavefront_.cleanup();
  // init wavefront
  odb::Point currPt;
  for (auto& idx : connComps) {
    if (isDst(idx.x(), idx.y(), idx.z())) {
      path.emplace_back(idx.x(), idx.y(), idx.z());
      dumpSearch(connComps,
                 nextPin,
                 path,
                 true,
                 ccMazeIdx1In,
                 ccMazeIdx2In,
                 ccMazeIdx1,
                 ccMazeIdx2,
                 centerPt,
                 route_with_jumpers);
      return true;
    }
    getPoint(currPt, idx.x(), idx.y());
    frCoord currDist = odb::Point::manhattanDistance(currPt, centerPt);
    FlexWavefrontGrid currGrid(
        idx.x(),
        idx.y(),
        idx.z(),
        std::numeric_limits<frCoord>::max(),
        std::numeric_limits<frCoord>::max(),
        true,
        std::numeric_limits<frCoord>::max(),
        currDist,
        0,
        getEstCost(idx, dstMazeIdx1, dstMazeIdx2, frDirEnum::UNKNOWN));
    if (ndr_ && router_cfg_->AUTO_TAPER_NDR_NETS) {
      auto it = mazeIdx2TaperBox.find(idx);
      if (it != mazeIdx2TaperBox.end()) {
        currGrid.setSrcTaperBox(it->second);
      }
    }
    if (debug_) {
      currGrid.setId(curr_id_++);
      printExpansion(currGrid, "Pushing");
    }
    wavefront_.push(currGrid);
  }
  while (!wavefront_.empty()) {
    auto currGrid = wavefront_.top();
    if (debug_) {
      printExpansion(currGrid, "Popping");
    }
    wavefront_.pop();
    if (getPrevAstarNodeDir({currGrid.x(), currGrid.y(), currGrid.z()})
        != frDirEnum::UNKNOWN) {
      continue;
    }
    if (graphics_) {
      graphics_->searchNode(this, currGrid);
    }
    if (drWorker_->getDRIter() >= debugMazeIter) {
      std::cout << "Expanding " << currGrid.x() << " " << currGrid.y() << " "
                << currGrid.z() << " coords: " << xCoords_[currGrid.x()] << " "
                << yCoords_[currGrid.y()] << " cost " << currGrid.getCost()
                << " g " << currGrid.getPathCost() << "\n";
    }
    if (isDst(currGrid.x(), currGrid.y(), currGrid.z())) {
      traceBackPath(currGrid, path, connComps, ccMazeIdx1, ccMazeIdx2);
      dumpSearch(connComps,
                 nextPin,
                 path,
                 true,
                 ccMazeIdx1In,
                 ccMazeIdx2In,
                 ccMazeIdx1,
                 ccMazeIdx2,
                 centerPt,
                 route_with_jumpers);
      return true;
    }
    // expand and update wavefront
    expandWavefront(
        currGrid, dstMazeIdx1, dstMazeIdx2, centerPt, route_with_jumpers);
  }
  dumpSearch(connComps,
             nextPin,
             path,
             false,
             ccMazeIdx1In,
             ccMazeIdx2In,
             ccMazeIdx1,
             ccMazeIdx2,
             centerPt,
             route_with_jumpers);
  return false;
}

}  // namespace drt
