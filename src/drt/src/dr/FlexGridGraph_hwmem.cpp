// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// The A* accelerator's inputs for one search() call, written straight from
// the grid graph. This is the same output initialization/gg_to_mem.py and
// a-star-accelerator/tests/pack_gg_nodes.py make from the gg.txt / search
// dumps, minus the intermediates the simulation never reads (the unpacked
// node banks and path_cost.txt); gg_to_mem.py's docstring is the reference
// for every layout below.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "db/tech/frLayer.h"
#include "dr/AstarHwTrial.h"
#include "dr/FlexDR.h"
#include "dr/FlexGridGraph.h"
#include "frBaseTypes.h"
#include "global.h"

namespace drt {

namespace {

// gg_to_mem.py bit numbers of the node word.
constexpr int kGuideBit = 21;
constexpr int kSrcBit = 22;
constexpr int kDstBit = 23;
constexpr int kNodeBits = 24;
// pack_gg_nodes.py: three node words per memory word, slot 0 in the low bits.
constexpr int kNodesPerWord = 3;
// gg_to_mem.py SECONDARY_NODE_FIELDS: the node-word bit behind each bit of
// the secondary word, LSB first (src, guide, the seven non-NDR costs from
// fixedShapeCostPlanarVert down to routeShapeCostPlanar, override).
constexpr int kSecondarySource[] = {22, 21, 20, 19, 18, 17, 16, 15, 14, 7};
constexpr int kSecondaryBits = 12;

bool writeFile(const std::string& path, const std::string& text)
{
  std::ofstream os(path, std::ios::binary);
  os.write(text.data(), static_cast<std::streamsize>(text.size()));
  return static_cast<bool>(os);
}

std::string memHeader(const std::string& name,
                      size_t words,
                      int bits,
                      const std::string& what)
{
  return "// " + name + " -- " + std::to_string(words) + " words x "
         + std::to_string(bits) + " bits, " + what
         + "\n// written by FlexGridGraph::writeHwMem(); layout as "
           "gg_to_mem.py\n";
}

}  // namespace

std::string FlexGridGraph::writeHwMem(const std::string& dir,
                                      const FlexMazeIdx& dstMazeIdx1,
                                      const FlexMazeIdx& dstMazeIdx2,
                                      const bool routeWithJumpers) const
{
  const astar_hw::TechImages& tech = astar_hw::techImages();
  if (!tech.valid) {
    return "no_tech_images";
  }
  frMIdx xDim, yDim, zDim;
  getDim(xDim, yDim, zDim);
  // The forbidden tables are indexed by the same z as the grid graph.
  if (tech.numLayers != zDim
      || static_cast<int>(tech.layerNums.size()) < zDim) {
    return "layer_count_mismatch";
  }
  for (frMIdx z = 0; z < zDim; ++z) {
    if (tech.layerNums[z] != zCoords_[z]) {
      return "layer_num_mismatch";
    }
  }
  const frMIdx dims[] = {xDim, yDim, zDim};
  const frMIdx lo[] = {dstMazeIdx1.x(), dstMazeIdx1.y(), dstMazeIdx1.z()};
  const frMIdx hi[] = {dstMazeIdx2.x(), dstMazeIdx2.y(), dstMazeIdx2.z()};
  for (int axis = 0; axis < 3; ++axis) {
    if (lo[axis] < 0 || hi[axis] >= dims[axis] || lo[axis] > hi[axis]) {
      return "bad_goal_box";
    }
  }

  // --- node words, canonical addr = (z * yDim + y) * xDim + x ---
  // Every flag is read live. The one difference from gg_to_mem.py is
  // hasSpecialVia: it takes that from gg.txt, written at the end of init(),
  // before initMazeCost_ap() marks the special-via access points. Neither the
  // A* nor the RTL reads the bit.
  const size_t numNodes = static_cast<size_t>(xDim) * yDim * zDim;
  std::vector<uint32_t> words(numNodes);
  std::vector<FlexMazeIdx> srcs;
  std::vector<FlexMazeIdx> dsts;
  size_t addr = 0;
  for (frMIdx z = 0; z < zDim; ++z) {
    for (frMIdx y = 0; y < yDim; ++y) {
      for (frMIdx x = 0; x < xDim; ++x, ++addr) {
        const auto idx = getIdx(x, y, z);
        const Node& n = nodes_[idx];
        // The accelerator has no NDR cost fields; a non-NDR net never reads
        // them (useNDRCosts()), so they must all be zero here.
        if (n.routeShapeCostPlanarNDR || n.routeShapeCostViaNDR
            || n.fixedShapeCostViaNDR || n.fixedShapeCostPlanarHorzNDR
            || n.fixedShapeCostPlanarVertNDR) {
          return "ndr_cost_fields";
        }
        uint32_t w
            = static_cast<uint32_t>(n.hasEastEdge)
              | static_cast<uint32_t>(n.hasNorthEdge) << 1
              | static_cast<uint32_t>(n.hasUpEdge) << 2
              | static_cast<uint32_t>(n.isBlockedEast) << 3
              | static_cast<uint32_t>(n.isBlockedNorth) << 4
              | static_cast<uint32_t>(n.isBlockedUp) << 5
              | static_cast<uint32_t>(n.hasSpecialVia) << 6
              | static_cast<uint32_t>(n.overrideShapeCostVia) << 7
              | static_cast<uint32_t>(n.hasGridCostEast) << 8
              | static_cast<uint32_t>(n.hasGridCostNorth) << 9
              | static_cast<uint32_t>(n.hasGridCostUp) << 10
              | static_cast<uint32_t>(n.hasApCostEast) << 11
              | static_cast<uint32_t>(n.hasApCostNorth) << 12
              | static_cast<uint32_t>(n.hasApCostUp) << 13
              | static_cast<uint32_t>(n.routeShapeCostPlanar != 0) << 14
              | static_cast<uint32_t>(n.routeShapeCostVia != 0) << 15
              | static_cast<uint32_t>(n.markerCostPlanar != 0) << 16
              | static_cast<uint32_t>(n.markerCostVia != 0) << 17
              | static_cast<uint32_t>(n.fixedShapeCostVia != 0) << 18
              | static_cast<uint32_t>(n.fixedShapeCostPlanarHorz != 0) << 19
              | static_cast<uint32_t>(n.fixedShapeCostPlanarVert != 0) << 20;
        if (guides_[idx]) {
          w |= 1u << kGuideBit;
        }
        if (srcs_[idx]) {
          w |= 1u << kSrcBit;
          srcs.emplace_back(x, y, z);
        }
        if (dsts_[idx]) {
          w |= 1u << kDstBit;
          dsts.emplace_back(x, y, z);
        }
        words[addr] = w;
      }
    }
  }

  // --- checkerboard banks: bank = x[0] ^ (z[0] if xDim is even, else y[0]),
  // bank addr = (z * yDim + y) * ceil(xDim / 2) + x / 2 ---
  const int bankX = (xDim + 1) / 2;
  const size_t bankDepth = static_cast<size_t>(yDim) * zDim * bankX;
  std::vector<uint32_t> banks[2] = {std::vector<uint32_t>(bankDepth, 0),
                                    std::vector<uint32_t>(bankDepth, 0)};
  for (frMIdx z = 0; z < zDim; ++z) {
    for (frMIdx y = 0; y < yDim; ++y) {
      const int rowBank = (xDim % 2 == 0) ? (z & 1) : (y & 1);
      const size_t row = static_cast<size_t>(z) * yDim + y;
      for (frMIdx x = 0; x < xDim; ++x) {
        banks[(x & 1) ^ rowBank][row * bankX + x / 2] = words[row * xDim + x];
      }
    }
  }

  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  const std::string prefix = dir + "/";
  bool ok = true;
  const char* bankNames[2] = {"even", "odd"};
  const size_t rows = static_cast<size_t>(yDim) * zDim;
  const int packedPerRow = (bankX + kNodesPerWord - 1) / kNodesPerWord;
  for (int b = 0; b < 2; ++b) {
    const std::vector<uint32_t>& bank = banks[b];
    std::string text
        = memHeader(std::string("gg_nodes_") + bankNames[b] + ".mem.p3",
                    rows * packedPerRow,
                    kNodeBits * kNodesPerWord,
                    "3 nodes per word, slot 0 in the low bits");
    text.reserve(text.size() + rows * packedPerRow * 19);
    for (size_t r = 0; r < rows; ++r) {
      for (int offset = 0; offset < bankX; offset += kNodesPerWord) {
        for (int slot = kNodesPerWord - 1; slot >= 0; --slot) {
          const int i = offset + slot;
          astar_hw::appendHex(
              text, i < bankX ? bank[r * bankX + i] : 0, kNodeBits);
        }
        text += '\n';
      }
    }
    ok &= writeFile(prefix + "gg_nodes_" + bankNames[b] + ".mem.p3", text);

    text = memHeader(std::string("gg_nodes_secondary_") + bankNames[b] + ".mem",
                     bankDepth,
                     kSecondaryBits,
                     "secondary node fields");
    text.reserve(text.size() + bankDepth * 4);
    for (const uint32_t w : bank) {
      uint32_t s = 0;
      for (int bit = 0; bit < static_cast<int>(std::size(kSecondarySource));
           ++bit) {
        s |= ((w >> kSecondarySource[bit]) & 1u) << bit;
      }
      astar_hw::appendHex(text, s, kSecondaryBits);
      text += '\n';
    }
    ok &= writeFile(prefix + "gg_nodes_secondary_" + bankNames[b] + ".mem",
                    text);
  }

  // --- coordinates: one signed width over all three axes, two per word,
  // even index in the upper half ---
  int64_t minCoord = xCoords_[0];
  int64_t maxCoord = xCoords_[0];
  for (const auto* values : {&xCoords_, &yCoords_, &zHeights_}) {
    for (const frCoord c : *values) {
      minCoord = std::min<int64_t>(minCoord, c);
      maxCoord = std::max<int64_t>(maxCoord, c);
    }
  }
  const int coordBits
      = astar_hw::nibbleAlign(astar_hw::signedBits(minCoord, maxCoord));
  auto writeCoords
      = [&](const std::string& name, const frVector<frCoord>& values) {
          const size_t depth = (values.size() + 1) / 2;
          std::string text
              = memHeader(name, depth, 2 * coordBits, "signed DBU pairs");
          for (size_t i = 0; i < values.size(); i += 2) {
            astar_hw::appendHex(text, values[i], coordBits);
            astar_hw::appendHex(
                text, i + 1 < values.size() ? values[i + 1] : 0, coordBits);
            text += '\n';
          }
          ok &= writeFile(prefix + name, text);
        };
  writeCoords("gg_xcoords.mem", xCoords_);
  writeCoords("gg_ycoords.mem", yCoords_);
  writeCoords("gg_zcoords.mem", zHeights_);

  // --- layers: {unidir, odb::dbTechLayerDir, minWidth} by z ---
  std::vector<frCoord> minWidths(zDim);
  frCoord maxMinWidth = 0;
  for (frMIdx z = 0; z < zDim; ++z) {
    minWidths[z] = getTech()->getLayer(getLayerNum(z))->getMinWidth();
    if (minWidths[z] < 0) {
      return "negative_min_width";
    }
    maxMinWidth = std::max(maxMinWidth, minWidths[z]);
  }
  const int minWidthBits
      = astar_hw::nibbleAlign(astar_hw::unsignedBits(maxMinWidth));
  const int unidirLsb = minWidthBits + 2;
  const int layerBits = astar_hw::nibbleAlign(unidirLsb + 1);
  {
    std::string text = memHeader(
        "gg_layers.mem", zDim, layerBits, "{unidir, dir, minWidth} by z");
    for (frMIdx z = 0; z < zDim; ++z) {
      const uint64_t unidir
          = getTech()->getLayer(getLayerNum(z))->isUnidirectional() ? 1 : 0;
      const uint64_t layerDir = getZDir(z).getValue();
      astar_hw::appendHex(text,
                          unidir << unidirLsb | layerDir << minWidthBits
                              | static_cast<uint64_t>(minWidths[z]),
                          layerBits);
      text += '\n';
    }
    ok &= writeFile(prefix + "gg_layers.mem", text);
  }

  // --- forbidden tables, packed once by FlexRP::prepHwTechImages() ---
  const int tableDepth = tech.numLayers * 4;
  const int forbidBits = 4 * tech.forbidHalf;
  ok &= writeFile(prefix + "via2via_forbidden_len.mem",
                  memHeader("via2via_forbidden_len.mem",
                            tableDepth,
                            2 * forbidBits,
                            "isDirY pairs")
                      + tech.via2viaForbidden);
  ok &= writeFile(
      prefix + "via2via_prl_len.mem",
      memHeader(
          "via2via_prl_len.mem", tableDepth, 2 * tech.prlBits, "isDirY pairs")
          + tech.via2viaPrl);
  ok &= writeFile(prefix + "via_forbidden_turn_len.mem",
                  memHeader("via_forbidden_turn_len.mem",
                            tableDepth,
                            forbidBits,
                            "addr z*4 + viaUp*2 + isDirY")
                      + tech.viaForbiddenTurn);

  // --- src_dst_pins.txt: exactly the two blocks, no comments ---
  {
    std::string text;
    for (const auto* list : {&srcs, &dsts}) {
      const char* label = list == &srcs ? "src" : "dst";
      text += std::string(label) + "s " + std::to_string(list->size()) + "\n";
      for (const FlexMazeIdx& mi : *list) {
        text += std::string(label) + " " + std::to_string(mi.x()) + " "
                + std::to_string(mi.y()) + " " + std::to_string(mi.z()) + " "
                + std::to_string(xCoords_[mi.x()]) + " "
                + std::to_string(yCoords_[mi.y()]) + " "
                + std::to_string(zHeights_[mi.z()]) + "\n";
      }
    }
    ok &= writeFile(prefix + "src_dst_pins.txt", text);
  }

  // --- params.sv: the build-time shape only ---
  const odb::Rect& rb = drWorker_->getRouteBox();
  const std::string routeBox
      = std::to_string(rb.xMin()) + " " + std::to_string(rb.yMin()) + " "
        + std::to_string(rb.xMax()) + " " + std::to_string(rb.yMax());
  {
    struct MemSpec
    {
      const char* name;
      size_t depth;
      int bits;
    };
    const MemSpec specs[] = {
        {"GG_NODES", numNodes, kNodeBits},
        {"GG_NODES_SECONDARY", numNodes, kSecondaryBits},
        {"GG_NODES_EVEN", bankDepth, kNodeBits},
        {"GG_NODES_ODD", bankDepth, kNodeBits},
        {"GG_NODES_SECONDARY_EVEN", bankDepth, kSecondaryBits},
        {"GG_NODES_SECONDARY_ODD", bankDepth, kSecondaryBits},
        {"GG_XCOORDS", (xCoords_.size() + 1) / 2, 2 * coordBits},
        {"GG_YCOORDS", (yCoords_.size() + 1) / 2, 2 * coordBits},
        {"GG_ZCOORDS", (zHeights_.size() + 1) / 2, 2 * coordBits},
        {"GG_LAYERS", static_cast<size_t>(zDim), layerBits},
        {"VIA2VIA_FORBIDDEN_LEN",
         static_cast<size_t>(tableDepth),
         2 * forbidBits},
        {"VIA2VIA_PRL_LEN", static_cast<size_t>(tableDepth), 2 * tech.prlBits},
        {"VIA_FORBIDDEN_TURN_LEN", static_cast<size_t>(tableDepth), forbidBits},
    };
    std::string text
        = "// Generated by FlexGridGraph::writeHwMem() -- do not edit.\n"
          "// Route box (DBU): "
          + routeBox
          + "\n// Memory widths and address widths are provided below; each\n"
            "// .mem file documents its field layout in the header.\n"
            "// Per-search settings are in search_cfg.txt.\n\n"
            "// Coordinates for grid and cost size, change when grid graph "
            "changes\n"
            "package DRT_PARAMS;\n"
            "  parameter GRID_I = "
          + std::to_string(xDim) + ";\n  parameter GRID_J = "
          + std::to_string(yDim) + ";\n  parameter GRID_K = "
          + std::to_string(zDim) + ";\n\n  parameter GG_NODES_BANK_X_DEPTH = "
          + std::to_string(bankX) + ";\n  parameter GG_NODES_BANK_DEPTH = "
          + std::to_string(bankDepth)
          + ";\n"
            "  parameter string GG_NODES_EVEN_FILE_NAME = "
            "\"gg_nodes_even.mem\";\n"
            "  parameter string GG_NODES_ODD_FILE_NAME = "
            "\"gg_nodes_odd.mem\";\n"
            "  parameter string GG_NODES_SECONDARY_EVEN_FILE_NAME = "
            "\"gg_nodes_secondary_even.mem\";\n"
            "  parameter string GG_NODES_SECONDARY_ODD_FILE_NAME = "
            "\"gg_nodes_secondary_odd.mem\";\n\n"
            "  // Memory depths and BRAM port widths\n";
    for (const MemSpec& spec : specs) {
      const std::string name = spec.name;
      text += "  parameter " + name
              + "_MEM_DEPTH = " + std::to_string(spec.depth) + ";\n";
      text += "  parameter " + name
              + "_MEM_DATA_WIDTH = " + std::to_string(spec.bits) + ";\n";
      text += "  parameter " + name + "_MEM_ADDR_WIDTH = "
              + std::to_string(astar_hw::unsignedBits(spec.depth - 1)) + ";\n";
    }
    text += "endpackage\n";
    ok &= writeFile(prefix + "params.sv", text);
  }

  // --- search_cfg.txt: the per-search settings top takes on cfg_in ---
  {
    // frLayerNum -> z. getTopLayerNum() counts cut and masterslice layers
    // too, so fall back to the highest routing layer at or below it.
    auto toZ = [&](const frLayerNum layerNum) {
      int z = -1;
      for (frMIdx i = 0; i < zDim; ++i) {
        if (zCoords_[i] == layerNum) {
          return static_cast<int>(i);
        }
        if (zCoords_[i] <= layerNum) {
          z = i;
        }
      }
      return z;
    };
    const frLayerNum bottomLayer = router_cfg_->BOTTOM_ROUTING_LAYER;
    const frLayerNum topLayer = getTech()->getTopLayerNum();
    const int bottomZ = toZ(bottomLayer);
    const int topZ = toZ(topLayer);
    if (bottomZ < 0 || topZ < 0) {
      return "routing_layer_below_stack";
    }
    std::string text
        = "# Generated by FlexGridGraph::writeHwMem() -- do not edit.\n"
          "# Route box (DBU): "
          + routeBox + "\n";
    auto line = [&text](const std::string& name, const int64_t value) {
      text += name + " " + std::to_string(value) + "\n";
    };
    line("lower_goal_x", xCoords_[lo[0]]);
    line("lower_goal_y", yCoords_[lo[1]]);
    line("lower_goal_z", zHeights_[lo[2]]);
    line("upper_goal_x", xCoords_[hi[0]]);
    line("upper_goal_y", yCoords_[hi[1]]);
    line("upper_goal_z", zHeights_[hi[2]]);
    line("iter", drWorker_->getDRIter());
    line("drc_cost", ggDRCCost_);
    line("marker_cost", ggMarkerCost_);
    line("fixed_shape_cost", ggFixedShapeCost_);
    line("grid_cost", router_cfg_->GRIDCOST);
    line("block_cost", router_cfg_->BLOCKCOST);
    line("guide_cost", router_cfg_->GUIDECOST);
    // Both bounds as z: getEstCost compares layerNum -+ 2 against them, the
    // z -+ 1 neighbour under its layerNum = (z + 1) * 2 mapping.
    text += "# bottom: layer_num " + std::to_string(bottomLayer) + " = z "
            + std::to_string(bottomZ) + "\n";
    line("bottom_routing_layer", bottomZ);
    text += "# top: layer_num " + std::to_string(topLayer) + " = z "
            + std::to_string(topZ) + "\n";
    line("top_routing_layer", topZ);
    line("use_nonpref_tracks", router_cfg_->USENONPREFTRACKS ? 1 : 0);
    line("jumper_route", routeWithJumpers ? 1 : 0);
    ok &= writeFile(prefix + "search_cfg.txt", text);
  }
  return ok ? "" : "write_failed";
}

}  // namespace drt
