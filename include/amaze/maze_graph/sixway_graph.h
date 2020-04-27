/**
 * Copyright (c) 2020 Tokoro
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef INCLUDE_AMAZE_MAZE_GRAPH_SIXWAY_GRAPH_H_
#define INCLUDE_AMAZE_MAZE_GRAPH_SIXWAY_GRAPH_H_

#include <algorithm>
#include <utility>
#include <vector>

#include "amaze/common/common_types.h"
#include "amaze/common/common_utils.h"
#include "amaze/maze_graph/maze_graph_base.h"

namespace amaze {
namespace maze_graph {

/// \~japanese
/// 迷路の壁ノード6方向グラフ表現．
///
/// このグラフ表現では壁の座標にノードを置き，隣接する壁どうしをつなぐ
/// 6方向のエッジを考えます．
///
/// \tparam TCost コストの型
/// \tparam TNodeId ノードIDの型
/// \tparam W 迷路の幅
///
/// \~english
/// A 6-way wall node graph representation of the maze.
///
/// This representation considers nodes at wall coordinates and 6 edges from
/// each node towards adjacent walls.
///
/// \tparam TCost Type of the cost
/// \tparam TNodeId Type of the node ID
/// \tparam W Maze width
template <bool kExplore = true, typename TCost = uint16_t,
          typename TNodeId = uint16_t, uint8_t W = kDefaultMazeWidth,
          TNodeId NodeCount = TNodeId(2 * W * (W - 1))>
class SixWayGraph : public MazeGraphBase<TCost, TNodeId, W, NodeCount> {
  static_assert(NodeCount == TNodeId(2 * W * (W - 1)),
                "The template parameter NodeCount has invalid value.");

 public:
  using Base = MazeGraphBase<TCost, TNodeId, W, NodeCount>;
  using Base::edge;
  using Base::edgeWithHypothesis;
  using Base::wallPositionOnEdge;

  using Base::kInvalidNode;
  using Base::kSize;

  explicit SixWayGraph(const Maze<W> &maze) : Base(maze) {}
  TCost distance(TNodeId id_from, TNodeId id_to) const override {
    if (id_from >= kSize || id_to >= kSize) /* [[unlikely]] */ {
      // TODO(tokoro10g): implement exception handling
#if 0
      std::cerr << "Out of bounds!!! (id_from: " << static_cast<int>(id_from)
                << ", id_to: " << static_cast<int>(id_to) << ") " << __FILE__
                << ":" << __LINE__ << std::endl;
#endif
      return Base::kInf;
    }
    auto as1 = agentStateByNodeId(id_from);
    auto as2 = agentStateByNodeId(id_to);
    return TCost(abs(static_cast<int>(as1.pos.x) - as2.pos.x) +
                 abs(static_cast<int>(as1.pos.y) - as2.pos.y));
  }
  std::vector<EdgeTo<TNodeId, TCost>> neighborEdges(
      TNodeId id,
      std::unordered_map<Position, bool> wall_overrides = {}) const override {
    std::vector<EdgeTo<TNodeId, TCost>> v;
    if (id >= kSize) /* [[unlikely]] */ {
      // TODO(tokoro10g): implement exception handling
#if 0
      std::cerr << "Out of bounds!!! (id: " << static_cast<int>(id) << ") "
                << __FILE__ << ":" << __LINE__ << std::endl;
#endif
      return v;  // returns empty
    }
    constexpr int8_t dx[8] = {0, 1, 2, 1, 0, -1, -2, -1};
    constexpr int8_t dy[8] = {2, 1, 0, -1, -2, -1, 0, 1};
    auto as = agentStateByNodeId(id);
    for (int i = 0; i < 8; i++) {
      if (as.pos.x + dx[i] < 0) continue;
      if (as.pos.y + dy[i] < 0) continue;
      if (as.pos.x + dx[i] > 2 * (W - 1)) continue;
      if (as.pos.y + dy[i] > 2 * (W - 1)) continue;
      auto as_tmp = as;
      as_tmp.pos.x = uint8_t(as_tmp.pos.x + dx[i]);
      as_tmp.pos.y = uint8_t(as_tmp.pos.y + dy[i]);
      if (!kExplore && (!Base::maze.isCheckedWall(as.pos) ||
                        !Base::maze.isCheckedWall(as_tmp.pos))) {
        continue;
      }

      auto it = wall_overrides.find(as_tmp.pos);
      bool wall_state;
      if (it == wall_overrides.end()) {
        wall_state =
            Base::maze.isSetWall(as_tmp.pos) || Base::maze.isSetWall(as.pos);
      } else {
        wall_state = it->second;
      }
      auto e = edgeWithHypothesis(as, as_tmp, wall_state);
      if (e.id != kInvalidNode) {
        v.push_back(e);
      }
    }
    return v;
  }
  Position wallPositionOnEdge(AgentState from, AgentState to) const override {
    if (!Base::edgeExist(from, to)) {
      return kInvalidAgentState.pos;
    }

    return to.pos;
  }
  EdgeTo<TNodeId, TCost> edgeWithHypothesis(AgentState as1, AgentState as2,
                                            bool blocked) const override {
    if (as1 == kInvalidAgentState ||
        as2 == kInvalidAgentState) /* [[unlikely]] */ {
#if 0
      std::cerr << "Out of bounds!!! (from: " << as1 << ", to: " << as2 << ") "
                << __FILE__ << ":" << __LINE__ << std::endl;
#endif
      return {kInvalidNode, Base::kInf};
    }

    TCost maxcost = 0;
    if (blocked) {
      maxcost = Base::kInf;
    }
    if (as1.pos.type() == PositionType::kWall &&
        abs(static_cast<int>(as1.pos.x) - as2.pos.x) == 1 &&
        abs(static_cast<int>(as1.pos.y) - as2.pos.y) == 1) {
      // diagonal path
      return {nodeIdByAgentState(as2), std::max(TCost(2), maxcost)};
    } else if ((abs(static_cast<int>(as1.pos.x) - as2.pos.x) == 2 &&
                as1.pos.y == as2.pos.y && as1.pos.x % 2 == 1 &&
                as1.pos.y % 2 == 0) ||
               (abs(static_cast<int>(as1.pos.y) - as2.pos.y) == 2 &&
                as1.pos.x == as2.pos.x && as1.pos.x % 2 == 0 &&
                as1.pos.y % 2 == 1)) {
      // straight path
      return {nodeIdByAgentState(as2), std::max(TCost(3), maxcost)};
    }
    return {kInvalidNode, Base::kInf};
  }
  EdgeTo<TNodeId, TCost> edge(AgentState as1, AgentState as2) const override {
    if (as1 == kInvalidAgentState ||
        as2 == kInvalidAgentState) /* [[unlikely]] */ {
#if 0
      std::cerr << "Out of bounds!!! (from: " << as1 << ", to: " << as2 << ") "
                << __FILE__ << ":" << __LINE__ << std::endl;
#endif
      return {kInvalidNode, Base::kInf};
    }
    if (!kExplore && (!Base::maze.isCheckedWall(as1.pos) ||
                      !Base::maze.isCheckedWall(as2.pos))) {
      return {kInvalidNode, Base::kInf};
    }
    return edgeWithHypothesis(
        as1, as2,
        Base::maze.isSetWall(as1.pos) || Base::maze.isSetWall(as2.pos));
  }
  TNodeId nodeIdByAgentState(AgentState as) const override {
    if (as.pos.type() != PositionType::kWall || as.pos.x > 2 * W ||
        as.pos.y > 2 * W) /* [[unlikely]] */ {
      // cell, pillar, or out of range
      return Base::kInvalidNode;
    }
    if (as.pos.y % 2 == 0) {
      // East node
      return TNodeId(as.pos.y / 2 * (2 * W - 1) + as.pos.x / 2);
    } else {
      // North node
      return TNodeId(as.pos.y / 2 * (2 * W - 1) + as.pos.x / 2 + W - 1);
    }
  }
  std::set<TNodeId> nodeIdsByPosition(Position p) const override {
    std::set<TNodeId> ids;
    TNodeId id = nodeIdByAgentState({p, kNoDirection, 0});
    if (id != Base::kInvalidNode) {
      ids.insert(id);
    }
    return ids;
  }
  AgentState agentStateByNodeId(TNodeId id) const override {
    if (id >= kSize) /* [[unlikely]] */ {
      // TODO(tokoro10g): implement exception handling
#if 0
      std::cerr << "Out of bounds!!! (id: " << static_cast<int>(id) << ") "
                << __FILE__ << ":" << __LINE__ << std::endl;
#endif
      return kInvalidAgentState;
    }
    AgentState as;
    TNodeId tmp = TNodeId(id % (2 * W - 1));
    TNodeId tmp2 = TNodeId(id / (2 * W - 1));
    bool isNorth = tmp > W - 2;
    if (isNorth) {
      as.pos.x = uint8_t((tmp - W + 1) * 2);
      as.pos.y = uint8_t(tmp2 * 2 + 1);
    } else {
      as.pos.x = uint8_t(tmp * 2 + 1);
      as.pos.y = uint8_t(tmp2 * 2);
    }
    as.dir = kNoDirection;
    as.attribute = 0;
    return as;
  }
  AgentState agentStateByEdge(TNodeId id_from, TNodeId id_to) const override {
    if (!Base::edgeExist(id_from, id_to)) {
      return kInvalidAgentState;
    }
    auto as1 = agentStateByNodeId(id_from);
    auto as2 = agentStateByNodeId(id_to);
    auto ret = as2;
    auto d = as2.pos - as1.pos;
    if (d.x == 0 && abs(d.y) == 2) {
      if (d.y < 0) {
        ret.dir = kSouth;
      } else {
        ret.dir = kNorth;
      }
    } else if (d.y == 0 && abs(d.x) == 2) {
      if (d.x < 0) {
        ret.dir = kWest;
      } else {
        ret.dir = kEast;
      }
    } else if (as1.pos.y % 2 == 0 /* from east node */ && abs(d.x) == 1 &&
               abs(d.y) == 1) {
      if (d.y < 0) {
        ret.dir = kSouth;
      } else {
        ret.dir = kNorth;
      }
    } else if (as1.pos.x % 2 == 0 /* from north node */ && abs(d.x) == 1 &&
               abs(d.y) == 1) {
      if (d.x < 0) {
        ret.dir = kWest;
      } else {
        ret.dir = kEast;
      }
    }
    return ret;
  }

  bool isPullBackSequence(std::array<TNodeId, 3> seq) const override {
    if (!Base::edgeExist(seq[0], seq[1]) || !Base::edgeExist(seq[1], seq[2])) {
      return false;
    }
    return (seq[0] == seq[2] || Base::edgeExist(seq[0], seq[2]));
  }

  TNodeId startNodeId() const override {
    auto p = Base::maze.start;
    if (p.type() == PositionType::kCell) {
      // \~japanese
      // エージェントは常に北側の壁からスタートするという仮定を設けています．
      // 別の言い方をすると，スタート時点の進行方向を北と定義し，座標系もそれにしたがって決めています．
      //
      // \~english
      // This assumes that the agent always starts from the north wall.
      // In other words, the initial direction is defined as north, and the
      // coordinates are determined accordingly.
      p.y++;
    }
    return nodeIdByAgentState({p, kNoDirection, 0});
  }
};

}  // namespace maze_graph
}  // namespace amaze
#endif  // INCLUDE_AMAZE_MAZE_GRAPH_SIXWAY_GRAPH_H_
