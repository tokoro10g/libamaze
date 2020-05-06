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

#ifndef AMAZE_SOLVER_BFS_H_
#define AMAZE_SOLVER_BFS_H_

#include <algorithm>
#include <array>
#include <bitset>
#include <queue>
#include <set>
#include <unordered_set>
#include <utility>
#include <vector>

#include "amaze/config.h"
#include "amaze/solver/solver_base.h"

#ifndef AMAZE_NO_STDIO
#include <iostream>
#endif

namespace amaze {
namespace solver {

/// \~japanese
/// 幅優先探索ソルバ
///
/// \~english
/// Breadth-first search solver
template <typename TCost, typename TNodeId, uint8_t W, TNodeId NodeCount>
class BFS : public SolverBase<TCost, TNodeId, W, NodeCount>,
            public AllAtOnceSolverBase {
 public:
  using MazeGraph = maze_graph::MazeGraphBase<TCost, TNodeId, W, NodeCount>;
  using Base = SolverBase<TCost, TNodeId, W, NodeCount>;
  using NodeId = TNodeId;
  using Cost = TCost;

  using Base::changeDestinations;
  using Base::kMaxCost;
  using Base::reconstructPath;

  explicit BFS(const MazeGraph *mg)
      : SolverBase<TCost, TNodeId, W, NodeCount>(mg),
        id_current(mg->startNodeId()),
        id_last(id_current),
        ids_destination(mg->goalNodeIds()),
        id_candidate(id_current),
        g() {
    g.fill(kMaxCost);
  }

  /// \~japanese
  /// 現在のノードから終点ノードまでの最短経路を計算します．
  ///
  /// \~english
  /// Computes the shortest path from the current node to the destination node.
  void computeShortestPath(
      std::unordered_map<Position, bool> wall_overrides = {}) {
    std::queue<NodeId> q;
    for (auto id : ids_destination) {
      g[id] = 0;
      q.push(id);
    }

    NodeId examined_nodes = 0;
    NodeId max_heap_size = 0;
    while (!q.empty()) {
      auto uid = q.front();
      q.pop();
      examined_nodes++;
      for (auto &[spid, spcost] :
           Base::mg->neighborEdges(uid, wall_overrides)) {
        if (g[spid] > satSum(g[uid], spcost)) {
          g[spid] = satSum(g[uid], spcost);
          q.push(spid);
        }
      }
      if (q.size() > max_heap_size) {
        max_heap_size = NodeId(q.size());
      }
    }
    // computation finished
#ifdef AMAZE_DEBUG
    std::cout << "The number of examined nodes in this round: "
              << examined_nodes << std::endl;
    std::cout << "Maximum size of the open list: " << max_heap_size
              << std::endl;
#endif
    return;
  }

  NodeId currentNodeId() const override { return id_current; }

  NodeId lastNodeId() const override { return id_last; }

  std::set<NodeId> destinationNodeIds() const override {
    return ids_destination;
  }

  SolverState currentSolverState() const override {
    if (ids_destination.find(id_current) != ids_destination.end()) {
      return SolverState::kReached;
    }
    if (g[id_current] == kMaxCost) {
      return SolverState::kFailed;
    }
    return SolverState::kInProgress;
  }

  std::pair<NodeId, Cost> lowestNeighbor(
      NodeId id, std::unordered_map<Position, bool> wall_overrides = {}) const {
    NodeId argmin = id;
    Cost mincost = kMaxCost;
    for (auto &[spid, spcost] : Base::mg->neighborEdges(id, wall_overrides)) {
      if (spcost != kMaxCost && mincost >= g[spid]) {
        mincost = g[spid];
        argmin = spid;
      }
    }
    return {argmin, mincost};
  }

  std::vector<AgentState> reconstructPath(
      NodeId id_from, const std::set<NodeId> &ids_to) const override {
    std::vector<AgentState> path;
    path.push_back(Base::mg->agentStateByNodeId(id_from));
    NodeId id_current_on_path = id_from;
    NodeId id_last_on_path = id_from;
    while (ids_to.find(id_current_on_path) == ids_to.end()) {
      auto [nid, ncost] = lowestNeighbor(id_current_on_path);
      if (ncost == kMaxCost) /* [[unlikely]] */ {
        return std::vector<AgentState>();
      }
      id_last_on_path = id_current_on_path;
      id_current_on_path = nid;
      path.push_back(
          Base::mg->agentStateByEdge(id_last_on_path, id_current_on_path));
    }
    return path;
  }

  void processBeforeReplanning() override { resetCostsAndLists(); }

  void preSense(const std::unordered_set<Position> &sense_positions
                [[maybe_unused]]) override {
    auto ln = lowestNeighbor(id_current);
    id_candidate = ln.first;
  }

  void postSense(const std::unordered_map<Position, bool> &wall_overrides
                 [[maybe_unused]]) override {
    if (currentSolverState() == SolverState::kReached) {
#ifndef AMAZE_NO_STDIO
      std::cout << "Reached the destination" << std::endl;
#endif
      return;
    }
    if (Base::mg->edgeCost(id_current, id_candidate) == kMaxCost) {
      processBeforeReplanning();
      computeShortestPath();
      if (currentSolverState() == SolverState::kFailed) /* [[unlikely]] */ {
#ifndef AMAZE_NO_STDIO
        std::cerr << "No route" << std::endl;
#endif
        return;
      }
    }
    id_last = id_current;
    id_current = lowestNeighbor(id_current).first;
  }

  void resetCostsAndLists() { g.fill(kMaxCost); }

  void reset() override {
    resetCostsAndLists();
    id_current = Base::mg->startNodeId();
    ids_destination = Base::mg->goalNodeIds();
    id_last = id_current;
  }

  void initialize() override {
    reset();
    computeShortestPath();
  }

  void changeDestinations(const std::set<NodeId> &ids) override {
    ids_destination = ids;
    id_last = id_current;
    processBeforeReplanning();
    computeShortestPath();
  }

 protected:
  /// \~japanese 現在のノードのID
  /// \~english Current node ID
  NodeId id_current;
  /// \~japanese 一手前のノードのID
  /// \~english Last node ID
  NodeId id_last;
  /// \~japanese 終点ノードのID
  /// \~english Destination node ID
  std::set<NodeId> ids_destination;
  /// \~japanese つぎに訪れる候補ノードのID
  /// \~english Next candidate node ID
  NodeId id_candidate;

  /// \~japanese 終点から各ノードまでのコスト値
  /// \~english Cost of each node from the destination
  std::array<Cost, NodeCount> g;
};  // namespace solver

}  // namespace solver
}  // namespace amaze
#endif  // AMAZE_SOLVER_BFS_H_
