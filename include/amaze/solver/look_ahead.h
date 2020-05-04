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

#ifndef INCLUDE_AMAZE_SOLVER_LOOK_AHEAD_H_
#define INCLUDE_AMAZE_SOLVER_LOOK_AHEAD_H_

#include <array>
#include <iostream>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

#include "amaze/maze_graph/sixway_turn_cost_graph.h"
#include "amaze/solver/solver_base.h"

namespace amaze {
namespace solver {

/// \~japanese
/// Look aheadソルバ
///
/// \~english
/// Look-ahead solver
template <typename TSolver>
class LookAhead : public TSolver {
  static_assert(std::is_base_of_v<AllAtOnceSolverBase, TSolver>);

 public:
  using Cost = typename TSolver::Cost;
  using NodeId = typename TSolver::NodeId;
  using TSolver::kMaxWidth;
  using TSolver::kNodeCount;

  using MazeGraph =
      maze_graph::MazeGraphBase<Cost, NodeId, kMaxWidth, kNodeCount>;

  using TSolver::computeShortestPath;
  using TSolver::currentSolverState;
  using TSolver::lowestNeighbor;
  using TSolver::processBeforeReplanning;

  using TSolver::id_current;
  using TSolver::id_last;
  using TSolver::ids_destination;
  using TSolver::kMaxCost;
  using TSolver::mg;

  explicit LookAhead(const MazeGraph *mg)
      : TSolver(mg), look_ahead_candidates() {}

  void preSense(const std::unordered_set<Position> &sense_positions
                [[maybe_unused]]) final {
    look_ahead_candidates.fill(mg->nodeIdByAgentState(kInvalidAgentState));

    auto [id_neighbor, cost_neighbor] = lowestNeighbor(id_current);
#if 1
    std::cout << "[0]\tcandidate: " << id_neighbor << " *" << cost_neighbor
              << std::endl;
#endif
    look_ahead_candidates.at(0) = id_neighbor;

    int count_neighbors = static_cast<int>(mg->neighbors(id_current).size());
    int count_candidates = std::min(count_neighbors, 4);
    std::unordered_map<Position, bool> wall_overrides;
    for (int i = 1; i < count_candidates; i++) {
      Position pos_wall =
          mg->wallPositionOnEdge(id_current, look_ahead_candidates.at(i - 1));
      if (mg->maze.isSetWall(pos_wall)) {
        break;
      }
      wall_overrides.insert({pos_wall, true});

      processBeforeReplanning(wall_overrides, pos_wall);
      computeShortestPath(wall_overrides);

      auto [id_neighbor, cost_neighbor] =
          lowestNeighbor(id_current, wall_overrides);
      if (currentSolverState() == SolverState::kFailed) {
        // wall_overridesが実際に起こるとゴールに到達できなくなる
        break;
      } else {
        look_ahead_candidates.at(i) = id_neighbor;
      }
#if 1
      std::cout << "[" << i << "] wall position: " << pos_wall << std::endl;
      std::cout << "\tcandidate: " << look_ahead_candidates.at(i) << " *"
                << cost_neighbor << std::endl;
#endif
    }
    for (auto &[pos, wall] : wall_overrides) {
      wall = !wall;
    }
  }

  void postSense(const std::unordered_map<Position, bool> &wall_overrides
                 [[maybe_unused]]) final {
    if (currentSolverState() == SolverState::kReached) {
#if 1
      std::cout << "Reached the destination" << std::endl;
#endif
      return;
    }
    for (auto id_candidate : look_ahead_candidates) {
      std::cout << "candidate " << id_candidate << std::endl;
      if (mg->edgeCost(id_current, id_candidate) != kMaxCost) {
        id_last = id_current;
        id_current = id_candidate;
#if 1
        std::cout << "[v]\tmove to " << id_current << std::endl;
        std::cout << "\tSolverState: " << static_cast<int>(currentSolverState())
                  << std::endl;
#endif
        return;
      }
    }
#if 1
    std::cout << "No route" << std::endl;
#endif
    // all paths are blocked, which is not supposed to happen
    __builtin_unreachable();
  }

 private:
  std::array<NodeId, 4> look_ahead_candidates;
};

}  // namespace solver
}  // namespace amaze

#endif  // INCLUDE_AMAZE_SOLVER_LOOK_AHEAD_H_
