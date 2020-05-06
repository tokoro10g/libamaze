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

#ifndef AMAZE_COMMON_AGENT_HELPER_H_
#define AMAZE_COMMON_AGENT_HELPER_H_

#include <unordered_set>

#include "amaze/common/common_types.h"
#include "amaze/maze_graph/fourway_graph.h"

namespace amaze {

template <typename TMazeGraph>
class MazeGraphFeatureBase {
 public:
  static std::unordered_set<Position> sensePositions(AgentState as) {
    std::unordered_set<Position> positions;
    if (as.dir == kNorth) {
      positions.insert(as.pos + Difference{0, 2});
      positions.insert(as.pos + Difference{1, 1});
      positions.insert(as.pos + Difference{-1, 1});
    } else if (as.dir == kEast) {
      positions.insert(as.pos + Difference{2, 0});
      positions.insert(as.pos + Difference{1, 1});
      positions.insert(as.pos + Difference{1, -1});
    } else if (as.dir == kSouth) {
      positions.insert(as.pos + Difference{0, -2});
      positions.insert(as.pos + Difference{1, -1});
      positions.insert(as.pos + Difference{-1, -1});
    } else if (as.dir == kWest) {
      positions.insert(as.pos + Difference{-2, 0});
      positions.insert(as.pos + Difference{-1, 1});
      positions.insert(as.pos + Difference{-1, -1});
    } else if (as.dir == kNoDirection) {
      if (as.pos.x % 2 == 0 && as.pos.y % 2 == 1) {
        positions.insert(as.pos + Difference{0, 2});
        positions.insert(as.pos + Difference{1, 1});
        positions.insert(as.pos + Difference{-1, 1});
        positions.insert(as.pos + Difference{0, -2});
        positions.insert(as.pos + Difference{1, -1});
        positions.insert(as.pos + Difference{-1, -1});
      } else if (as.pos.x % 2 == 1 && as.pos.y % 2 == 0) {
        positions.insert(as.pos + Difference{2, 0});
        positions.insert(as.pos + Difference{1, 1});
        positions.insert(as.pos + Difference{1, -1});
        positions.insert(as.pos + Difference{-2, 0});
        positions.insert(as.pos + Difference{-1, 1});
        positions.insert(as.pos + Difference{-1, -1});
      }
    } else /* [[unlikely]] */ {
      return std::unordered_set<Position>();
    }
    return positions;
  }
  static std::unordered_map<Position, bool> sense(
      Maze<TMazeGraph::kMaxWidth> &virtual_maze,
      const Maze<TMazeGraph::kMaxWidth> &reference_maze,
      const std::unordered_set<Position> &sense_positions) {
    std::unordered_map<Position, bool> wall_overrides;
    for (auto p : sense_positions) {
      if (p.x > 2 * TMazeGraph::kMaxWidth || p.y > 2 * TMazeGraph::kMaxWidth) {
        continue;
      }
      // TODO(tokoro10g): consider cases where the wall information acquired
      // previously was wrong
      if (virtual_maze.isCheckedWall(p)) {
        continue;
      }
      virtual_maze.setCheckedWall(p, true);
      if (reference_maze.isSetWall(p)) {
        virtual_maze.setWall(p, true);
        wall_overrides.insert({p, true});
      } else {
        virtual_maze.setWall(p, false);
      }
    }
    return wall_overrides;
  }

 private:
  MazeGraphFeatureBase();
};

template <typename TMazeGraph>
class MazeGraphFeature : public MazeGraphFeatureBase<TMazeGraph> {
 private:
  MazeGraphFeature();
};

template <bool kExplore, typename TCost, typename TNodeId, uint8_t W>
class MazeGraphFeature<maze_graph::FourWayGraph<kExplore, TCost, TNodeId, W>>
    : public MazeGraphFeatureBase<
          maze_graph::FourWayGraph<kExplore, TCost, TNodeId, W>> {
 public:
  static std::unordered_set<Position> sensePositions(AgentState as) {
    std::unordered_set<Position> positions;
    if (as.dir == kNorth) {
      positions.insert(as.pos + Difference{0, 1});
      positions.insert(as.pos + Difference{1, 0});
      positions.insert(as.pos + Difference{-1, 0});
    } else if (as.dir == kEast) {
      positions.insert(as.pos + Difference{1, 0});
      positions.insert(as.pos + Difference{0, 1});
      positions.insert(as.pos + Difference{0, -1});
    } else if (as.dir == kSouth) {
      positions.insert(as.pos + Difference{0, -1});
      positions.insert(as.pos + Difference{1, 0});
      positions.insert(as.pos + Difference{-1, 0});
    } else if (as.dir == kWest) {
      positions.insert(as.pos + Difference{-1, 0});
      positions.insert(as.pos + Difference{0, 1});
      positions.insert(as.pos + Difference{0, -1});
    } else if (as.dir == kNoDirection) {
      positions.insert(as.pos + Difference{1, 0});
      positions.insert(as.pos + Difference{-1, 0});
      positions.insert(as.pos + Difference{0, 1});
      positions.insert(as.pos + Difference{0, -1});
    } else /* [[unlikely]] */ {
      return std::unordered_set<Position>();
    }
    return positions;
  }
};

template <typename TSolver>
class SolverFeatureBase {
 private:
  SolverFeatureBase();
};

template <typename TSolver>
class SolverFeature : public SolverFeatureBase<TSolver> {
 private:
  SolverFeature();
};

template <typename TMazeGraph, typename TSolver>
class AgentHelper : public MazeGraphFeature<TMazeGraph>,
                    public SolverFeature<TSolver> {
 public:
  using MazeGraph = TMazeGraph;
  using Solver = TSolver;

  static std::unordered_set<Position> currentSensePositions(
      const TSolver &solver) {
    return MazeGraphFeature<TMazeGraph>::sensePositions(
        solver.currentAgentState());
  }

 private:
  AgentHelper() {}
};

}  // namespace amaze

#endif  // AMAZE_COMMON_AGENT_HELPER_H_
