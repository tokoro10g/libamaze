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

#ifndef INCLUDE_AMAZE_COMMON_AGENT_HELPER_H_
#define INCLUDE_AMAZE_COMMON_AGENT_HELPER_H_

#include <vector>

#include "amaze/common/common_types.h"
#include "amaze/maze_graph/fourway_graph.h"

namespace amaze {

template <typename TMazeGraph>
class MazeGraphFeatureBase {
 public:
  static std::vector<Position> sensePositions(AgentState as) {
    std::vector<Position> positions;
    if (as.dir == kNorth) {
      positions.push_back(as.pos + Difference{0, 2});
      positions.push_back(as.pos + Difference{1, 1});
      positions.push_back(as.pos + Difference{-1, 1});
    } else if (as.dir == kEast) {
      positions.push_back(as.pos + Difference{2, 0});
      positions.push_back(as.pos + Difference{1, 1});
      positions.push_back(as.pos + Difference{1, -1});
    } else if (as.dir == kSouth) {
      positions.push_back(as.pos + Difference{0, -2});
      positions.push_back(as.pos + Difference{1, -1});
      positions.push_back(as.pos + Difference{-1, -1});
    } else if (as.dir == kWest) {
      positions.push_back(as.pos + Difference{-2, 0});
      positions.push_back(as.pos + Difference{-1, 1});
      positions.push_back(as.pos + Difference{-1, -1});
    } else if (as.dir == kNoDirection) {
      if (as.pos.x % 2 == 0 && as.pos.y % 2 == 1) {
        positions.push_back(as.pos + Difference{0, 2});
        positions.push_back(as.pos + Difference{1, 1});
        positions.push_back(as.pos + Difference{-1, 1});
        positions.push_back(as.pos + Difference{0, -2});
        positions.push_back(as.pos + Difference{1, -1});
        positions.push_back(as.pos + Difference{-1, -1});
      } else if (as.pos.x % 2 == 1 && as.pos.y % 2 == 0) {
        positions.push_back(as.pos + Difference{2, 0});
        positions.push_back(as.pos + Difference{1, 1});
        positions.push_back(as.pos + Difference{1, -1});
        positions.push_back(as.pos + Difference{-2, 0});
        positions.push_back(as.pos + Difference{-1, 1});
        positions.push_back(as.pos + Difference{-1, -1});
      }
    } else /* [[unlikely]] */ {
      return std::vector<Position>();
    }
    return positions;
  }
  static std::vector<Position> sense(
      Maze<TMazeGraph::kWidth> &virtual_maze,
      const Maze<TMazeGraph::kWidth> &reference_maze,
      const std::vector<Position> &sense_positions) {
    std::vector<Position> changed_positions;
    for (auto p : sense_positions) {
      if (p.x > 2 * TMazeGraph::kWidth || p.y > 2 * TMazeGraph::kWidth) {
        continue;
      }
      if (virtual_maze.isCheckedWall(p)) {
        continue;
      }
      virtual_maze.setCheckedWall(p, true);
      if (reference_maze.isSetWall(p)) {
        virtual_maze.setWall(p, true);
        changed_positions.push_back(p);
      } else {
        virtual_maze.setWall(p, false);
      }
    }
    return changed_positions;
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
  static std::vector<Position> sensePositions(AgentState as) {
    std::vector<Position> positions;
    if (as.dir == kNorth) {
      positions.push_back(as.pos + Difference{0, 1});
      positions.push_back(as.pos + Difference{1, 0});
      positions.push_back(as.pos + Difference{-1, 0});
    } else if (as.dir == kEast) {
      positions.push_back(as.pos + Difference{1, 0});
      positions.push_back(as.pos + Difference{0, 1});
      positions.push_back(as.pos + Difference{0, -1});
    } else if (as.dir == kSouth) {
      positions.push_back(as.pos + Difference{0, -1});
      positions.push_back(as.pos + Difference{1, 0});
      positions.push_back(as.pos + Difference{-1, 0});
    } else if (as.dir == kWest) {
      positions.push_back(as.pos + Difference{-1, 0});
      positions.push_back(as.pos + Difference{0, 1});
      positions.push_back(as.pos + Difference{0, -1});
    } else if (as.dir == kNoDirection) {
      positions.push_back(as.pos + Difference{1, 0});
      positions.push_back(as.pos + Difference{-1, 0});
      positions.push_back(as.pos + Difference{0, 1});
      positions.push_back(as.pos + Difference{0, -1});
    } else /* [[unlikely]] */ {
      return std::vector<Position>();
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

  static std::vector<Position> currentSensePositions(const TSolver &solver) {
    return MazeGraphFeature<TMazeGraph>::sensePositions(
        solver.currentAgentState());
  }

 private:
  AgentHelper() {}
};

}  // namespace amaze

#endif  // INCLUDE_AMAZE_COMMON_AGENT_HELPER_H_
