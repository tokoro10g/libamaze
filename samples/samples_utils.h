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

#ifndef AMAZE_SAMPLES_SAMPLES_UTILS_H_
#define AMAZE_SAMPLES_SAMPLES_UTILS_H_

#include <cxxabi.h>

#include <chrono>  // NOLINT(build/c++11)
#include <iostream>

#include "amaze/common/agent_helper.h"
#include "amaze/common/common_types.h"

template <typename T>
std::string getTypeName(const T& t, const bool is_ignore_template = false) {
  std::size_t length = 0;
  int status = 0;
  char* const type_name =
      abi::__cxa_demangle(typeid(t).name(), nullptr, &length, &status);
  if (is_ignore_template) {
    auto p = std::find(type_name, type_name + length, '<');
    if (p != type_name + length) {
      *p = 0;  // terminate string
    }
  }
  return std::string(type_name);
}

class SolverBenchBase {
 public:
  SolverBenchBase(){};
  virtual ~SolverBenchBase() = default;
  virtual void run() = 0;
};

template <typename TMazeGraph, class TSolver>
class SolverBench : public SolverBenchBase {
 public:
  SolverBench(const amaze::Maze<TMazeGraph::kMaxWidth>& reference_maze)
      : reference_maze(reference_maze),
        virtual_maze(),
        mg(virtual_maze),
        solver(&mg),
        t_start(),
        t_end() {
    amaze::utils::loadEmptyMaze(virtual_maze);
    virtual_maze.goals = reference_maze.goals;
    virtual_maze.setWall({1, 0}, true);
    virtual_maze.setCheckedWall({0, 1}, true);
    virtual_maze.setCheckedWall({1, 0}, true);
  }

  void run() {
    using AH = amaze::AgentHelper<decltype(mg), decltype(solver)>;

    t_start = std::chrono::high_resolution_clock::now();
    {
      solver.initialize();
      while (solver.currentSolverState() ==
             amaze::solver::SolverState::kInProgress) {
        const auto sense_positions = AH::currentSensePositions(solver);
        solver.preSense(sense_positions);
        const auto wall_overrides =
            AH::sense(virtual_maze, reference_maze, sense_positions);
        solver.postSense(wall_overrides);
      }
    }
    t_end = std::chrono::high_resolution_clock::now();

    const std::string mg_type_name = getTypeName(mg, true);
    const std::string solver_type_name = getTypeName(solver, true);
    std::cout << "[ " << mg_type_name << ", " << solver_type_name << " ]"
              << std::endl;
    std::cout << "Time: "
              << static_cast<double>(
                     std::chrono::duration_cast<std::chrono::microseconds>(
                         t_end - t_start)
                         .count()) /
                     1000.0
              << " ms" << std::endl;
  }

 private:
  const amaze::Maze<TMazeGraph::kMaxWidth> reference_maze;
  amaze::Maze<TMazeGraph::kMaxWidth> virtual_maze;
  TMazeGraph mg;
  TSolver solver;
  std::chrono::high_resolution_clock::time_point t_start;
  std::chrono::high_resolution_clock::time_point t_end;
};

struct None {};

template <template <typename TCost, typename TNodeId, uint8_t, TNodeId>
          typename Tmpl>
struct SolverSel {
  template <typename TCost, typename TNodeId, uint8_t kMaxWidth,
            TNodeId kNodeCount>
  struct Bind {
    typedef Tmpl<TCost, TNodeId, kMaxWidth, kNodeCount> type;
  };
};

template <template <typename TCost, typename TNodeId, uint8_t, TNodeId>
          typename Head_,
          template <typename TCost, typename TNodeId, uint8_t, TNodeId>
          typename... Tail_>
struct SolverList {
  template <typename TMazeGraph>
  using Head = Head_<typename TMazeGraph::Cost, typename TMazeGraph::NodeId,
                     TMazeGraph::kMaxWidth, TMazeGraph::kNodeCount>;
  using Tail = SolverList<Tail_...>;
};

template <template <typename TCost, typename TNodeId, uint8_t, TNodeId>
          typename Head_>
struct SolverList<Head_> {
  template <typename TMazeGraph>
  using Head = Head_<typename TMazeGraph::Cost, typename TMazeGraph::NodeId,
                     TMazeGraph::kMaxWidth, TMazeGraph::kNodeCount>;
  using Tail = None;
};

template <typename Head_, typename... Tail_>
struct MazeGraphList {
  using Head = Head_;
  using Tail = MazeGraphList<Tail_...>;
};

template <typename Head_>
struct MazeGraphList<Head_> {
  using Head = Head_;
  using Tail = None;
};

template <typename SolverList_, typename MazeGraphList_, bool IsTop = true>
struct BenchFactory {
  template <uint8_t W>
  static constexpr void run(const amaze::Maze<W>& maze) {
    /* clang-format off */
    SolverBench<
        typename MazeGraphList_::Head,
        typename SolverList_::template Head<typename MazeGraphList_::Head>
      >(maze).run();
    /* clang-format on */
    BenchFactory<typename SolverList_::Tail, MazeGraphList_, false>::run(maze);
    if (IsTop) {
      BenchFactory<SolverList_, typename MazeGraphList_::Tail, true>::run(maze);
    }
  }
};

template <typename MazeGraphList_, bool IsTop>
struct BenchFactory<None, MazeGraphList_, IsTop> {
  template <uint8_t W>
  static constexpr void run(const amaze::Maze<W>& maze [[maybe_unused]]) {}
};

template <typename SolverList_, bool IsTop>
struct BenchFactory<SolverList_, None, IsTop> {
  template <uint8_t W>
  static constexpr void run(const amaze::Maze<W>& maze [[maybe_unused]]) {}
};

#endif  // AMAZE_SAMPLES_SAMPLES_UTILS_H_
