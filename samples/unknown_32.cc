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

//===-- samples/unknown_32.cc - Possible Worst Case Example ---------------===//
///
/// \file
/// This file is an example code that uses libamaze to compute the shortest
/// path of a maze.
///
//===----------------------------------------------------------------------===//

#include <chrono>  // NOLINT(build/c++11)
#include <iostream>
#include <vector>

#include "amaze/common/common_types.h"
#include "amaze/common/maze_utils.h"
#include "amaze/maze_graph/fourway_graph.h"
#include "amaze/maze_graph/sixway_graph.h"
#include "amaze/maze_graph/sixway_turn_cost_graph.h"
#include "amaze/solver/astar.h"
#include "amaze/solver/dstarlite.h"

int main() {
  using namespace std::chrono;  // NOLINT(build/namespaces)

  constexpr uint8_t max_maze_width = 32;
  constexpr int repeat_count = 100;
  amaze::Maze<max_maze_width> maze;

  amaze::utils::loadEmptyMaze(maze);
  maze.goals.push_back({61, 62});
  maze.goals.push_back({62, 61});
  maze.goals.push_back({62, 62});
  maze.setWall({1, 0}, true);
  maze.setCheckedWall({0, 1}, true);
  maze.setCheckedWall({1, 0}, true);
  amaze::utils::printMaze(maze);
  std::cout << std::endl;

  // 迷路グラフを定義し，スタートとゴールの状態を表示します．
  //
  // Defines a maze graph and display agent states of the start and goals.
  //
  amaze::maze_graph::FourWayGraph mg_four(maze);
  amaze::maze_graph::SixWayGraph mg_six(maze);
  amaze::maze_graph::SixWayTurnCostGraph mg_six_cost(maze);

  // ソルバを初期化して最短経路を導出します．
  //
  // Initializes the solver and calculate the shortest path.
  //
  auto solver_four = amaze::solver::DStarLite(&mg_four);
  auto solver_six = amaze::solver::DStarLite(&mg_six);
  auto solver_six_cost = amaze::solver::DStarLite(&mg_six_cost);
  auto solver_four_astar = amaze::solver::AStar(&mg_four);
  auto solver_six_astar = amaze::solver::AStar(&mg_six);
  auto solver_six_cost_astar = amaze::solver::AStar(&mg_six_cost);

  high_resolution_clock::time_point tstart, tend;

#if 1
  std::cout << "FourWayGraph: " << std::endl;
  tstart = high_resolution_clock::now();
  for (int k = 0; k < repeat_count; k++) {
    solver_four.initialize();
  }
  tend = high_resolution_clock::now();
  std::cout << "[D* Lite] Average time: "
            << static_cast<double>(
                   duration_cast<microseconds>(tend - tstart).count()) /
                   1000.0 / repeat_count
            << " ms" << std::endl;
  tstart = high_resolution_clock::now();
  for (int k = 0; k < repeat_count; k++) {
    solver_four_astar.initialize();
  }
  tend = high_resolution_clock::now();
  std::cout << "[A*] Average time: "
            << static_cast<double>(
                   duration_cast<microseconds>(tend - tstart).count()) /
                   1000.0 / repeat_count
            << " ms" << std::endl;
#endif

#if 1
  std::cout << "SixWayGraph: " << std::endl;
  tstart = high_resolution_clock::now();
  for (int k = 0; k < repeat_count; k++) {
    solver_six.initialize();
  }
  tend = high_resolution_clock::now();
  std::cout << "[D* Lite] Average time: "
            << static_cast<double>(
                   duration_cast<microseconds>(tend - tstart).count()) /
                   1000.0 / repeat_count
            << " ms" << std::endl;
  tstart = high_resolution_clock::now();
  for (int k = 0; k < repeat_count; k++) {
    solver_six_astar.initialize();
  }
  tend = high_resolution_clock::now();
  std::cout << "[A*] Average time: "
            << static_cast<double>(
                   duration_cast<microseconds>(tend - tstart).count()) /
                   1000.0 / repeat_count
            << " ms" << std::endl;
#endif

#if 1
  std::cout << "SixWayTurnCostGraph: " << std::endl;
  tstart = high_resolution_clock::now();
  for (int k = 0; k < repeat_count; k++) {
    solver_six_cost.initialize();
  }
  tend = high_resolution_clock::now();
  std::cout << "[D* Lite] Average time: "
            << static_cast<double>(
                   duration_cast<microseconds>(tend - tstart).count()) /
                   1000.0 / repeat_count
            << " ms" << std::endl;
  tstart = high_resolution_clock::now();
  for (int k = 0; k < repeat_count; k++) {
    solver_six_cost_astar.initialize();
  }
  tend = high_resolution_clock::now();
  std::cout << "[A*] Average time: "
            << static_cast<double>(
                   duration_cast<microseconds>(tend - tstart).count()) /
                   1000.0 / repeat_count
            << " ms" << std::endl;
#endif

  return 0;
}
