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

//===-- samples/sixway_dynamic.cc - Dynamic Search Example ----------------===//
///
/// \file
/// This file is an example code that uses libamaze to simulate the exploration
/// and trial runs of a micromouse robot.
///
//===----------------------------------------------------------------------===//

#include <cstdint>
#include <iostream>
#include <string>

#include "amaze/common/agent_helper.h"
#include "amaze/common/common_types.h"
#include "amaze/common/maze_utils.h"
#include "amaze/maze_graph/sixway_graph.h"
#include "amaze/maze_graph/sixway_turn_cost_graph.h"
#include "amaze/solver/astar.h"
#include "amaze/solver/bfs.h"
#include "amaze/solver/dstarlite.h"
#include "amaze/solver/look_ahead.h"
#include "samples_utils.h"

void showUsage(std::string name) {
  std::cout << "usage:" << std::endl;
  std::cout << name << " filename" << std::endl;
}

int main(int argc, char* argv[]) {
  using namespace amaze;
  using namespace amaze::maze_graph;
  using namespace amaze::solver;
  using namespace amaze::utils;

  if (argc < 2) {
    showUsage(argv[0]);
    return 0;
  }
  std::string maze_filename = argv[1];

  constexpr uint8_t max_maze_width = 32;
  Maze<max_maze_width> reference_maze;
  Maze<max_maze_width> virtual_maze;
  loadMazeFromFile(reference_maze, argv[1]);

  printMaze(reference_maze);

  using MazeGraph1 = SixWayGraph<true, uint16_t, uint16_t, max_maze_width>;
  using MazeGraph2 = FourWayGraph<true, uint16_t, uint16_t, max_maze_width>;

  using SL = SolverList<DStarLite, AStar, BFS>;
  using ML = MazeGraphList<MazeGraph1, MazeGraph2>;
  BenchFactory<SL, ML>::run(reference_maze);

  return 0;
}
