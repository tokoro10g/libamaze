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

//===-- samples/sixway_static.cc - Static Search Example ------------------===//
///
/// \file
/// This file is an example code that uses libamaze to compute the shortest
/// path of a maze.
///
//===----------------------------------------------------------------------===//

#include <iostream>
#include <vector>

#include "amaze/amaze.h"

#include "amaze/common/agent_helper.h"
#include "amaze/common/common_types.h"
#include "amaze/common/common_utils.h"
#include "amaze/common/maze_utils.h"
#include "amaze/maze_graph/sixway_graph.h"
#include "amaze/solver/astar.h"
#include "amaze/solver/bfs.h"
#include "amaze/solver/dstarlite.h"

#define ENABLE_TURN_COST 0

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cout << "usage:" << std::endl;
    std::cout << argv[0] << " filename" << std::endl;
    return -1;
  }

  /*
   * 初期化 Initialization
   */

  // Mazeクラスのテンプレートパラメータには，迷路の幅・高さの最大値を指定します．
  //
  // Specify the maximum width/height of the maze in the template parameter of
  // Maze class.
  //
  constexpr uint8_t max_maze_width = 32;
  amaze::Maze<max_maze_width> maze;
  amaze::utils::loadMazeFromFile(maze, argv[1]);

  // 仮想迷路をグラフで表現します．SixWayGraphクラス(壁をノードとした6方向に連結するグラフ)を使用します．
  //
  // Defines a graph representation of the virtual maze.
  // This example uses SixWayGraph class that represents a graph with
  // nodes on the wall positions.
  //
#if ENABLE_TURN_COST
  amaze::maze_graph::SixWayTurnCostGraph mg(maze);
#else
  amaze::maze_graph::SixWayGraph mg(maze);
#endif
  // 参照迷路の形状とスタート・ゴールの情報を表示します．
  //
  // Prints the reference maze, and show information of the start and goals.
  //
  amaze::utils::printMaze(maze);
  auto id_start = mg.startNodeId();
  auto goal_ids = mg.goalNodeIds();
  std::cout << "The start is " << mg.agentStateByNodeId(id_start) << std::endl;
  std::cout << "The goals are ";
  for (auto id_goal : goal_ids) {
    std::cout << mg.agentStateByNodeId(id_goal) << ", ";
  }
  std::cout << std::endl;

  /*
   * 最短経路の導出 Calculation of the Shortest Path
   */

  // ソルバを初期化して最短経路を導出，表示します．
  //
  // Initializes solver and displays the calculated shortest path.
  //
  auto solver = amaze::solver::DStarLite(&mg);
  solver.initialize();

  std::cout << "Found path: " << std::endl;
  std::vector<amaze::AgentState> path =
      solver.reconstructPath(id_start, goal_ids);
  for (auto as : path) {
    std::cout << as << std::endl;
  }

  return 0;
}
