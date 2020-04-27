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

#include <chrono>  // NOLINT(build/c++11)
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

#define ENABLE_TURN_COST 0

void showUsage(std::string name) {
  std::cout << "usage:" << std::endl;
  std::cout << name << " filename" << std::endl;
}

int main(int argc, char *argv[]) {
  using namespace std::chrono;  // NOLINT(build/namespaces)

  if (argc < 2) {
    showUsage(argv[0]);
    return 0;
  }
  std::string maze_filename = argv[1];

  /*
   * 初期化 Initialization
   */

  // Mazeクラスのテンプレートパラメータには，迷路の幅・高さの最大値を指定します．
  // このサンプルでは，迷路データを仮想・参照の2つ用意します．
  // 仮想迷路(virtual_maze)は，ロボットがもつ部分的な迷路情報を表すデータで，
  // 探索をしながら参照迷路(reference_maze)のデータを参照して更新していきます．
  // 仮想迷路は空の迷路で初期化し，参照迷路にはコマンドライン引数で指定したファイルからデータを読み込みます．
  //
  // Specify the maximum width/height of the maze in the template parameter of
  // Maze class. In this example, we prepare two kinds of maze data; virtual and
  // reference mazes. The virtual maze (virtual_maze) represents the partial
  // maze information the robot owns, and the data is updated by referring to
  // the reference maze (reference_maze) during exploration. The virtual maze is
  // initialized with empty data, and we load reference maze data from the file
  // passed by the command-line argument.
  //
  constexpr uint8_t max_maze_width = 32;
  amaze::Maze<max_maze_width> reference_maze;
  amaze::Maze<max_maze_width> virtual_maze;
  amaze::utils::loadMazeFromFile(reference_maze, argv[1]);
  amaze::utils::loadEmptyMaze(virtual_maze);

  // 仮想迷路のゴールを参照迷路のゴールと同一に設定します．
  //
  // Sets the goals of the virtal maze to those of the reference maze.
  //
  auto goals = reference_maze.goals;
  virtual_maze.goals = goals;
  // スタート区画の前壁(0, 1)と右壁(1, 0)はスタート時に有無が確定しており，
  // 右壁があります．
  //
  // The front (0, 1) and the right (1, 0) of the start cell are checked before
  // exploration, and there is a wall on the right.
  //
  virtual_maze.setWall({1, 0}, true);
  virtual_maze.setCheckedWall({0, 1}, true);
  virtual_maze.setCheckedWall({1, 0}, true);

  // 仮想迷路をグラフで表現します．
  // SixWayGraphクラス(壁をノードとした6方向に連結するグラフ)を使用します．
  //
  // Defines a graph representation of the virtual maze.
  // This example uses SixWayGraph class that represents a graph with
  // nodes on the wall positions.
  //
#if ENABLE_TURN_COST
  amaze::maze_graph::SixWayTurnCostGraph mg(virtual_maze);
#else
  amaze::maze_graph::SixWayGraph mg(virtual_maze);
#endif
  // グラフのポインタをD* Liteソルバに渡して，ソルバを初期化します．
  //
  // Passes the pointer of the graph to a D* Lite solver and initializes the
  // solver.
  //
  auto solver_DSL = amaze::solver::DStarLite(&mg);
  auto solver_AS = amaze::solver::AStar(&mg);
  auto solver_BFS = amaze::solver::BFS(&mg);
  high_resolution_clock::time_point tstart, tend;

  tstart = high_resolution_clock::now();
  solver_DSL.initialize();
  using AH = amaze::AgentHelper<decltype(mg), decltype(solver_DSL)>;

  // 参照迷路の形状とスタート・ゴールの情報を表示します．
  //
  // Prints the reference maze, and show information of the start and goals.
  //
  amaze::utils::printMaze(reference_maze);
  auto goal_ids = mg.goalNodeIds();

  /*
   * 行きの探索 Exploration to the goal
   */

  // ゴールノードのうちどれか1つにたどり着くまで探索を行います．
  //
  // Repeats until the solver arrives one of the goals.
  //
  while (solver_DSL.currentSolverState() ==
         amaze::solver::SolverState::kInProgress) {
    // 現在のソルバの状態から，つぎにセンシングする壁の位置のリストを取得します．
    //
    // Obtains a list of wall positions to sense before the next step, according
    // to the current solver state.
    //
    auto sense_positions = AH::currentSensePositions(solver_DSL);
    // センシング前のソルバの処理を行います．
    // 通常のD* Liteソルバでは何もしません．
    //
    // Performs a solver action before sensing. A normal D* Lite solver does not
    // do anything.
    //
    solver_DSL.preSense(sense_positions);
    // 壁のセンシングを模擬した処理を行います．
    // AgentHelperを使用して参照迷路から壁の有無を取得し，仮想迷路に反映します．
    //
    // Simulates the wall sensing.
    // AgentHelper::sense updates the wall information in the virtual maze based
    // on that acquired from the reference maze.
    //
    auto wall_overrides =
        AH::sense(virtual_maze, reference_maze, sense_positions);
    // センシング後のソルバの処理を行います．変更された壁の座標を渡して，つぎに取るべき行動を決定します．
    // ソルバの状態はこの処理で更新されます．
    //
    // Performs a solver action after sensing. This process determines the next
    // action according to the positions of changed walls. The solver state is
    // updated during this process.
    //
    solver_DSL.postSense(wall_overrides);
    // ここでsolver.currentAgentState()を取得すると，つぎの状態が取得できます．
    //
    // The return value of solver.currentAgentState() here corresponds to the
    // next agent state.
    //
  }
  tend = high_resolution_clock::now();
  std::cout << "[D* Lite] Time: "
            << static_cast<double>(
                   duration_cast<microseconds>(tend - tstart).count()) /
                   1000.0
            << " ms" << std::endl;

  /*
   * A*
   */
  amaze::utils::loadEmptyMaze(virtual_maze);
  virtual_maze.goals = goals;
  virtual_maze.setWall({1, 0}, true);
  virtual_maze.setCheckedWall({0, 1}, true);
  virtual_maze.setCheckedWall({1, 0}, true);

  tstart = high_resolution_clock::now();
  solver_AS.initialize();
  using AH2 = amaze::AgentHelper<decltype(mg), decltype(solver_AS)>;
  while (solver_AS.currentSolverState() ==
         amaze::solver::SolverState::kInProgress) {
    auto sense_positions = AH2::currentSensePositions(solver_AS);
    solver_AS.preSense(sense_positions);
    auto wall_overrides =
        AH2::sense(virtual_maze, reference_maze, sense_positions);
    solver_AS.postSense(wall_overrides);
  }
  tend = high_resolution_clock::now();
  std::cout << "[A*] Time: "
            << static_cast<double>(
                   duration_cast<microseconds>(tend - tstart).count()) /
                   1000.0
            << " ms" << std::endl;

  /*
   * BFS
   */
  amaze::utils::loadEmptyMaze(virtual_maze);
  virtual_maze.goals = goals;
  virtual_maze.setWall({1, 0}, true);
  virtual_maze.setCheckedWall({0, 1}, true);
  virtual_maze.setCheckedWall({1, 0}, true);

  tstart = high_resolution_clock::now();
  solver_BFS.initialize();
  using AH3 = amaze::AgentHelper<decltype(mg), decltype(solver_BFS)>;
  while (solver_BFS.currentSolverState() ==
         amaze::solver::SolverState::kInProgress) {
    auto sense_positions = AH3::currentSensePositions(solver_BFS);
    solver_BFS.preSense(sense_positions);
    auto wall_overrides =
        AH3::sense(virtual_maze, reference_maze, sense_positions);
    solver_BFS.postSense(wall_overrides);
  }
  tend = high_resolution_clock::now();
  std::cout << "[BFS] Time: "
            << static_cast<double>(
                   duration_cast<microseconds>(tend - tstart).count()) /
                   1000.0
            << " ms" << std::endl;

  return 0;
}
