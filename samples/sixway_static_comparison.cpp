//===-- samples/sixway_static_comparison.cpp - Static Search Benchmark ----===//
//
// Part of libamaze, under the MIT License.
// See https://opensource.org/licenses/MIT for license information.
// SPDX-License-Identifier: MIT
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file is an example code which uses libamaze to compute the shortest
/// path of a maze.
///
//===----------------------------------------------------------------------===//

#include "astar.h"
#include "bfs.h"
#include "dstarlite.h"
#include "maze.h"
#include "mazeutility.h"
#include "sixwaywallnodegraph.h"
#include "sixwaywallnodeturncostgraph.h"
#include <chrono>
#include <iostream>
#include <vector>

using namespace Amaze;
using namespace std::chrono;

#define ENABLE_TURN_COST 0

int main(int argc, char* argv[])
{
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
    // Specify the maximum width/height of the maze in the template parameter of Maze class.
    //
    constexpr uint8_t max_maze_width = 32;
    Maze<max_maze_width> maze;
    Utility::loadMazeFromFile(maze, argv[1]);

    // 仮想迷路をグラフで表現します．SixWayWallNodeGraphクラス(壁をノードとした6方向に連結するグラフ)を使用します．
    //
    // Defines a graph representation of the virtual maze.
    // This example uses FourWayStepMapGraph class which represents a graph with nodes on the wall positions.
    //
#if ENABLE_TURN_COST
    SixWayWallNodeTurnCostGraph mg(maze);
#else
    SixWayWallNodeGraph mg(maze);
#endif
    // 参照迷路の形状とスタート・ゴールの情報を表示します．
    //
    // Prints the reference maze, and show information of the start and goals.
    //
    Utility::printMaze(maze);
    auto goal_ids = mg.goalNodeIds();

    /*
     * 最短経路の導出 Calculation of the Shortest Path
     */

    // ソルバを初期化して最短経路を導出，表示します．
    //
    // Initializes solver and displays the calculated shortest path.
    //
    auto solver_DSL = DStarLite(&mg);
    auto solver_AS = AStar(&mg);
    auto solver_BFS = BFS(&mg);
    high_resolution_clock::time_point tstart, tend;

    tstart = high_resolution_clock::now();
    for (int k = 0; k < 100; k++) {
        solver_DSL.initialize();
    }
    tend = high_resolution_clock::now();
    std::cout << "[D* Lite] Average time: " << (double)duration_cast<microseconds>(tend - tstart).count() / 1000.0 / 100.0 << " ms" << std::endl;

    tstart = high_resolution_clock::now();
    for (int k = 0; k < 100; k++) {
        solver_AS.initialize();
    }
    tend = high_resolution_clock::now();
    std::cout << "[A*] Average time: " << (double)duration_cast<microseconds>(tend - tstart).count() / 1000.0 / 100.0 << " ms" << std::endl;

    tstart = high_resolution_clock::now();
    for (int k = 0; k < 100; k++) {
        solver_BFS.initialize();
    }
    tend = high_resolution_clock::now();
    std::cout << "[BFS] Average time: " << (double)duration_cast<microseconds>(tend - tstart).count() / 1000.0 / 100.0 << " ms" << std::endl;

    return 0;
}
