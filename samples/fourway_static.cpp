//===-- samples/fourway_static.cpp - Static Search Example ----------------===//
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
#include "dstarlite.h"
#include "fourwaystepmapgraph.h"
#include "maze.h"
#include "mazeutility.h"
#include <iostream>
#include <vector>

using namespace Amaze;

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

    // 仮想迷路をグラフで表現します．FourWayStepMapGraphクラス(区画を基準とした歩数マップ)を使用します．
    //
    // Defines a graph representation of the virtual maze.
    // This example uses FourWayStepMapGraph class which represents a step map based on cells.
    //
    FourWayStepMapGraph mg(maze);
    // 参照迷路の形状とスタート・ゴールの情報を表示します．
    //
    // Prints the reference maze, and show information of the start and goals.
    //
    Utility::printMaze(maze);
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
    auto solver = DStarLite(&mg);
    solver.initialize();

    std::cout << "Found path: " << std::endl;
    std::vector<AgentState> path = solver.reconstructPath(id_start, goal_ids);
    for (auto as : path) {
        std::cout << as << std::endl;
    }

    return 0;
}
