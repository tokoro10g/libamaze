//===-- samples/sixway_dynamic.cpp - Dynamic Search Example ---------------===//
//
// Part of libamaze, under the MIT License.
// See https://opensource.org/licenses/MIT for license information.
// SPDX-License-Identifier: MIT
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file is an example code which uses libamaze to simulate the exploration
/// and trial runs of a micromouse robot.
///
//===----------------------------------------------------------------------===//

#include "agenthelper.h"
#include "astar.h"
#include "bfs.h"
#include "dstarlite.h"
#include "maze.h"
#include "mazeutility.h"
#include "sixwaywallnodegraph.h"
#include "sixwaywallnodeturncostgraph.h"
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

using namespace Amaze;
using namespace std::chrono;

#define ENABLE_TURN_COST 0

void showUsage(std::string name)
{
    std::cout << "usage:" << std::endl;
    std::cout << name << " filename" << std::endl;
}

int main(int argc, char* argv[])
{
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
    // Specify the maximum width/height of the maze in the template parameter of Maze class.
    // In this example, we prepare two kinds of maze data; virtual and reference mazes.
    // The virtual maze (virtual_maze) represents the partial maze information the robot owns,
    // and the data is updated by referring to the reference maze (reference_maze) during exploration.
    // The virtual maze is initialized with empty data, and we load reference maze data from the file
    // passed by the command-line argument.
    //
    constexpr uint8_t max_maze_width = 32;
    Maze<max_maze_width> reference_maze;
    Maze<max_maze_width> virtual_maze;
    Utility::loadMazeFromFile(reference_maze, argv[1]);
    Utility::loadEmptyMaze(virtual_maze);

    // 仮想迷路のゴールを参照迷路のゴールと同一に設定します．
    //
    // Sets the goals of the virtal maze to those of the reference maze.
    //
    auto goals = reference_maze.goals;
    virtual_maze.goals = goals;
    // スタート区画の前壁(0, 1)と右壁(1, 0)はスタート時に有無が確定しており，右壁があります．
    //
    // The front (0, 1) and the right (1, 0) of the start cell are checked before exploration,
    // and there is a wall on the right.
    //
    virtual_maze.setWall({ 1, 0 }, true);
    virtual_maze.setCheckedWall({ 0, 1 }, true);
    virtual_maze.setCheckedWall({ 1, 0 }, true);

    // 仮想迷路をグラフで表現します．FourWayStepMapGraphクラス(区画を基準とした歩数マップ)を使用します．
    //
    // Defines a graph representation of the virtual maze.
    // This example uses FourWayStepMapGraph class which represents a step map based on cells.
    //
#if ENABLE_TURN_COST
    SixWayWallNodeTurnCostGraph mg(virtual_maze);
#else
    SixWayWallNodeGraph mg(virtual_maze);
#endif
    // グラフのポインタをD* Liteソルバに渡して，ソルバを初期化します．
    //
    // Passes the pointer of the graph to a D* Lite solver and initializes the solver.
    //
    auto solver_DSL = DStarLite(&mg);
    auto solver_AS = AStar(&mg);
    auto solver_BFS = BFS(&mg);
    high_resolution_clock::time_point tstart, tend;

    tstart = high_resolution_clock::now();
    solver_DSL.initialize();
    using AH = AgentHelper<decltype(mg), decltype(solver_DSL)>;

    // 参照迷路の形状とスタート・ゴールの情報を表示します．
    //
    // Prints the reference maze, and show information of the start and goals.
    //
    Utility::printMaze(reference_maze);
    auto goal_ids = mg.goalNodeIds();

    /*
     * 行きの探索 Exploration to the goal
     */

    // ゴールノードのうちどれか1つにたどり着くまで探索を行います．
    //
    // Repeats until the solver arrives one of the goals.
    //
    while (goal_ids.find(solver_DSL.currentNodeId()) == goal_ids.end()) {
        // 現在のソルバの状態から，つぎにセンシングする壁の位置のリストを取得します．
        //
        // Obtains a list of wall positions to sense next according to the current solver state.
        //
        auto sense_positions = AH::currentSensePositions(solver_DSL);
        // センシング前のソルバの処理を行います．通常のD* Liteソルバでは何もしません．
        //
        // Performs a solver action before sensing. A normal D* Lite solver does not do anything.
        //
        solver_DSL.preSense(sense_positions);
        // センシングを行います．このサンプルでは，参照迷路の中の指定した位置にある壁の情報を仮想迷路に反映し，
        //
        // 壁のデータに変更が加わった座標のリストを取得しています．(実装はsample_agent.hを参照)
        //
        auto changed_positions = AH::sense(virtual_maze, reference_maze, sense_positions);
        // センシング後のソルバの処理を行います．変更された壁の座標を渡して，つぎに取るべき行動を決定します．
        // ソルバの状態はこの処理で更新されます．
        //
        // Performs a solver action after sensing. This process determines the next action according to the positions of changed walls.
        // The solver state is updated during this process.
        //
        solver_DSL.postSense(changed_positions);
        // ここでsolver.currentAgentState()を取得すると，つぎの状態が取得できます．
        //
        // The return value of solver.currentAgentState() here corresponds to the next agent state.
        //
    }
    tend = high_resolution_clock::now();
    std::cout << "[D* Lite] Time: " << (double)duration_cast<microseconds>(tend - tstart).count() / 1000.0 << " ms" << std::endl;

    /*
     * A*
     */
    Utility::loadEmptyMaze(virtual_maze);
    virtual_maze.goals = goals;
    virtual_maze.setWall({ 1, 0 }, true);
    virtual_maze.setCheckedWall({ 0, 1 }, true);
    virtual_maze.setCheckedWall({ 1, 0 }, true);

    tstart = high_resolution_clock::now();
    solver_AS.initialize();
    using AH2 = AgentHelper<decltype(mg), decltype(solver_AS)>;
    while (goal_ids.find(solver_AS.currentNodeId()) == goal_ids.end()) {
        auto sense_positions = AH2::currentSensePositions(solver_AS);
        solver_AS.preSense(sense_positions);
        auto changed_positions = AH2::sense(virtual_maze, reference_maze, sense_positions);
        solver_AS.postSense(changed_positions);
    }
    tend = high_resolution_clock::now();
    std::cout << "[A*] Time: " << (double)duration_cast<microseconds>(tend - tstart).count() / 1000.0 << " ms" << std::endl;

    /*
     * BFS
     */
    Utility::loadEmptyMaze(virtual_maze);
    virtual_maze.goals = goals;
    virtual_maze.setWall({ 1, 0 }, true);
    virtual_maze.setCheckedWall({ 0, 1 }, true);
    virtual_maze.setCheckedWall({ 1, 0 }, true);

    tstart = high_resolution_clock::now();
    solver_BFS.initialize();
    using AH3 = AgentHelper<decltype(mg), decltype(solver_BFS)>;
    while (goal_ids.find(solver_BFS.currentNodeId()) == goal_ids.end()) {
        auto sense_positions = AH3::currentSensePositions(solver_BFS);
        solver_BFS.preSense(sense_positions);
        auto changed_positions = AH3::sense(virtual_maze, reference_maze, sense_positions);
        solver_BFS.postSense(changed_positions);
    }
    tend = high_resolution_clock::now();
    std::cout << "[BFS] Time: " << (double)duration_cast<microseconds>(tend - tstart).count() / 1000.0 << " ms" << std::endl;

    return 0;
}
