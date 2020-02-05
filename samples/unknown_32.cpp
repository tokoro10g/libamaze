#include "astar.h"
#include "dstarlite.h"
#include "fourwaystepmapgraph.h"
#include "maze.h"
#include "mazeutility.h"
#include "sixwaywallnodegraph.h"
#include "sixwaywallnodeturncostgraph.h"
#include <chrono>
#include <iostream>
#include <vector>

using namespace Amaze;
using namespace std::chrono;

int main()
{
    constexpr uint8_t max_maze_width = 32;
    Maze<max_maze_width> maze;

    Utility::loadEmptyMaze(maze);
    maze.goals.push_back({ 61, 62 });
    maze.goals.push_back({ 62, 61 });
    maze.goals.push_back({ 62, 62 });
    maze.setWall({ 1, 0 }, true);
    maze.setCheckedWall({ 0, 1 }, true);
    maze.setCheckedWall({ 1, 0 }, true);
    Utility::printMaze(maze);
    std::cout << std::endl;

    // 迷路グラフを定義し，スタートとゴールの状態を表示します．
    //
    // Defines a maze graph and display agent states of the start and goals.
    //
    FourWayStepMapGraph mg_four(maze);
    SixWayWallNodeGraph mg_six(maze);
    SixWayWallNodeTurnCostGraph mg_six_cost(maze);

    // ソルバを初期化して最短経路を導出します．
    //
    // Initializes the solver and calculate the shortest path.
    //
    auto solver_four = DStarLite(&mg_four);
    auto solver_six = DStarLite(&mg_six);
    auto solver_six_cost = DStarLite(&mg_six_cost);
    auto solver_four_astar = AStar(&mg_four);
    auto solver_six_astar = AStar(&mg_six);
    auto solver_six_cost_astar = AStar(&mg_six_cost);

    high_resolution_clock::time_point tstart, tend;

#if 1
    std::cout << "FourWayStepMapGraph: " << std::endl;
    tstart = high_resolution_clock::now();
    for (int k = 0; k < 100; k++) {
        solver_four.initialize();
    }
    tend = high_resolution_clock::now();
    std::cout << "[D* Lite] Average time: " << (double)duration_cast<microseconds>(tend - tstart).count() / 1000.0 / 100.0 << " ms" << std::endl;
    tstart = high_resolution_clock::now();
    for (int k = 0; k < 100; k++) {
        solver_four_astar.initialize();
    }
    tend = high_resolution_clock::now();
    std::cout << "[A*] Average time: " << (double)duration_cast<microseconds>(tend - tstart).count() / 1000.0 / 100.0 << " ms" << std::endl;
#endif

#if 1
    std::cout << "SixWayWallNodeGraph: " << std::endl;
    tstart = high_resolution_clock::now();
    for (int k = 0; k < 100; k++) {
        solver_six.initialize();
    }
    tend = high_resolution_clock::now();
    std::cout << "[D* Lite] Average time: " << (double)duration_cast<microseconds>(tend - tstart).count() / 1000.0 / 100.0 << " ms" << std::endl;
    tstart = high_resolution_clock::now();
    for (int k = 0; k < 100; k++) {
        solver_six_astar.initialize();
    }
    tend = high_resolution_clock::now();
    std::cout << "[A*] Average time: " << (double)duration_cast<microseconds>(tend - tstart).count() / 1000.0 / 100.0 << " ms" << std::endl;
#endif

#if 1
    std::cout << "SixWayWallNodeTurnCostGraph: " << std::endl;
    tstart = high_resolution_clock::now();
    for (int k = 0; k < 100; k++) {
        solver_six_cost.initialize();
    }
    tend = high_resolution_clock::now();
    std::cout << "[D* Lite] Average time: " << (double)duration_cast<microseconds>(tend - tstart).count() / 1000.0 / 100.0 << " ms" << std::endl;
    tstart = high_resolution_clock::now();
    for (int k = 0; k < 100; k++) {
        solver_six_cost_astar.initialize();
    }
    tend = high_resolution_clock::now();
    std::cout << "[A*] Average time: " << (double)duration_cast<microseconds>(tend - tstart).count() / 1000.0 / 100.0 << " ms" << std::endl;
#endif

    return 0;
}
