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
    /// \~japanese
    /// \p Maze クラスのテンプレートパラメータには，迷路の幅・高さの最大値を指定します．
    /// \~english
    /// Specify the maximum width/height of the maze in the template parameter of \p Maze class.
    constexpr uint8_t max_maze_width = 32;
    Maze<max_maze_width> reference_maze;

    Utility::loadMazeFromFile(reference_maze, argv[1]);
    Utility::printMaze(reference_maze);
    std::cout << std::endl;

    /// \~japanese
    /// 迷路グラフを定義し，スタートとゴールの状態を表示します．
    /// \~english
    /// Define a maze graph and display agent states of the start and goals.
    FourWayStepMapGraph mg(reference_maze);
    const uint16_t id_start = mg.getStartNodeId();
    std::vector<uint16_t> goals = mg.getGoalNodeIds();
    std::cout << "The start is " << mg.agentStateByNodeId(id_start) << std::endl;
    std::cout << "The goals are ";
    for (uint16_t id_goal : goals) {
        std::cout << mg.agentStateByNodeId(id_goal) << ", ";
    }
    std::cout << std::endl;

    /// \~japanese
    /// ソルバを初期化して最短経路を導出します．
    /// \~english
    /// Initialize solver and calculate the shortest path.
    auto solver = DStarLite(mg);
    solver.initialize();

    std::vector<AgentState> path = solver.reconstructPath(id_start, goals);
    for (AgentState as : path) {
        std::cout << as << std::endl;
    }

    return 0;
}
