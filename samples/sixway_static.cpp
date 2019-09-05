#include "dstarlite.h"
#include "sixwaywallnodeturncostgraph.h"
#include "maze.h"
#include "mazeutility.h"
#include "sample_agent.h"
#include <iostream>
#include <vector>

using namespace Amaze;

int main(int argc, char* argv[])
{
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
    ///
    /// \~english
    ///
    SixWayWallNodeTurnCostGraph mg(reference_maze);
    const uint16_t id_start = mg.getStartNodeId();
    std::vector<uint16_t> goals;
    mg.getGoalNodeIds(goals);
    std::cout << "The start is " << mg.agentStateByNodeId(id_start) << std::endl;
    std::cout << "The goals are ";
    for (uint16_t id_goal : goals) {
        std::cout << mg.agentStateByNodeId(id_goal) << ", ";
    }
    std::cout << std::endl;

    /// \~japanese
    ///
    /// \~english
    ///
    auto solver = DStarLite(mg);
    solver.initialize();

    std::vector<AgentState> path;
    solver.reconstructPath(id_start, goals, path);
    for (AgentState as : path) {
        std::cout << as << std::endl;
    }

    return 0;
}
