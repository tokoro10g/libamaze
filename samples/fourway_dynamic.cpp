#include "dstarlite.h"
#include "fourwaystepmapgraph.h"
#include "maze.h"
#include "mazeutility.h"
#include "sample_agent.h"
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
    Maze<max_maze_width> maze;

    Utility::loadMazeFromFile(reference_maze, argv[1]);
    Utility::loadEmptyMaze(maze);

    std::vector<Position> goals;
    reference_maze.getGoals(goals);
    maze.addGoals(goals);
    maze.setWall({ 1, 0 }, true);
    maze.setCheckedWall({ 0, 1 }, true);
    maze.setCheckedWall({ 1, 0 }, true);

    FourWayStepMapGraph mg(maze);
    auto solver = DStarLite(mg);

    solver.initialize();
    Utility::printMaze(maze);
    std::vector<uint16_t> goal_ids;
    mg.getGoalNodeIds(goal_ids);
    std::cout << "The start is " << mg.getStartNodeId() << std::endl;
    std::cout << "The goals are ";
    for (auto id_goal : goal_ids) {
        std::cout << id_goal << ", ";
    }
    std::cout << std::endl;

    while (std::find(goal_ids.begin(), goal_ids.end(), solver.getCurrentNodeId()) == goal_ids.end()) {
        std::cout << solver.getCurrentAgentState() << std::endl;
        std::vector<Position> changed_positions;
        solver.preSense();
        senseFourWay(maze, reference_maze, solver.getCurrentAgentState(), changed_positions);
        solver.postSense(changed_positions);
    }
    std::vector<Position> changed_positions;
    senseFourWay(maze, reference_maze, solver.getCurrentAgentState(), changed_positions);
    std::cout << solver.getCurrentAgentState() << std::endl;

    solver.changeDestination(mg.getStartNodeId());
    std::cout << "Now the goal is " << (int)mg.getStartNodeId() << std::endl;

    while (solver.getCurrentNodeId() != mg.getStartNodeId()) {
        std::cout << solver.getCurrentAgentState() << std::endl;
        std::vector<Position> changed_positions;
        solver.preSense();
        senseFourWay(maze, reference_maze, solver.getCurrentAgentState(), changed_positions);
        solver.postSense(changed_positions);
    }
    std::cout << solver.getCurrentAgentState() << std::endl;

    FourWayStepMapGraph<false> mg_fast_run(maze);
    auto solver_fast_run = DStarLite(mg_fast_run);
    solver_fast_run.initialize();

    std::vector<AgentState> path;
    solver_fast_run.reconstructPath(mg_fast_run.getStartNodeId(), goal_ids, path);
    for (AgentState as : path) {
        std::cout << as << std::endl;
    }
    return 0;
}
