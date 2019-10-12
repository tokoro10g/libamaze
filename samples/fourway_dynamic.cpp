#include "agentstrategy.h"
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

    std::vector<Position> goals = reference_maze.goals;
    maze.goals.insert(maze.goals.end(), goals.begin(), goals.end());
    maze.setWall({ 1, 0 }, true);
    maze.setCheckedWall({ 0, 1 }, true);
    maze.setCheckedWall({ 1, 0 }, true);

    FourWayStepMapGraph mg(maze);
    auto solver = DStarLite(&mg);
    using AS = AgentStrategy<decltype(mg), decltype(solver)>;

    solver.initialize();
    Utility::printMaze(reference_maze);
    std::vector<uint16_t> goal_ids = mg.goalNodeIds();
    std::cout << "The start is " << mg.startNodeId() << std::endl;
    std::cout << "The goals are ";
    for (auto id_goal : goal_ids) {
        std::cout << id_goal << ", ";
    }
    std::cout << std::endl;

    while (std::find(goal_ids.begin(), goal_ids.end(), solver.currentNodeId()) == goal_ids.end()) {
        std::cout << solver.currentAgentState() << std::endl;
        solver.preSense(std::vector<Position>());
        std::vector<Position> changed_positions = sense(maze, reference_maze, AS::currentSensePositions(solver));
        solver.postSense(changed_positions);
    }
    std::vector<Position> changed_positions = sense(maze, reference_maze, AS::currentSensePositions(solver));
    std::cout << solver.currentAgentState() << std::endl;

    solver.changeDestination(mg.startNodeId());
    std::cout << "Now the goal is " << (int)mg.startNodeId() << std::endl;

    while (solver.currentNodeId() != mg.startNodeId()) {
        std::cout << solver.currentAgentState() << std::endl;
        solver.preSense(std::vector<Position>());
        std::vector<Position> changed_positions = sense(maze, reference_maze, AS::currentSensePositions(solver));
        solver.postSense(changed_positions);
    }
    std::cout << solver.currentAgentState() << std::endl;

    FourWayStepMapGraph<false> mg_fast_run(maze);
    solver.changeMazeGraph(&mg_fast_run);

    std::vector<AgentState> path = solver.reconstructPath(mg_fast_run.startNodeId(), goal_ids);
    for (AgentState as : path) {
        std::cout << as << std::endl;
    }
    return 0;
}
