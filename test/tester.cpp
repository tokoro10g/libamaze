#include "dstarlite.h"
#include "fourwaystepmapgraph.h"
#include "maze.h"
#include "mazeutility.h"
#include "sixwaywallnodegraph.h"
#include "sixwaywallnodeturncostgraph.h"
#include <bitset>
#include <chrono>
#include <iostream>
#include <sstream>

using namespace Amaze;
using namespace std::chrono;

template <uint8_t W>
void senseFourWay(Maze<W>& maze, const Maze<W>& reference_maze, AgentState as, std::vector<Position>& changed_positions)
{
    constexpr int8_t dx[4] = { 0, 1, 0, -1 };
    constexpr int8_t dy[4] = { 1, 0, -1, 0 };
    for (int i = 0; i < 4; i++) {
        Position p = { uint8_t(as.pos.x + dx[i]), uint8_t(as.pos.y + dy[i]) };
        if (p.x > 2 * W || p.y > 2 * W) {
            continue;
        }
        if (maze.isCheckedWall(p)) {
            continue;
        }
        maze.setCheckedWall(p, true);
        if (reference_maze.isSetWall(p)) {
            maze.setWall(p, true);
            Position pto = { uint8_t(p.x + dx[i]), uint8_t(p.y + dy[i]) };
            changed_positions.push_back(pto);
        } else {
            maze.setWall(p, false);
        }
    }
}

template <uint8_t W>
void senseSixWay(Maze<W>& maze, const Maze<W>& reference_maze, AgentState as, std::vector<Position>& changed_positions)
{
    constexpr int8_t dx[8] = { 0, 1, 2, 1, 0, -1, -2, -1 };
    constexpr int8_t dy[8] = { 2, 1, 0, -1, -2, -1, 0, 1 };
    for (int i = 0; i < 8; i++) {
        if (as.pos.x % 2 == 0 && as.pos.y % 2 == 1 && (i == 2 || i == 6)) {
            continue;
        } else if (as.pos.x % 2 == 1 && as.pos.y % 2 == 0 && (i == 0 || i == 4)) {
            continue;
        }
        Position p = { uint8_t(as.pos.x + dx[i]), uint8_t(as.pos.y + dy[i]) };
        if (p.x > 2 * W || p.y > 2 * W) {
            continue;
        }
        if (maze.isCheckedWall(p)) {
            continue;
        }
        maze.setCheckedWall(p, true);
        if (reference_maze.isSetWall(p)) {
            maze.setWall(p, true);
            changed_positions.push_back(p);
        } else {
            maze.setWall(p, false);
        }
    }
}

int main()
{
    high_resolution_clock::time_point tstart = high_resolution_clock::now();
    const std::string maze_data = "1\n"
                                  "32\n"
                                  "32\n"
                                  "19\n"
                                  "20\n"
                                  "9 3 9 3 9 5 5 5 3 9 1 3 9 5 5 5 5 5 3 9 1 3 9 5 5 5 5 5 3 9 1 3\n"
                                  "c 6 a a c 5 5 3 a c 4 6 c 5 5 5 5 3 a c 4 6 c 5 5 5 5 3 a c 4 6\n"
                                  "9 5 6 a 9 5 5 6 a d 1 5 1 5 5 5 3 a c 5 5 5 5 5 5 5 5 6 c 5 5 3\n"
                                  "c 1 7 a c 5 5 3 8 3 c 3 e 9 5 5 6 c 5 5 5 5 5 1 5 5 5 3 9 3 9 2\n"
                                  "9 4 3 a 9 1 3 a a c 3 c 3 a 9 1 3 9 5 5 5 5 3 a 9 1 3 a a c 6 e\n"
                                  "c 1 6 a 8 0 2 a 8 3 c 3 a a 8 0 2 a 9 5 5 5 6 a 8 0 2 a c 5 5 3\n"
                                  "9 4 3 a c 4 6 a a c 3 c 2 a c 4 6 a c 5 5 5 3 a c 4 6 a b 9 3 a\n"
                                  "c 1 6 c 1 5 7 a c 5 4 5 2 a d 5 5 2 9 5 5 5 6 c 5 5 1 6 8 6 c 6\n"
                                  "d 4 3 9 4 5 5 2 9 5 5 5 2 a d 5 5 2 8 5 5 5 3 9 5 1 4 3 8 5 5 7\n"
                                  "9 3 a a 9 3 9 2 a 9 1 3 a a 9 3 b a a 9 1 3 a c 1 4 1 6 a 9 1 3\n"
                                  "8 2 a 8 2 8 2 a a 8 0 2 a a a a a a a 8 0 2 a 9 4 1 4 3 a 8 0 2\n"
                                  "c 6 a a 8 2 8 2 a c 4 6 a a a a a a a 8 4 6 a c 1 4 1 6 a c 4 6\n"
                                  "9 5 6 8 6 c 6 e 8 5 5 5 6 8 6 c 6 a c 4 5 5 4 5 4 5 4 7 c 5 5 3\n"
                                  "c 1 7 8 5 5 5 3 8 5 5 5 7 c 5 5 5 6 d 1 5 1 5 1 5 5 5 3 9 5 1 6\n"
                                  "d 0 7 a 9 1 3 a c 5 5 5 3 b 9 1 3 b d 0 5 0 7 a 9 1 3 a c 1 4 3\n"
                                  "d 0 7 a 8 0 2 a 9 5 5 5 6 a 8 0 2 a 9 4 1 4 3 a 8 0 2 a 9 4 1 6\n"
                                  "d 0 7 a c 4 6 a c 5 5 5 3 a c 4 6 a 8 5 0 5 2 a c 4 6 8 4 1 4 3\n"
                                  "9 4 7 8 5 5 5 6 9 5 5 5 4 4 5 5 5 4 4 5 4 5 6 c 5 5 1 6 9 4 1 6\n"
                                  "c 5 3 8 5 5 5 7 c 5 5 5 3 9 1 5 1 1 1 5 5 5 3 9 3 9 2 b 8 5 4 7\n"
                                  "9 3 a c 5 5 5 3 b 9 1 3 a a e b e a a 9 1 3 8 2 8 2 8 2 a 9 1 3\n"
                                  "8 2 a 9 5 5 5 6 a 8 0 2 a 8 1 4 1 2 a 8 0 2 a 8 2 8 2 a a 8 0 2\n"
                                  "c 6 a c 5 5 5 3 a c 4 6 a a e b e a a c 4 6 8 2 8 2 8 2 a c 4 6\n"
                                  "9 5 6 9 5 5 1 6 8 5 5 5 6 c 1 4 5 6 c 5 5 1 6 8 6 c 6 c 4 5 5 3\n"
                                  "c 5 3 c 5 5 4 3 a 9 3 9 3 9 4 5 5 3 9 5 1 4 3 8 5 5 5 1 1 1 1 2\n"
                                  "9 3 a b 9 1 3 a 8 2 8 2 8 2 9 1 3 a c 1 4 1 6 a 9 1 3 a a a a a\n"
                                  "a a a a 8 0 2 a a 8 2 8 2 a 8 0 2 a 9 4 1 4 3 a 8 0 2 a a a a a\n"
                                  "a c 6 a c 4 6 a 8 2 8 2 a a c 4 6 8 4 1 4 1 6 a c 4 6 a a a a a\n"
                                  "8 1 7 8 5 5 5 2 e c 2 c 6 c 5 5 1 6 d 4 5 4 5 4 5 5 5 6 c 4 4 2\n"
                                  "a c 3 c 5 5 3 a 9 5 4 5 3 9 3 9 2 b 9 5 5 5 1 1 5 1 5 3 b b b a\n"
                                  "a b a 9 5 5 6 a a 9 1 3 a a 8 2 8 2 a 9 1 3 a a 9 2 9 0 0 0 0 2\n"
                                  "a a a c 5 5 3 a a 8 0 2 a 8 2 8 2 a a 8 0 2 a 8 6 8 6 a a e e a\n"
                                  "e e c 5 5 5 6 c 6 c 4 6 c 6 c 6 c 4 6 c 4 6 e c 5 4 5 4 4 5 5 6\n";
    std::istringstream iss(maze_data);

    constexpr uint8_t max_maze_width = 32;
    Maze<max_maze_width> reference_maze;
    Utility::loadMazeFromStream(reference_maze, iss);

    std::vector<Position> goals;
    reference_maze.getGoals(goals);

    Utility::printMaze(reference_maze);
    std::cout << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////

    FourWayStepMapGraph mg1(reference_maze);
    FourWayStepMapGraph<true, uint16_t> mg2(reference_maze);
    FourWayStepMapGraph<true, float> mg3(reference_maze);
    SixWayWallNodeTurnCostGraph mg4(reference_maze);

    auto solver1 = DStarLite(mg1);
    auto solver4 = DStarLite(mg4);

    //////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << std::endl;
    std::cout << "=======Testing D* Lite for 4-way Graph=======" << std::endl;
    std::cout << std::endl;

    solver1.initialize();
    std::vector<decltype(mg1)::NodeId> goal_ids;
    mg1.getGoalNodeIds(goal_ids);
    std::cout << "The start is " << mg1.getStartNodeId() << std::endl;
    std::cout << "The goals are ";
    for (auto id_goal : goal_ids) {
        std::cout << id_goal << ", ";
    }
    std::cout << std::endl;

    while (std::find(goal_ids.begin(), goal_ids.end(), solver1.getCurrentNodeId()) == goal_ids.end()) {
        std::cout << mg1.agentStateByNodeId(solver1.getCurrentNodeId()) << ": " << (int)solver1.getCurrentNodeId() << std::endl;
        solver1.preSense();
        solver1.postSense(std::vector<Position>());
    }
    std::cout << solver1.getCurrentNodeId() << std::endl;

    std::vector<decltype(mg1)::NodeId> path1;
    solver1.reconstructPath(mg1.getStartNodeId(), goal_ids, path1);
    for (auto p : path1) {
        std::cout << p << ", ";
    }
    std::cout << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << std::endl;
    std::cout << "=======Testing D* Lite for 6-way Graph=======" << std::endl;
    std::cout << std::endl;

    solver4.initialize();
    std::vector<decltype(mg4)::NodeId> goal_ids4;
    mg4.getGoalNodeIds(goal_ids4);
    std::cout << "The start is " << mg4.getStartNodeId() << std::endl;
    std::cout << "The goals are ";
    for (auto id_goal : goal_ids4) {
        std::cout << id_goal << ", ";
    }
    std::cout << std::endl;

    while (std::find(goal_ids4.begin(), goal_ids4.end(), solver4.getCurrentNodeId()) == goal_ids4.end()) {
        std::cout << mg4.agentStateByNodeId(solver4.getCurrentNodeId()) << ": " << (int)solver4.getCurrentNodeId() << std::endl;
        solver4.preSense();
        solver4.postSense(std::vector<Position>());
    }
    std::cout << solver4.getCurrentNodeId() << std::endl;

    //return 0;

    //////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << std::endl;
    std::cout << "=======Testing Dynamic Search Using D* Lite for 4-way Graph=======" << std::endl;
    std::cout << std::endl;

    Maze<max_maze_width> maze;
    Utility::loadEmptyMaze(maze);
    maze.addGoals(goals);
    maze.setWall({ 1, 0 }, true);
    maze.setCheckedWall({ 0, 1 }, true);
    maze.setCheckedWall({ 1, 0 }, true);
    FourWayStepMapGraph mg5(maze);
    auto solver5 = DStarLite(mg5);

    solver5.initialize();
    Utility::printMaze(maze);
    std::vector<decltype(mg5)::NodeId> goal_ids5;
    mg5.getGoalNodeIds(goal_ids5);
    std::cout << "The start is " << mg5.getStartNodeId() << std::endl;
    std::cout << "The goals are ";
    for (auto id_goal : goal_ids5) {
        std::cout << id_goal << ", ";
    }
    std::cout << std::endl;

    uint16_t id_last = solver5.getCurrentNodeId();
    while (std::find(goal_ids5.begin(), goal_ids5.end(), solver5.getCurrentNodeId()) == goal_ids5.end()) {
        uint16_t id = solver5.getCurrentNodeId();
        if (id == id_last) {
            std::cout << mg5.agentStateByNodeId(id) << std::endl;
        } else {
            std::cout << mg5.agentStateByEdge(id_last, id) << std::endl;
        }
        std::vector<Position> changed_positions;

        solver5.preSense();
        senseFourWay(maze, reference_maze, mg5.agentStateByNodeId(id), changed_positions);
        solver5.postSense(changed_positions);
        id_last = id;
    }
    std::vector<Position> changed_positions;
    senseFourWay(maze, reference_maze, mg5.agentStateByNodeId(solver5.getCurrentNodeId()), changed_positions);
    std::cout << solver5.getCurrentNodeId() << std::endl;

    //Utility::printMaze(maze);

    solver5.changeDestination(mg5.getStartNodeId());
    goal_ids5.clear();
    solver5.getDestinationNodeIds(goal_ids5);
    std::cout << "Now the goals are ";
    for (auto id_goal : goal_ids5) {
        std::cout << id_goal << ", ";
    }
    std::cout << std::endl;

    //id_last = solver6.getCurrentNodeId();
    //for(int i=0;i<20;i++){
    while (std::find(goal_ids5.begin(), goal_ids5.end(), solver5.getCurrentNodeId()) == goal_ids5.end()) {
        //Utility::printMaze(maze);
        uint16_t id = solver5.getCurrentNodeId();
        if (id == id_last) {
            std::cout << mg5.agentStateByNodeId(id) << std::endl;
        } else {
            std::cout << mg5.agentStateByEdge(id_last, id) << std::endl;
        }
        std::vector<Position> changed_positions;

        solver5.preSense();
        senseFourWay(maze, reference_maze, mg5.agentStateByNodeId(id), changed_positions);
        solver5.postSense(changed_positions);
        id_last = id;
    }
    std::cout << mg5.agentStateByEdge(id_last, solver5.getCurrentNodeId()) << std::endl;

    FourWayStepMapGraph<false> mg5_fast_run(maze);
    auto solver5_fast_run = DStarLite(mg5_fast_run);
    solver5_fast_run.initialize();

    Utility::printMaze(maze);
    std::vector<decltype(mg5)::NodeId> path5;
    goal_ids5.clear();
    mg5.getGoalNodeIds(goal_ids5);
    solver5_fast_run.reconstructPath(mg5_fast_run.getStartNodeId(), goal_ids5, path5);
    for (auto p : path5) {
        std::cout << p << ", ";
    }
    std::cout << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << std::endl;
    std::cout << "=======Testing Dynamic Search Using D* Lite for 6-way Graph=======" << std::endl;
    std::cout << std::endl;

    Utility::loadEmptyMaze(maze);
    maze.addGoals(goals);
    maze.setWall({ 1, 0 }, true);
    maze.setCheckedWall({ 0, 1 }, true);
    maze.setCheckedWall({ 1, 0 }, true);
    SixWayWallNodeTurnCostGraph mg6(maze);
    auto solver6 = DStarLite(mg6);

    solver6.initialize();
    Utility::printMaze(maze);
    std::vector<decltype(mg6)::NodeId> goal_ids6;
    mg6.getGoalNodeIds(goal_ids6);
    std::cout << "The start is " << mg6.getStartNodeId() << std::endl;
    std::cout << "The goals are ";
    for (auto id_goal : goal_ids6) {
        std::cout << id_goal << ", ";
    }
    std::cout << std::endl;

    id_last = solver6.getCurrentNodeId();
    //for(int i=0;i<40;i++){
    while (std::find(goal_ids6.begin(), goal_ids6.end(), solver6.getCurrentNodeId()) == goal_ids6.end()) {
        uint16_t id = solver6.getCurrentNodeId();
        if (id == id_last) {
            std::cout << mg6.agentStateByNodeId(id) << std::endl;
        } else {
            std::cout << mg6.agentStateByEdge(id_last, id) << std::endl;
        }
        std::vector<Position> changed_positions;

        solver6.preSense();
        senseSixWay(maze, reference_maze, mg6.agentStateByNodeId(id), changed_positions);
        solver6.postSense(changed_positions);
        id_last = id;
    }
    std::cout << mg6.agentStateByEdge(id_last, solver6.getCurrentNodeId()) << std::endl;

    //Utility::printMaze(maze);

    solver6.changeDestination(mg6.getStartNodeId());
    goal_ids6.clear();
    solver6.getDestinationNodeIds(goal_ids6);
    std::cout << "Now the goals are ";
    for (auto id_goal : goal_ids6) {
        std::cout << id_goal << ", ";
    }
    std::cout << std::endl;

    //id_last = solver6.getCurrentNodeId();
    //for(int i=0;i<40;i++){
    while (std::find(goal_ids6.begin(), goal_ids6.end(), solver6.getCurrentNodeId()) == goal_ids6.end()) {
        //Utility::printMaze(maze);
        uint16_t id = solver6.getCurrentNodeId();
        if (id == id_last) {
            std::cout << mg6.agentStateByNodeId(id) << std::endl;
        } else {
            std::cout << mg6.agentStateByEdge(id_last, id) << std::endl;
        }
        std::vector<Position> changed_positions;

        solver6.preSense();
        senseSixWay(maze, reference_maze, mg6.agentStateByNodeId(id), changed_positions);
        solver6.postSense(changed_positions);
        id_last = id;
    }
    std::cout << mg6.agentStateByEdge(id_last, solver6.getCurrentNodeId()) << std::endl;

    high_resolution_clock::time_point tend = high_resolution_clock::now();
    std::cout << "Time elapsed: " << (float)duration_cast<microseconds>(tend - tstart).count() / 1000.f << " ms" << std::endl;
    return 0;
}
