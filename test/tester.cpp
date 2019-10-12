#include "agentstrategy.h"
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
std::vector<Amaze::Position> sense(Amaze::Maze<W>& maze, const Amaze::Maze<W>& reference_maze, const std::vector<Amaze::Position>& sense_positions)
{
    std::vector<Amaze::Position> changed_positions;
    for (auto p : sense_positions) {
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
    return changed_positions;
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

    std::vector<Position> goals = reference_maze.goals;

    Utility::printMaze(reference_maze);
    std::cout << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////

    FourWayStepMapGraph mg1(reference_maze);
    FourWayStepMapGraph<true, uint16_t> mg2(reference_maze);
    SixWayWallNodeTurnCostGraph mg4(reference_maze);

    auto solver1 = DStarLite(&mg1);
    using AS1 = AgentStrategy<decltype(mg1), decltype(solver1)>;
    auto solver4 = DStarLite(&mg4);

    //////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << std::endl;
    std::cout << "=======Testing D* Lite for 4-way Graph=======" << std::endl;
    std::cout << std::endl;

    solver1.initialize();
    std::vector<decltype(mg1)::NodeId> goal_ids = mg1.goalNodeIds();
    std::cout << "The start is " << mg1.startNodeId() << std::endl;
    std::cout << "The goals are ";
    for (auto id_goal : goal_ids) {
        std::cout << id_goal << ", ";
    }
    std::cout << std::endl;

    while (std::find(goal_ids.begin(), goal_ids.end(), solver1.currentNodeId()) == goal_ids.end()) {
        std::cout << solver1.currentAgentState() << ": " << (int)solver1.currentNodeId() << std::endl;
        std::vector<Position> positions = AS1::currentSensePositions(solver1);
        std::cout << "Next sense: ";
        for (auto p : positions) {
            std::cout << p;
        }
        std::cout << std::endl;
        solver1.preSense(std::vector<Position>());
        solver1.postSense(std::vector<Position>());
    }
    std::cout << solver1.currentNodeId() << std::endl;

    std::vector<AgentState> path1 = solver1.reconstructPath(mg1.startNodeId(), goal_ids);
    for (auto p : path1) {
        std::cout << p << ", ";
    }
    std::cout << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << std::endl;
    std::cout << "=======Testing D* Lite for 6-way Graph=======" << std::endl;
    std::cout << std::endl;

    solver4.initialize();
    std::vector<decltype(mg4)::NodeId> goal_ids4 = mg4.goalNodeIds();
    std::cout << "The start is " << mg4.startNodeId() << std::endl;
    std::cout << "The goals are ";
    for (auto id_goal : goal_ids4) {
        std::cout << id_goal << ", ";
    }
    std::cout << std::endl;

    while (std::find(goal_ids4.begin(), goal_ids4.end(), solver4.currentNodeId()) == goal_ids4.end()) {
        std::cout << solver4.currentAgentState() << ": " << (int)solver4.currentNodeId() << std::endl;
        solver4.preSense(std::vector<Position>());
        solver4.postSense(std::vector<Position>());
    }
    std::cout << solver4.currentNodeId() << std::endl;

    //return 0;

    //////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << std::endl;
    std::cout << "=======Testing Dynamic Search Using D* Lite for 4-way Graph=======" << std::endl;
    std::cout << std::endl;

    Maze<max_maze_width> maze;
    Utility::loadEmptyMaze(maze);
    maze.goals.insert(maze.goals.end(), goals.begin(), goals.end());
    maze.setWall({ 1, 0 }, true);
    maze.setCheckedWall({ 0, 1 }, true);
    maze.setCheckedWall({ 1, 0 }, true);
    FourWayStepMapGraph mg5(maze);
    auto solver5 = DStarLite(&mg5);
    using AS5 = AgentStrategy<decltype(mg5), decltype(solver5)>;

    solver5.initialize();
    Utility::printMaze(maze);
    std::vector<decltype(mg5)::NodeId> goal_ids5 = mg5.goalNodeIds();
    std::cout << "The start is " << mg5.startNodeId() << std::endl;
    std::cout << "The goals are ";
    for (auto id_goal : goal_ids5) {
        std::cout << id_goal << ", ";
    }
    std::cout << std::endl;

    while (std::find(goal_ids5.begin(), goal_ids5.end(), solver5.currentNodeId()) == goal_ids5.end()) {
        std::cout << solver5.currentAgentState() << std::endl;

        solver5.preSense(std::vector<Position>());
        std::vector<Position> changed_positions = sense(maze, reference_maze, AS5::currentSensePositions(solver5));
        solver5.postSense(changed_positions);
    }
    std::vector<Position> changed_positions = sense(maze, reference_maze, AS5::currentSensePositions(solver5));
    std::cout << solver5.currentNodeId() << std::endl;

    //Utility::printMaze(maze);

    solver5.changeDestination(mg5.startNodeId());
    goal_ids5 = solver5.destinationNodeIds();
    std::cout << "Now the goals are ";
    for (auto id_goal : goal_ids5) {
        std::cout << id_goal << ", ";
    }
    std::cout << std::endl;

    //for(int i=0;i<20;i++){
    while (std::find(goal_ids5.begin(), goal_ids5.end(), solver5.currentNodeId()) == goal_ids5.end()) {
        //Utility::printMaze(maze);
        std::cout << solver5.currentAgentState() << std::endl;

        solver5.preSense(std::vector<Position>());
        std::vector<Position> changed_positions = sense(maze, reference_maze, AS5::currentSensePositions(solver5));
        solver5.postSense(changed_positions);
    }
    std::cout << solver5.currentAgentState() << std::endl;

    FourWayStepMapGraph<false> mg5_fast_run(maze);
    solver5.changeMazeGraph(&mg5_fast_run);

    Utility::printMaze(maze);
    goal_ids5 = mg5.goalNodeIds();
    std::vector<AgentState> path5 = solver5.reconstructPath(mg5_fast_run.startNodeId(), goal_ids5);
    for (auto p : path5) {
        std::cout << p << ", ";
    }
    std::cout << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << std::endl;
    std::cout << "=======Testing Dynamic Search Using D* Lite for 6-way Graph=======" << std::endl;
    std::cout << std::endl;

    Utility::loadEmptyMaze(maze);
    maze.goals.insert(maze.goals.end(), goals.begin(), goals.end());
    maze.setWall({ 1, 0 }, true);
    maze.setCheckedWall({ 0, 1 }, true);
    maze.setCheckedWall({ 1, 0 }, true);
    SixWayWallNodeTurnCostGraph mg6(maze);
    auto solver6 = DStarLite(&mg6);
    using AS6 = AgentStrategy<decltype(mg6), decltype(solver6)>;

    solver6.initialize();
    Utility::printMaze(maze);
    std::vector<decltype(mg6)::NodeId> goal_ids6 = mg6.goalNodeIds();
    std::cout << "The start is " << mg6.startNodeId() << std::endl;
    std::cout << "The goals are ";
    for (auto id_goal : goal_ids6) {
        std::cout << id_goal << ", ";
    }
    std::cout << std::endl;

    //for(int i=0;i<40;i++){
    while (std::find(goal_ids6.begin(), goal_ids6.end(), solver6.currentNodeId()) == goal_ids6.end()) {
        std::cout << solver6.currentAgentState() << std::endl;

        solver6.preSense(std::vector<Position>());
        std::vector<Position> changed_positions = sense(maze, reference_maze, AS6::currentSensePositions(solver6));
        solver6.postSense(changed_positions);
    }
    std::cout << solver6.currentAgentState() << std::endl;
    Utility::printMaze(maze);

    solver6.changeDestination(mg6.startNodeId());
    goal_ids6 = solver6.destinationNodeIds();
    std::cout << "Now the goals are ";
    for (auto id_goal : goal_ids6) {
        std::cout << id_goal << ", ";
    }
    std::cout << std::endl;

    //for(int i=0;i<40;i++){
    while (std::find(goal_ids6.begin(), goal_ids6.end(), solver6.currentNodeId()) == goal_ids6.end()) {
        //Utility::printMaze(maze);
        std::cout << solver6.currentAgentState() << std::endl;

        solver6.preSense(std::vector<Position>());
        std::vector<Position> changed_positions = sense(maze, reference_maze, AS6::currentSensePositions(solver6));
        solver6.postSense(changed_positions);
    }
    std::cout << solver6.currentAgentState() << std::endl;
    Utility::printMaze(maze);

    goal_ids6 = mg6.goalNodeIds();
    solver6.changeDestinations(goal_ids6);
    std::cout << "Now the goals are ";
    for (auto id_goal : goal_ids6) {
        std::cout << id_goal << ", ";
    }
    std::cout << std::endl;

    //for(int i=0;i<40;i++){
    while (std::find(goal_ids6.begin(), goal_ids6.end(), solver6.currentNodeId()) == goal_ids6.end()) {
        //Utility::printMaze(maze);
        std::cout << solver6.currentAgentState() << std::endl;

        solver6.preSense(std::vector<Position>());
        std::vector<Position> changed_positions = sense(maze, reference_maze, AS6::currentSensePositions(solver6));
        solver6.postSense(changed_positions);
    }
    std::cout << solver6.currentAgentState() << std::endl;
    Utility::printMaze(maze);

    high_resolution_clock::time_point tend = high_resolution_clock::now();
    std::cout << "Time elapsed: " << (float)duration_cast<microseconds>(tend - tstart).count() / 1000.f << " ms" << std::endl;
    return 0;
}
