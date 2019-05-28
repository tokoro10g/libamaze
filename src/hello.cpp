#include "dstarlite.h"
#include "maze.h"
#include "mazegraph.h"
#include "mazeutility.h"
#include <fstream>
#include <iostream>

using namespace Amaze;

int main()
{
    constexpr uint8_t max_maze_width = 32;
    Maze maze(max_maze_width, max_maze_width);
    Utility::loadMazeFromFile(maze, "../micromouse_mazedat/maze2017halfexp.dat");

    Utility::printMaze(maze);
    std::cout << std::endl;

    FourWayStepMapGraph<> mg1(maze);
    auto solver1 = DStarLite(mg1);

    auto e11 = mg1.getEdge({ 0, 1, North }, { 0, 1, East });
    auto e12 = mg1.getEdge({ 0, 1, North }, { 0, 2, North });
    auto e13 = mg1.getEdge({ 0, 1, East }, { 1, 2, North });
    std::cout << (e11.first ? "true" : "false") << " " << (int)e11.second << std::endl;
    std::cout << (e12.first ? "true" : "false") << " " << (int)e12.second << std::endl;
    std::cout << (e13.first ? "true" : "false") << " " << (int)e13.second << std::endl;
    std::cout << std::endl;
    std::cout << mg1.size << std::endl;
    std::cout << std::endl;

    FourWayStepMapGraph<uint16_t> mg2(maze);

    auto e21 = mg2.getEdge({ 0, 1, North }, { 0, 1, East });
    auto e22 = mg2.getEdge({ 0, 1, North }, { 0, 2, North });
    auto e23 = mg2.getEdge({ 0, 1, East }, { 1, 2, North });
    std::cout << (e21.first ? "true" : "false") << " " << (int)e21.second << std::endl;
    std::cout << (e22.first ? "true" : "false") << " " << (int)e22.second << std::endl;
    std::cout << (e23.first ? "true" : "false") << " " << (int)e23.second << std::endl;
    std::cout << std::endl;

    FourWayStepMapGraph<float> mg3(maze);

    auto e31 = mg3.getEdge({ 0, 1, North }, { 0, 1, East });
    auto e32 = mg3.getEdge({ 0, 1, North }, { 0, 2, North });
    auto e33 = mg3.getEdge({ 0, 1, East }, { 1, 2, North });
    std::cout << (e31.first ? "true" : "false") << " " << e31.second << std::endl;
    std::cout << (e32.first ? "true" : "false") << " " << e32.second << std::endl;
    std::cout << (e33.first ? "true" : "false") << " " << e33.second << std::endl;
    std::cout << std::endl;

    SixWayWallNodeGraph<uint16_t> mg4(maze);
    auto solver4 = DStarLite(mg4);
    auto e41 = mg4.getEdge({ 0, 1, North }, { 0, 1, East });
    auto e42 = mg4.getEdge({ 0, 1, North }, { 0, 2, North });
    auto e43 = mg4.getEdge({ 0, 1, East }, { 1, 1, North });
    std::cout << (e41.first ? "true" : "false") << " " << (int)e41.second << std::endl;
    std::cout << (e42.first ? "true" : "false") << " " << (int)e42.second << std::endl;
    std::cout << (e43.first ? "true" : "false") << " " << (int)e43.second << std::endl;
    std::cout << std::endl;
    std::cout << mg4.size << std::endl;
    std::cout << std::endl;

    Direction d;
    d = Back;
    std::cout << d << std::endl;

    std::cout << maze.getGoal() << std::endl;

    std::cout << std::endl;
    std::cout << "=======Testing D* Lite for 4-way Graph=======" << std::endl;
    std::cout << std::endl;

    solver1.initialize();
    decltype(mg1)::NodeId id_goal = mg1.getGoalNodeId();
    std::cout << "The start is " << mg4.getStartNodeId() << std::endl;
    std::cout << "The goal is " << id_goal << std::endl;
    std::cout << std::endl;

    while (solver1.getCurrentNodeId() != id_goal) {
        std::cout << solver1.getCurrentNodeId() << std::endl;
        solver1.postSense(std::vector<Coordinates>());
    }
    std::cout << id_goal << std::endl;

    std::cout << std::endl;
    std::cout << "=======Testing D* Lite for 6-way Graph=======" << std::endl;
    std::cout << std::endl;

    solver4.initialize();
    decltype(mg4)::NodeId id_goal4 = mg4.getGoalNodeId();
    std::cout << "The start is " << mg4.getStartNodeId() << std::endl;
    std::cout << "The goal is " << id_goal4 << std::endl;
    std::cout << std::endl;

    while (solver4.getCurrentNodeId() != id_goal4) {
        std::cout << solver4.getCurrentNodeId() << std::endl;
        solver4.postSense(std::vector<Coordinates>());
    }
    std::cout << id_goal4 << std::endl;

    return 0;
}
