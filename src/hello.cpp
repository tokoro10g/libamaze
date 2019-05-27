#include "maze.h"
#include "mazegraph.h"
#include "mazeutility.h"
#include "dstarlite.h"
#include <fstream>
#include <iostream>

using namespace Amaze;

int main()
{
    Maze maze;
    Utility::loadMazeFromFile(maze, "../micromouse_mazedat/maze2018halfexp.dat");

    Utility::printMaze(maze);
    std::cout << std::endl;

    FourWayStepMapGraph<> mg1(maze);
    auto solver1 = DStarLite(mg1);

    auto e11 = mg1.getEdge({ 0, 1, North }, { 0, 1, East });
    auto e12 = mg1.getEdge({ 0, 1, North }, { 0, 2, North });
    auto e13 = mg1.getEdge({ 0, 1, East },  { 1, 2, North });
    std::cout << (e11.first ? "true" : "false") << " " << (int)e11.second << std::endl;
    std::cout << (e12.first ? "true" : "false") << " " << (int)e12.second << std::endl;
    std::cout << (e13.first ? "true" : "false") << " " << (int)e13.second << std::endl;
    std::cout << std::endl;
    std::cout<< mg1.size <<std::endl;
    std::cout << std::endl;

    FourWayStepMapGraph<uint16_t> mg2(maze);
    auto solver2 = DStarLite(mg2);

    auto e21 = mg2.getEdge({ 0, 1, North }, { 0, 1, East });
    auto e22 = mg2.getEdge({ 0, 1, North }, { 0, 2, North });
    auto e23 = mg2.getEdge({ 0, 1, East },  { 1, 2, North });
    std::cout << (e21.first ? "true" : "false") << " " << (int)e21.second << std::endl;
    std::cout << (e22.first ? "true" : "false") << " " << (int)e22.second << std::endl;
    std::cout << (e23.first ? "true" : "false") << " " << (int)e23.second << std::endl;
    std::cout << std::endl;

    FourWayStepMapGraph<float> mg3(maze);
    auto solver3 = DStarLite(mg3);

    auto e31 = mg3.getEdge({ 0, 1, North }, { 0, 1, East });
    auto e32 = mg3.getEdge({ 0, 1, North }, { 0, 2, North });
    auto e33 = mg3.getEdge({ 0, 1, East },  { 1, 2, North });
    std::cout << (e31.first ? "true" : "false") << " " << e31.second << std::endl;
    std::cout << (e32.first ? "true" : "false") << " " << e32.second << std::endl;
    std::cout << (e33.first ? "true" : "false") << " " << e33.second << std::endl;
    std::cout << std::endl;

    SixWayWallNodeGraph<> mg4(maze);
    auto e41 = mg4.getEdge({ 0, 1, North }, { 0, 1, East });
    auto e42 = mg4.getEdge({ 0, 1, North }, { 0, 2, North });
    auto e43 = mg4.getEdge({ 0, 1, East },  { 1, 1, North });
    std::cout << (e41.first ? "true" : "false") << " " << (int)e41.second << std::endl;
    std::cout << (e42.first ? "true" : "false") << " " << (int)e42.second << std::endl;
    std::cout << (e43.first ? "true" : "false") << " " << (int)e43.second << std::endl;
    std::cout << std::endl;
    std::cout<< mg4.size <<std::endl;
    std::cout << std::endl;

    Direction d;
    d = Back;
    std::cout << d << std::endl;

    std::cout << maze.getGoal() << std::endl;

    return 0;
}
