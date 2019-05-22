#include "maze.h"
#include "mazegraph.h"
#include "mazeutility.h"
#include <fstream>
#include <iostream>

int main()
{
    Amaze::Maze maze;
    Amaze::Utility::loadMazeFromFile(maze, "/home/tokoro/dev/micromouse_mazedat/maze2018halfexp.dat");

    std::cout << "Hello" << std::endl;
    Amaze::Utility::printMaze(maze);

    Amaze::FourWayStepMapGraph<uint16_t, uint8_t> mg(maze);
    std::cout << mg.edgeExist(0, 1) << " " << (int)mg.edgeCost(0, 1) << std::endl;
    std::cout << mg.edgeExist(0, 2) << " " << (int)mg.edgeCost(0, 2) << std::endl;
    std::cout << mg.edgeExist(0, 32) << " " << (int)mg.edgeCost(0, 32) << std::endl;
    std::cout << mg.edgeExist(31, 32) << " " << (int)mg.edgeCost(31, 32) << std::endl;

    Amaze::MazeGraph<uint16_t, uint8_t>* mp = &mg;
    std::cout << mp->edgeExist(0, 1) << " " << (int)mp->edgeCost(0, 1) << std::endl;
    std::cout << mp->edgeExist(0, 2) << " " << (int)mp->edgeCost(0, 2) << std::endl;
    std::cout << mp->edgeExist(0, 32) << " " << (int)mp->edgeCost(0, 32) << std::endl;
    std::cout << mp->edgeExist(31, 32) << " " << (int)mp->edgeCost(31, 32) << std::endl;

    Amaze::SixWayWallNodeGraph<uint16_t, uint8_t> mg2(maze);
    std::pair<bool, uint8_t> e1 = mg2.getEdge(mg2.nodeIdByCoord({ 0, 1, Amaze::DirNorth }), mg2.nodeIdByCoord({ 0, 1, Amaze::DirEast }));
    std::pair<bool, uint8_t> e2 = mg2.getEdge(mg2.nodeIdByCoord({ 0, 1, Amaze::DirNorth }), mg2.nodeIdByCoord({ 0, 2, Amaze::DirNorth }));
    std::pair<bool, uint8_t> e3 = mg2.getEdge(mg2.nodeIdByCoord({ 0, 1, Amaze::DirEast }), mg2.nodeIdByCoord({ 1, 1, Amaze::DirNorth }));
    std::cout << (e1.first ? "true" : "false") << " " << (int)e1.second << std::endl;
    std::cout << (e2.first ? "true" : "false") << " " << (int)e2.second << std::endl;
    std::cout << (e3.first ? "true" : "false") << " " << (int)e3.second << std::endl;

    Amaze::Direction d;
    d = Amaze::DirBack;
    std::cout << d << std::endl;

    std::cout << maze.getGoal() << std::endl;

    return 0;
}
