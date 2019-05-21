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
    std::cout << mg.isConnected(0, 1) << " " << (int)mg.edgeCost(0, 1) << std::endl;
    std::cout << mg.isConnected(0, 2) << " " << (int)mg.edgeCost(0, 2) << std::endl;
    std::cout << mg.isConnected(0, 32) << " " << (int)mg.edgeCost(0, 32) << std::endl;
    std::cout << mg.isConnected(31, 32) << " " << (int)mg.edgeCost(31, 32) << std::endl;

    Amaze::MazeGraph<uint16_t, uint8_t>* mp = &mg;
    std::cout << mp->isConnected(0, 1) << " " << (int)mp->edgeCost(0, 1) << std::endl;
    std::cout << mp->isConnected(0, 2) << " " << (int)mp->edgeCost(0, 2) << std::endl;
    std::cout << mp->isConnected(0, 32) << " " << (int)mp->edgeCost(0, 32) << std::endl;
    std::cout << mp->isConnected(31, 32) << " " << (int)mp->edgeCost(31, 32) << std::endl;

    Amaze::Direction d;
    d = Amaze::DirBack;
    std::cout << d << std::endl;

    std::cout << maze.getGoal() << std::endl;

    return 0;
}
