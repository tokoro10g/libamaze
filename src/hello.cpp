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

    Amaze::GridMazeGraph mg(maze);
    Amaze::GridMazeGraph::Node n(1);
    std::cout << n.id << std::endl;

    return 0;
}
