#pragma once

#include "maze.h"
#include <fstream>
#include <iostream>
#include <string>

namespace Amaze {

namespace Utility {
    void loadMazeFromStream(Maze& maze, std::istream& fs);
    void loadMazeFromStdin(Maze& maze);
    void loadMazeFromFile(Maze& maze, std::string filename);
    void loadEmptyMaze(int w, int h, int x, int y, Maze& maze);
    void printMaze(const Maze& maze, const int m = 1);
}

std::ostream& operator<<(std::ostream& os, Direction d);
std::ostream& operator<<(std::ostream& os, Coord c);

}
