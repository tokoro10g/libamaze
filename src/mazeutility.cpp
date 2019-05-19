#include "mazeutility.h"

#include <fstream>
#include <iostream>

namespace Amaze {
namespace Utility {

    void loadMazeFromStream(Maze& maze, std::istream& fs)
    {
        int type, w, h;
        fs >> type;
        fs >> w >> h;
        maze.resize(w, h);
        maze.setType(type);
        if (type == 1) {
            int x, y;
            fs >> x >> y;
            maze.setGoal(x, y);
        } else {
            maze.setGoal(7, 7);
        }
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                char ch;
                CellData cell = { 0 };
                fs >> ch;
                if (ch >= '0' && ch <= '9') {
                    cell.byte = ch - '0';
                } else if (ch >= 'a' && ch <= 'f') {
                    cell.byte = ch + 0xa - 'a';
                } else if (ch == ' ' || ch == '\n' || ch == '\r') {
                    j--;
                    continue;
                } else {
                    std::cerr << "Invalid maze data" << std::endl;
                    return;
                }
                maze.setCell((h - i - 1) * w + j, cell);
            }
        }
    }

    void loadMazeFromStdin(Maze& maze)
    {
        loadMazeFromStream(maze, std::cin);
    }

    void loadMazeFromFile(Maze& maze, std::string filename)
    {
        std::ifstream fs;
        fs.open(filename);
        loadMazeFromStream(maze, fs);
        fs.close();
    }

    void loadEmptyMaze(int w, int h, int x, int y, Maze& maze)
    {
        maze.resize(w, h);
        maze.setType(1);
        maze.setGoal(x, y);
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                CellData cell = { 0 };
                if (j == 0) {
                    cell.bits.WEST = 1;
                } else if (j == w - 1) {
                    cell.bits.EAST = 1;
                }
                if (i == 0) {
                    cell.bits.SOUTH = 1;
                } else if (i == h - 1) {
                    cell.bits.NORTH = 1;
                }
                if (i == 0 && j == 0) {
                    cell.bits.EAST = 1;
                    cell.bits.CHECKED_EAST = 1;
                }
                if (i == 0 && j == 1) {
                    cell.bits.WEST = 1;
                    cell.bits.CHECKED_WEST = 1;
                }
                maze.setCell(i * w + j, cell);
            }
        }
    }

    void printMaze(const Maze& maze, const int m)
    {
        int w = maze.getWidth();
        int h = maze.getHeight();

        std::cout << "_";
        for (int i = 0; i < w * m; i++) {
            std::cout << "__";
        }
        std::cout << std::endl;

        for (int cursory = h - 1; cursory >= 0; cursory--) {
            for (int j = 0; j < m; j++) {
                for (int cursorx = 0; cursorx < w; cursorx++) {
                    int index = cursorx + w * cursory;
                    CellData cd = maze.getCell(index);
                    if (cursorx == 0)
                        std::cout << "|";

                    for (int i = 0; i < m; i++) {
                        if (cd.bits.SOUTH && j == m - 1) {
                            std::cout << "\x1b[4m"; // underline
                        }
                        std::cout << " ";
                        std::cout << "\x1b[0m";
                    }

                    for (int i = 0; i < m; i++) {
                        if (cd.bits.EAST && i == m - 1) {
                            std::cout << "|";
                        } else if (cd.bits.SOUTH && j == m - 1) {
                            std::cout << "\x1b[4m";
                            std::cout << " ";
                            std::cout << "\x1b[0m";
                        } else {
                            std::cout << " ";
                        }
                    }
                    if (cursorx == w - 1) {
                        std::cout << std::endl;
                    }
                }
            }
        }
    }
}
}
