#pragma once

#include "maze.h"
#include <cassert>
#include <fstream>
#include <iostream>
#include <string>

namespace Amaze {

namespace Utility {
    template <uint8_t W>
    bool loadMazeFromStream(Maze<W>& maze, std::istream& fs)
    {
        int type, w, h;
        fs >> type;
        fs >> w >> h;
        if (W < w || W < h) {
            std::cerr << "Too large" << std::endl;
            return false;
        }
        if (type == 1) {
            int x, y;
            fs >> x >> y;
            Position p = { static_cast<uint8_t>(x * 2), static_cast<uint8_t>(y * 2) };
            maze.setGoal(p);
        } else {
            Position p = { 7 * 2, 7 * 2 };
            maze.setGoal(p);
        }
        for (int cursory = h - 1; cursory >= 0; cursory--) {
            for (int cursorx = 0; cursorx < w; cursorx++) {
                char ch;
                uint8_t byte;
                fs >> ch;
                if (ch >= '0' && ch <= '9') {
                    byte = ch - '0';
                } else if (ch >= 'a' && ch <= 'f') {
                    byte = ch + 0xa - 'a';
                } else if (ch == ' ' || ch == '\n' || ch == '\r') {
                    cursorx--;
                    continue;
                } else {
                    std::cerr << "Invalid maze data" << std::endl;
                    return false;
                }

                Position p;
                p.x = 2 * cursorx;
                if (cursory != 0) {
                    p.y = 2 * cursory - 1;
                    maze.setWall(p, byte & 0x4);
                }
                if (cursory != h - 1) {
                    p.y = 2 * cursory + 1;
                    maze.setWall(p, byte & 0x1);
                }
                p.y = 2 * cursory;
                if (cursorx != 0) {
                    p.x = 2 * cursorx - 1;
                    maze.setWall(p, byte & 0x8);
                }
                if (cursorx != w - 1) {
                    p.x = 2 * cursorx + 1;
                    maze.setWall(p, byte & 0x2);
                }
            }
        }
        return true;
    }

    template <uint8_t W>
    bool loadMazeFromStdin(Maze<W>& maze)
    {
        return loadMazeFromStream(maze, std::cin);
    }

    template <uint8_t W>
    bool loadMazeFromFile(Maze<W>& maze, std::string filename)
    {
        std::ifstream fs;
        fs.open(filename);
        if (fs.fail()) {
            return false;
        }
        bool result = loadMazeFromStream(maze, fs);
        fs.close();
        return result;
    }

    template <uint8_t W>
    void loadEmptyMaze(int x, int y, Maze<W>& maze)
    {
        Position p = { static_cast<uint8_t>(x), static_cast<uint8_t>(y) };
        maze.resetData();
        maze.setGoal(p);
    }

    template <uint8_t W>
    void printMaze(const Maze<W>& maze, const int m = 1)
    {
        std::cout << "_";
        for (int i = 0; i < W * m; i++) {
            std::cout << "__";
        }
        std::cout << std::endl;

        for (int cursory = W - 1; cursory >= 0; cursory--) {
            for (int j = 0; j < m; j++) {
                for (int cursorx = 0; cursorx < W; cursorx++) {
                    if (cursorx == 0)
                        std::cout << "|";

                    for (int i = 0; i < m; i++) {
                        if (j == m - 1 && (cursory == 0 || maze.isSetWall({ static_cast<uint8_t>(2 * cursorx), static_cast<uint8_t>(2 * cursory - 1) }))) {
                            std::cout << "\x1b[4m"; // underline
                        }
                        std::cout << " ";
                        std::cout << "\x1b[0m";
                    }

                    for (int i = 0; i < m; i++) {
                        if (i == m - 1 && (cursorx == W - 1 || maze.isSetWall({ static_cast<uint8_t>(2 * cursorx + 1), static_cast<uint8_t>(2 * cursory) }))) {
                            std::cout << "|";
                        } else if (j == m - 1 && (cursory == 0 || maze.isSetWall({ static_cast<uint8_t>(2 * cursorx), static_cast<uint8_t>(2 * cursory - 1) }))) {
                            std::cout << "\x1b[4m";
                            std::cout << " ";
                            std::cout << "\x1b[0m";
                        } else {
                            std::cout << " ";
                        }
                    }
                    if (cursorx == W - 1) {
                        std::cout << std::endl;
                    }
                }
            }
        }
    }
}

std::ostream& operator<<(std::ostream& os, Direction d);
std::ostream& operator<<(std::ostream& os, Coordinates c);

}
