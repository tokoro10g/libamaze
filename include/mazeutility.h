#pragma once

#include "maze.h"
#include <cassert>
#include <fstream>
#include <iostream>
#include <set>
#include <string>

namespace Amaze {

namespace Utility {
    template <uint8_t W>
    bool loadMazeFromStream(Maze<W>& maze, std::istream& is)
    {
        int type, w, h;
        is >> type;
        is >> w >> h;
        if (W < w || W < h) {
            std::cerr << "Too large" << std::endl;
            return false;
        }
        if (type == 1) {
            int x, y;
            is >> x >> y;
            const int goal_width = 2;
            for (int i = 0; i < goal_width * 2 + 1; i++) {
                for (int j = 0; j < goal_width * 2 + 1; j++) {
                    maze.addGoal({ uint8_t(x * 2 - 1 + i), uint8_t(y * 2 - 1 + j) });
                }
            }
        } else {
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 5; j++) {
                    maze.addGoal({ uint8_t(13 + i), uint8_t(13 + j) });
                }
            }
        }
        for (int cursory = h - 1; cursory >= 0; cursory--) {
            for (int cursorx = 0; cursorx < w; cursorx++) {
                char ch;
                uint8_t byte;
                is >> ch;
                if (ch >= '0' && ch <= '9') {
                    byte = uint8_t(ch - '0');
                } else if (ch >= 'a' && ch <= 'f') {
                    byte = uint8_t(ch + 0xa - 'a');
                } else if (ch == ' ' || ch == '\n' || ch == '\r') {
                    cursorx--;
                    continue;
                } else {
                    std::cerr << "Invalid maze data" << std::endl;
                    return false;
                }

                Position p;
                p.x = uint8_t(2 * cursorx);
                if (cursory != 0) {
                    p.y = uint8_t(2 * cursory - 1);
                    maze.setWall(p, byte & 0x4);
                }
                if (cursory != W - 1) {
                    p.y = uint8_t(2 * cursory + 1);
                    maze.setWall(p, byte & 0x1);
                }
                p.y = uint8_t(2 * cursory);
                if (cursorx != 0) {
                    p.x = uint8_t(2 * cursorx - 1);
                    maze.setWall(p, byte & 0x8);
                }
                if (cursorx != W - 1) {
                    p.x = uint8_t(2 * cursorx + 1);
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
        Position p = { uint8_t(x), uint8_t(y) };
        maze.resetData();
        maze.clearGoals();
        maze.addGoal(p);
    }
    template <uint8_t W>
    void loadEmptyMaze(Maze<W>& maze)
    {
        maze.resetData();
        maze.clearGoals();
    }

    template <uint8_t W>
    void printMaze(const Maze<W>& maze, const int m = 1)
    {
        std::vector<Position> goals;
        maze.getGoals(goals);
        std::set<std::pair<int, int>> goal_cells;

        for (auto p : goals) {
            if (p.x % 2 == 0 && p.y % 2 == 0) {
                goal_cells.insert({ p.x / 2, p.y / 2 });
            }
        }

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
                        if (j == m - 1 && (cursory == 0 || maze.isSetWall({ uint8_t(2 * cursorx), uint8_t(2 * cursory - 1) }))) {
                            std::cout << "\x1b[4m"; // underline
                        }
                        if (j == m / 2 && i == m / 2 && goal_cells.find({ cursorx, cursory }) != goal_cells.end()) {
                            std::cout << "G";
                        } else if (j == m / 2 && i == m / 2 && maze.getStart().x / 2 == cursorx && maze.getStart().y / 2 == cursory) {
                            std::cout << "S";
                        } else {
                            std::cout << " ";
                        }
                        std::cout << "\x1b[0m";
                    }

                    for (int i = 0; i < m; i++) {
                        if (i == m - 1 && (cursorx == W - 1 || maze.isSetWall({ uint8_t(2 * cursorx + 1), uint8_t(2 * cursory) }))) {
                            std::cout << "|";
                        } else if (j == m - 1 && (cursory == 0 || maze.isSetWall({ uint8_t(2 * cursorx), uint8_t(2 * cursory - 1) }))) {
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

}
