#pragma once

#include "common.h"
#include "maze.h"
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <vector>

namespace Amaze {

namespace Utility {
    template <uint8_t W>
    bool loadMazeFromStream(Maze<W>& maze, std::istream& is)
    {
        std::vector<std::string> char_data;
        std::string line;
        std::getline(is, line);
        char_data.push_back(line);

        if ((line.length() - 1) % 4 != 0) {
            // invalid format
            return false;
        }
        if (line.length() > 4 * W + 1) {
            // too large
            return false;
        }
        uint8_t w = static_cast<uint8_t>((line.length() - 1) / 4);

        for (uint8_t i = 0; i < w * 2; i++) {
            std::string line;
            std::getline(is, line);
            char_data.push_back(line);
        }

        for (int8_t cursory = int8_t(w - 1); cursory >= 0; cursory--) {
            for (uint8_t cursorx = 0; cursorx < w; cursorx++) {
                uint8_t x = cursorx;
                uint8_t y = uint8_t(w - 1 - cursory);

                uint8_t ch_x = uint8_t(cursorx * 4 + 2);
                uint8_t ch_y = uint8_t(cursory * 2 + 1);

                char cell_char = char_data[ch_y][ch_x];
                Position p = { uint8_t(x * 2), uint8_t(y * 2) };
                if (cell_char == 'G') {
                    if (std::find(maze.goals.begin(), maze.goals.end(), p) == maze.goals.end()) {
                        maze.goals.push_back(p);
                        if (y != w - 1 && char_data[ch_y - 2][ch_x] == 'G') {
                            maze.goals.push_back(p + Difference({ 0, 1 }));
                        }
                        if (y != 0 && char_data[ch_y + 2][ch_x] == 'G') {
                            maze.goals.push_back(p + Difference({ 0, -1 }));
                        }
                        if (x != w - 1 && char_data[ch_y][ch_x + 4] == 'G') {
                            maze.goals.push_back(p + Difference({ 1, 0 }));
                        }
                        if (x != 0 && char_data[ch_y][ch_x - 4] == 'G') {
                            maze.goals.push_back(p + Difference({ -1, 0 }));
                        }
                    }
                } else if (cell_char == 'S') {
                    maze.start = p;
                }

                if (y != w - 1) {
                    // north wall
                    maze.setWall(p + Difference({ 0, 1 }), char_data[cursory * 2][cursorx * 4 + 2] == '-');
                }
                if (y != 0) {
                    // south wall
                    maze.setWall(p + Difference({ 0, -1 }), char_data[cursory * 2 + 2][cursorx * 4 + 2] == '-');
                }
                if (x != w - 1) {
                    // east wall
                    maze.setWall(p + Difference({ 1, 0 }), char_data[cursory * 2 + 1][cursorx * 4 + 4] == '|');
                }
                if (x != 0) {
                    // west wall
                    maze.setWall(p + Difference({ -1, 0 }), char_data[cursory * 2 + 1][cursorx * 4] == '|');
                }
            }
        }
        return true;
    }

    template <uint8_t W>
    bool loadMazeFromStdin(Maze<W>& maze) { return loadMazeFromStream(maze, std::cin); }

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
        maze.resetData();
        maze.goals.clear();
        maze.goals.push_back({ uint8_t(x), uint8_t(y) });
    }
    template <uint8_t W>
    void loadEmptyMaze(Maze<W>& maze)
    {
        maze.resetData();
        maze.goals.clear();
    }

    template <uint8_t W>
    void printMaze(const Maze<W>& maze, const int m = 1)
    {
        std::set<std::pair<int, int>> goal_cells;

        for (auto p : maze.goals) {
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
                        } else if (j == m / 2 && i == m / 2 && maze.start.x / 2 == cursorx && maze.start.y / 2 == cursory) {
                            std::cout << "S";
                        } else {
                            std::cout << " ";
                        }
                        std::cout << "\x1b[0m"; // end of underline
                    }

                    for (int i = 0; i < m; i++) {
                        if (i == m - 1 && (cursorx == W - 1 || maze.isSetWall({ uint8_t(2 * cursorx + 1), uint8_t(2 * cursory) }))) {
                            std::cout << "|";
                        } else if (j == m - 1 && (cursory == 0 || maze.isSetWall({ uint8_t(2 * cursorx), uint8_t(2 * cursory - 1) }))) {
                            std::cout << "\x1b[4m"; // underline
                            std::cout << " ";
                            std::cout << "\x1b[0m"; // end of underline
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

} // namespace Amaze
