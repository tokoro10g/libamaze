#include "common.h"
#include "maze.h"
#include <iostream>

template <uint8_t W>
std::vector<Amaze::Position> sense(Amaze::Maze<W>& maze, const Amaze::Maze<W>& reference_maze, const std::vector<Amaze::Position>& sense_positions)
{
    std::vector<Amaze::Position> changed_positions;
    for (auto p : sense_positions) {
        if (p.x > 2 * W || p.y > 2 * W) {
            continue;
        }
        if (maze.isCheckedWall(p)) {
            continue;
        }
        maze.setCheckedWall(p, true);
        if (reference_maze.isSetWall(p)) {
            maze.setWall(p, true);
            changed_positions.push_back(p);
        } else {
            maze.setWall(p, false);
        }
    }
    return changed_positions;
}
