#include "common.h"
#include "maze.h"

template <uint8_t W>
void senseFourWay(Amaze::Maze<W>& maze, const Amaze::Maze<W>& reference_maze, Amaze::AgentState as, std::vector<Amaze::Position>& changed_positions)
{
    constexpr int8_t dx[4] = { 0, 1, 0, -1 };
    constexpr int8_t dy[4] = { 1, 0, -1, 0 };
    for (int i = 0; i < 4; i++) {
        Amaze::Position p = { uint8_t(as.pos.x + dx[i]), uint8_t(as.pos.y + dy[i]) };
        if (p.x > 2 * W || p.y > 2 * W) {
            continue;
        }
        if (maze.isCheckedWall(p)) {
            continue;
        }
        maze.setCheckedWall(p, true);
        if (reference_maze.isSetWall(p)) {
            maze.setWall(p, true);
            Amaze::Position pto = { uint8_t(p.x + dx[i]), uint8_t(p.y + dy[i]) };
            changed_positions.push_back(pto);
        } else {
            maze.setWall(p, false);
        }
    }
}

template <uint8_t W>
void senseSixWay(Amaze::Maze<W>& maze, const Amaze::Maze<W>& reference_maze, Amaze::AgentState as, std::vector<Amaze::Position>& changed_positions)
{
    constexpr int8_t dx[9] = { 0, 1, 2, 1, 0, -1, -2, -1 };
    constexpr int8_t dy[9] = { 2, 1, 0, -1, -2, -1, 0, 1 };
    for (int i = 0; i < 8; i++) {
        if (as.pos.x % 2 == 0 && as.pos.y % 2 == 1 && (i == 2 || i == 6)) {
            continue;
        } else if (as.pos.x % 2 == 1 && as.pos.y % 2 == 0 && (i == 0 || i == 4)) {
            continue;
        }
        Amaze::Position p = { uint8_t(as.pos.x + dx[i]), uint8_t(as.pos.y + dy[i]) };
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
}
