#pragma once

#include "common.h"
#include <bitset>
#include <vector>

namespace Amaze {

template <uint8_t W = 32>
class Maze {
private:
    using MazeData = std::bitset<(2 * W - 1) * (2 * W - 1)>;
    MazeData maze_data;
    MazeData check_data;
    Position start;
    Position goal;

    void setInternal(Position p, bool is_checked, bool val)
    {
        // TODO: assert
        if (!((p.x % 2) ^ (p.y % 2)) || p.x >= 2 * W - 1 || p.y >= 2 * W - 1) {
            // not a wall
            return;
        }

        uint16_t idx = uint16_t(p.y * (2 * W - 1) + p.x);
        if (is_checked) {
            check_data[idx] = val;
        } else {
            maze_data[idx] = val;
        }
    }
    bool isSetInternal(Position p, bool is_checked) const
    {
        // TODO: assert
        if (!((p.x % 2) ^ (p.y % 2)) || p.x >= 2 * W - 1 || p.y >= 2 * W - 1) {
            // not a wall
            return false;
        }

        uint16_t idx = uint16_t(p.y * (2 * W - 1) + p.x);
        if (is_checked) {
            return check_data[idx];
        } else {
            return maze_data[idx];
        }
    }

public:
    Maze()
        : maze_data()
        , check_data()
        , start({ 0, 0 })
        , goal({ 0, 0 })
    {
    }

    void resetData()
    {
        maze_data.reset();
        check_data.reset();
    }

    uint8_t getWidth() const { return W; }

    Position getStart() const { return start; }
    void setStart(Position p) { start = p; }
    Position getGoal() const { return goal; }
    void setGoal(Position p) { goal = p; }

    void setWall(Position p, bool is_set) { setInternal(p, false, is_set); }
    void setCheckedWall(Position p, bool is_set) { setInternal(p, true, is_set); }
    void toggleWall(Position p)
    {
        setWall(p, !isSetWall(p));
    }
    void toggleCheckedWall(Position p)
    {
        setCheckedWall(p, !isCheckedWall(p));
    }

    bool isSetWall(Position p) const { return isSetInternal(p, false); }
    bool isCheckedWall(Position p) const { return isSetInternal(p, true); }
};

}
