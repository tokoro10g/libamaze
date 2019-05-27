#pragma once

#include "common.h"
#include <vector>

namespace Amaze {

union __attribute__((__packed__)) CellData {
    uint8_t byte : 8;
    struct __attribute__((__packed__)) {
        unsigned NORTH : 1;
        unsigned EAST : 1;
        unsigned SOUTH : 1;
        unsigned WEST : 1;
        unsigned CHECKED_NORTH : 1;
        unsigned CHECKED_EAST : 1;
        unsigned CHECKED_SOUTH : 1;
        unsigned CHECKED_WEST : 1;
    } bits;
};

typedef std::vector<CellData> MazeData;

class Maze {
private:
    MazeData data;
    uint8_t w;
    uint8_t h;
    uint8_t type;
    Coordinates goal;

    void setWallInternal(Coordinates c, bool is_checked, bool val)
    {
        data[c.y * w + c.x].byte |= (c.dir.half << (4 * is_checked));

        if (c.dir.bits.SOUTH && c.y != 0) {
            data[(c.y - 1) * w + c.x].bits.NORTH = val;
        }
        if (c.dir.bits.EAST && c.x != w - 1) {
            data[c.y * w + c.x + 1].bits.WEST = val;
        }
        if (c.dir.bits.NORTH && c.y != w - 1) {
            data[(c.y + 1) * w + c.x].bits.SOUTH = val;
        }
        if (c.dir.bits.WEST && c.x != 0) {
            data[c.y * w + c.x - 1].bits.EAST = val;
        }
    }

public:
    Maze()
        : w(0)
        , h(0)
        , type(0)
        , goal({ 0, 0, East })
    {
    }
    Maze(uint8_t _w)
        : w(_w)
        , h(_w)
        , type(0)
        , goal({ 0, 0, East })
    {
        data.resize(w * h);
    }
    Maze(uint8_t _w, uint8_t _h)
        : w(_w)
        , h(_h)
        , type(0)
        , goal({ 0, 0, East })
    {
        data.resize(w * h);
    }
    ~Maze() { data.clear(); }

    uint8_t getWidth() const { return w; }
    uint8_t getHeight() const { return h; }
    void resize(uint8_t _w, uint8_t _h)
    {
        w = _w;
        h = _h;
        data.resize(w * h);
    }

    void setType(uint8_t _type) { type = _type; }
    Coordinates getGoal() const { return goal; }
    void setGoal(Coordinates c) { goal = c; }

    void setWall(Coordinates c) { setWallInternal(c, false, true); }
    void setCheckedWall(Coordinates c) { setWallInternal(c, true, true); }
    void resetWall(Coordinates c) { setWallInternal(c, false, false); }
    void resetCheckedWall(Coordinates c) { setWallInternal(c, true, false); }
    void toggleWall(Coordinates c)
    {
        if (isSetWall(c)) {
            resetWall(c);
        } else {
            setWall(c);
        }
    }
    void toggleCheckedWall(Coordinates c)
    {
        if (isCheckedWall(c)) {
            resetCheckedWall(c);
        } else {
            setCheckedWall(c);
        }
    }
    void setCell(uint16_t index, CellData cd) { data[index] = cd; }
    CellData getCell(uint16_t index) const { return data[index]; }

    bool isSetWall(Coordinates c) const { return isSetWall(c.y * w + c.x, c.dir); }
    bool isSetWall(uint16_t index, Direction dir) const { return data[index].byte & dir.half; }
    bool isCheckedWall(Coordinates c) const { return isCheckedWall(c.y * w + c.x, c.dir); }
    bool isCheckedWall(uint16_t index, Direction dir) const { return data[index].byte & (dir.half << 4); }
    bool isCheckedCell(Coordinates c) const { return isCheckedCell(c.y * w + c.x); }
    bool isCheckedCell(uint16_t index) const { return (data[index].byte & 0xF0) == 0xF0; }
};

}
