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
    uint8_t gx, gy;
    Direction gd;

    void setWallInternal(const Coord& c, Direction dir, bool is_checked, bool val);

public:
    static constexpr Direction DirFront = { 0x1 };
    static constexpr Direction DirRight = { 0x2 };
    static constexpr Direction DirBack = { 0x4 };
    static constexpr Direction DirLeft = { 0x8 };
    static constexpr Direction DirNorth = { 0x1 };
    static constexpr Direction DirEast = { 0x2 };
    static constexpr Direction DirSouth = { 0x4 };
    static constexpr Direction DirWest = { 0x8 };

public:
    Maze()
        : w(0)
        , h(0)
        , type(0)
        , gx(0)
        , gy(0)
    {
        gd.half = 0x2;
    }
    Maze(uint8_t _w, uint8_t _h)
        : w(_w)
        , h(_h)
        , type(0)
        , gx(0)
        , gy(0)
    {
        gd.half = 0x2;
    }
    Maze(const Maze& m)
        : data(m.data)
        , w(m.w)
        , h(m.h)
        , type(m.type)
        , gx(m.gx)
        , gy(m.gy)
    {
        gd.half = 0x2;
    }
    ~Maze() { data.clear(); }

    uint8_t getWidth() const { return w; }
    uint8_t getHeight() const { return h; }
    void resize(uint8_t _w, uint8_t _h);

    void setType(uint8_t _type) { type = _type; }
    uint8_t getGoalX() const { return gx; }
    uint8_t getGoalY() const { return gy; }
    Direction getGoalDir() const { return gd; }
    void setGoal(uint8_t _gx, uint8_t _gy)
    {
        gx = _gx;
        gy = _gy;
    }
    void setGoalDir(Direction d) { gd = d; }
    uint16_t getGoalNodeIndex() const
    {
        Coord c;
        c.x = gx;
        c.y = gy;
        c.dir = gd;
        return coordToNodeIndex(c);
    }

    uint16_t coordToNodeIndex(Coord c) const
    {
        switch (c.dir.half) {
        case DirNorth.half:
            return c.y * (2 * w - 1) + c.x + w - 1;
        case DirEast.half:
            return c.y * (2 * w - 1) + c.x;
        case DirSouth.half:
            return c.y * (2 * w - 1) + c.x - w;
        case DirWest.half:
            return c.y * (2 * w - 1) + c.x - 1;
        }
        return -1;
    }

    void setWall(const Coord& c, Direction dir) { setWallInternal(c, dir, false, true); }
    void setCheckedWall(const Coord& c, Direction dir) { setWallInternal(c, dir, true, true); }
    void resetWall(const Coord& c, Direction dir) { setWallInternal(c, dir, false, false); }
    void resetCheckedWall(const Coord& c, Direction dir) { setWallInternal(c, dir, true, false); }
    void toggleWall(const Coord& c, Direction dir)
    {
        if (isSetWall(c, dir)) {
            resetWall(c, dir);
        } else {
            setWall(c, dir);
        }
    }
    void toggleCheckedWall(const Coord& c, Direction dir)
    {
        if (isCheckedWall(c, dir)) {
            resetCheckedWall(c, dir);
        } else {
            setCheckedWall(c, dir);
        }
    }
    void setCell(uint16_t index, CellData cd) { data[index] = cd; }
    CellData getCell(uint16_t index) const { return data[index]; }

    bool isSetWall(const Coord& c, Direction dir) const { return isSetWall(c.y * w + c.x, dir); }
    bool isSetWall(const uint16_t index, Direction dir) const { return data[index].byte & dir.half; }
    bool isCheckedWall(const Coord& c, Direction dir) const { return data[c.y * w + c.x].byte & (dir.half << 4); }
    bool isCheckedCell(const Coord& c) const { return isCheckedCell(c.y * w + c.x); }
    bool isCheckedCell(const uint16_t index) const { return (data[index].byte & 0xF0) == 0xF0; }
};
}
