#pragma once

#include "maze.h"
#include <cmath>
#include <limits>

namespace Amaze {

template <typename TNodeId, typename TCost>
class MazeGraph {
public:
    MazeGraph(const Maze& maze)
        : maze(maze)
    {
    }
    virtual bool isConnected(TNodeId id_from, TNodeId id_to) const { return false; }
    virtual TCost distance(TNodeId id_from, TNodeId id_to) const { return TCost(0); }
    virtual TCost edgeCost(TNodeId id_from, TNodeId id_to) const { return TCost(0); }
    virtual TNodeId nodeIdByCoord(Coord c) const { return TNodeId(0); }
    virtual Coord coordByNodeId(TNodeId id) const { return Coord(); }

    unsigned int size;

protected:
    const Maze& maze;
};

template <typename TNodeId, typename TCost>
class FourWayStepMapGraph : public MazeGraph<TNodeId, TCost> {
public:
    using Base = MazeGraph<TNodeId, TCost>;

    FourWayStepMapGraph(const Maze& maze)
        : Base(maze)
    {
        Base::size = maze.getWidth() * maze.getHeight();
    }
    bool isConnected(TNodeId id_from, TNodeId id_to) const
    {
        Coord c1, c2;
        c1 = coordByNodeId(id_from);
        c2 = coordByNodeId(id_to);
        if ((abs((int)c1.x - c2.x) == 1) ^ (abs((int)c1.y - c2.y) == 1)) {
            return true;
        }
        return false;
    }
    TCost edgeCost(TNodeId id_from, TNodeId id_to) const
    {
        uint8_t w = Base::maze.getWidth();
        Coord c = coordByNodeId(id_from);
        if (!isConnected(id_from, id_to)) {
            return std::numeric_limits<TCost>::max();
        }
        int diff = id_to - id_from;
        if (diff == w) {
            c.dir = DirNorth;
        } else if (diff == -w) {
            c.dir = DirSouth;
        } else if (diff == 1) {
            c.dir = DirEast;
        } else if (diff == -1) {
            c.dir = DirWest;
        }
        if (Base::maze.isSetWall(c)) {
            return std::numeric_limits<TCost>::max();
        } else {
            return 1;
        }
    }
    TCost distance(TNodeId id_from, TNodeId id_to) const
    {
        Coord c1, c2;
        c1 = coordByNodeId(id_from);
        c2 = coordByNodeId(id_to);
        return std::max(abs((int)c1.x - c2.x), abs((int)c1.y - c2.y));
    }
    TNodeId nodeIdByCoord(Coord c) const
    {
        uint8_t w = Base::maze.getWidth();
        return TNodeId(c.x + c.y * w);
    }
    Coord coordByNodeId(TNodeId id) const
    {
        uint8_t w = Base::maze.getWidth();
        uint8_t x = id % w;
        uint8_t y = id / w;
        return { x, y, 0 };
    }
};

template <typename TNodeId, typename TCost>
class SixWayWallNodeGraph : public MazeGraph<TNodeId, TCost> {
public:
    using Base = MazeGraph<TNodeId, TCost>;

    SixWayWallNodeGraph(const Maze& maze)
        : Base(maze)
    {
        Base::size = maze.getWidth() * maze.getHeight();
    }
    bool isConnected(TNodeId id_from, TNodeId id_to) const
    {
        Coord c1, c2;
        c1 = coordByNodeId(id_from);
        c2 = coordByNodeId(id_to);
        uint8_t tmp1 = std::max(c1.dir.half, c2.dir.half);
        uint8_t tmp2 = std::min(c1.dir.half, c2.dir.half);
        if (c1.x == c2.x && c1.y == c2.y && tmp1 / tmp2 == 2) {
            return true;
        }
        if (c1.dir.half == DirEast.half) {
            if (c2.dir.half == DirEast.half && abs((int)c1.x - c2.x) == 1 && c1.y == c2.y) {
                return true;
            }
            if (c2.dir.half == DirNorth.half) {
                if (c2.x == c1.x + 1 && c1.y == c2.y) {
                    return true;
                }
                if (c2.x == c1.x + 1 && c1.y == c2.y + 1) {
                    return true;
                }
                if (c2.x == c1.x && c1.y == c2.y + 1) {
                    return true;
                }
            }
        } else {
            if (c2.dir.half == DirNorth.half && abs((int)c1.y - c2.y) == 1 && c1.x == c2.x) {
                return true;
            }
            if (c2.dir.half == DirEast.half) {
                if (c1.x == c2.x + 1 && c2.y == c1.y) {
                    return true;
                }
                if (c1.x == c2.x + 1 && c2.y == c1.y + 1) {
                    return true;
                }
                if (c1.x == c2.x && c2.y == c1.y + 1) {
                    return true;
                }
            }
        }
        return false;
    }
    TCost edgeCost(TNodeId id_from, TNodeId id_to) const
    {
        uint8_t w = Base::maze.getWidth();
        Coord c = coordByNodeId(id_from);
        if (!isConnected(id_from, id_to)) {
            return std::numeric_limits<TCost>::max();
        }
        // TODO: implement
    }
    TCost distance(TNodeId id_from, TNodeId id_to) const
    {
        Coord c1 = coordByNodeId(id_from);
        Coord c2 = coordByNodeId(id_to);
        return TCost(2) * std::max(abs((int)c1.x - c2.x), abs((int)c1.y - c2.y)) + (c1.x == c2.x && c1.y == c2.y && c1.dir.half != c2.dir.half);
    }
    TNodeId nodeIdByCoord(Coord c) const
    {
        uint8_t w = Base::maze.getWidth();
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
        return std::numeric_limits<TNodeId>::max();
    }
    Coord coordByNodeId(TNodeId id) const
    {
        uint8_t w = Base::maze.getWidth();
        Coord c;
        TNodeId tmp = id % (2 * w - 1);
        TNodeId tmp2 = id / (2 * w - 1);
        bool isNorth = tmp > w - 2;
        c.dir = isNorth ? DirNorth : DirEast;
        if (isNorth) {
            c.x = tmp - w + 1;
            c.y = tmp2;
        } else {
            c.x = tmp;
            c.y = tmp2;
        }
        return c;
    }
};

}
