#pragma once

#include "maze.h"
#include <cmath>
#include <limits>
#include <utility>

namespace Amaze {

template <typename TCost = uint8_t, typename TNodeId = uint16_t>
class MazeGraph {
public:
    using Cost = TCost;
    using NodeId = TNodeId;
    MazeGraph(const Maze& maze)
        : maze(maze)
    {
    }
    virtual TCost distance(TNodeId id_from, TNodeId id_to) const;
    virtual std::pair<bool, TCost> getEdge(Coordinates from, Coordinates to) const
    {
        return getEdge(nodeIdByCoordinates(from), nodeIdByCoordinates(to));
    }
    virtual std::pair<bool, TCost> getEdge(TNodeId id_from, TNodeId id_to) const;
    virtual bool edgeExist(TNodeId id_from, TNodeId id_to) const
    {
        std::pair<bool, TCost> e = getEdge(id_from, id_to);
        return e.first;
    }
    virtual TCost edgeCost(TNodeId id_from, TNodeId id_to) const
    {
        std::pair<bool, TCost> e = getEdge(id_from, id_to);
        return e.second;
    }
    virtual TNodeId nodeIdByCoordinates(Coordinates c) const { return TNodeId(0); }
    virtual Coordinates coordByNodeId(TNodeId id) const { return Coordinates(); }
    virtual TNodeId getGoalNodeId() const
    {
        return nodeIdByCoordinates(maze.getGoal());
    }
    const Maze& getCMaze() const
    {
        return maze;
    }

    unsigned int size;

protected:
    const Maze& maze;
};

template <typename TCost = uint8_t, typename TNodeId = uint16_t>
class FourWayStepMapGraph : public MazeGraph<TCost, TNodeId> {
public:
    using Base = MazeGraph<TCost, TNodeId>;

    FourWayStepMapGraph(const Maze& maze)
        : Base(maze)
    {
        Base::size = maze.getWidth() * maze.getHeight();
    }
    TCost distance(TNodeId id_from, TNodeId id_to) const
    {
        Coordinates c1, c2;
        c1 = coordByNodeId(id_from);
        c2 = coordByNodeId(id_to);
        return std::max(abs((int)c1.x - c2.x), abs((int)c1.y - c2.y));
    }
    std::pair<bool, TCost> getEdge(Coordinates from, Coordinates to) const
    {
        return getEdge(nodeIdByCoordinates(from), nodeIdByCoordinates(to));
    }
    std::pair<bool, TCost> getEdge(TNodeId id_from, TNodeId id_to) const
    {
        Coordinates c1, c2;
        c1 = coordByNodeId(id_from);
        c2 = coordByNodeId(id_to);
        TCost maxcost = 0;
        if (Base::maze.isSetWall(c1) || Base::maze.isSetWall(c2)) {
            maxcost = std::numeric_limits<TCost>::max();
        }
        if ((abs((int)c1.x - c2.x) == 1) ^ (abs((int)c1.y - c2.y) == 1)) {
            return { true, std::max(TCost(1), maxcost) };
        }
        return { false, std::numeric_limits<TCost>::max() };
    }
    TNodeId nodeIdByCoordinates(Coordinates c) const
    {
        uint8_t w = Base::maze.getWidth();
        return TNodeId(c.x + c.y * w);
    }
    Coordinates coordByNodeId(TNodeId id) const
    {
        uint8_t w = Base::maze.getWidth();
        uint8_t x = id % w;
        uint8_t y = id / w;
        return { x, y, { 0 } };
    }
};

template <typename TCost = uint8_t, typename TNodeId = uint16_t>
class SixWayWallNodeGraph : public MazeGraph<TCost, TNodeId> {
public:
    using Base = MazeGraph<TCost, TNodeId>;

    SixWayWallNodeGraph(const Maze& maze)
        : Base(maze)
    {
        Base::size = 2 * maze.getWidth() * maze.getHeight() - maze.getWidth() - maze.getHeight();
    }
    TCost distance(TNodeId id_from, TNodeId id_to) const
    {
        Coordinates c1 = coordByNodeId(id_from);
        Coordinates c2 = coordByNodeId(id_to);
        return TCost(2) * std::max(abs((int)c1.x - c2.x), abs((int)c1.y - c2.y)) + (c1.x == c2.x && c1.y == c2.y && c1.dir.half != c2.dir.half);
    }
    std::pair<bool, TCost> getEdge(Coordinates from, Coordinates to) const
    {
        return getEdge(nodeIdByCoordinates(from), nodeIdByCoordinates(to));
    }
    std::pair<bool, TCost> getEdge(TNodeId id_from, TNodeId id_to) const
    {
        Coordinates c1, c2;
        c1 = coordByNodeId(id_from);
        c2 = coordByNodeId(id_to);

        TCost maxcost = 0;
        if (Base::maze.isSetWall(c1) || Base::maze.isSetWall(c2)) {
            maxcost = std::numeric_limits<TCost>::max();
        }

        uint8_t dirmax = std::max(c1.dir.half, c2.dir.half);
        uint8_t dirmin = std::min(c1.dir.half, c2.dir.half);
        if (c1.x == c2.x && c1.y == c2.y && (dirmax / dirmin == 2 || (dirmax == 8 && dirmin == 1))) {
            return { true, std::max(TCost(2), maxcost) };
        }
        if (c1.dir.half == East.half) {
            if (c2.dir.half == East.half && abs((int)c1.x - c2.x) == 1 && c1.y == c2.y) {
                return { true, std::max(TCost(3), maxcost) };
            }
            if (c2.dir.half == North.half) {
                if (c2.x == c1.x + 1 && c1.y == c2.y) {
                    return { true, std::max(TCost(2), maxcost) };
                }
                if (c2.x == c1.x + 1 && c1.y == c2.y + 1) {
                    return { true, std::max(TCost(2), maxcost) };
                }
                if (c2.x == c1.x && c1.y == c2.y + 1) {
                    return { true, std::max(TCost(2), maxcost) };
                }
            }
        } else {
            if (c2.dir.half == North.half && abs((int)c1.y - c2.y) == 1 && c1.x == c2.x) {
                return { true, std::max(TCost(3), maxcost) };
            }
            if (c2.dir.half == East.half) {
                if (c1.x == c2.x + 1 && c2.y == c1.y) {
                    return { true, std::max(TCost(2), maxcost) };
                }
                if (c1.x == c2.x + 1 && c2.y == c1.y + 1) {
                    return { true, std::max(TCost(2), maxcost) };
                }
                if (c1.x == c2.x && c2.y == c1.y + 1) {
                    return { true, std::max(TCost(2), maxcost) };
                }
            }
        }
        return { false, std::numeric_limits<TCost>::max() };
    }
    TNodeId nodeIdByCoordinates(Coordinates c) const
    {
        uint8_t w = Base::maze.getWidth();
        switch (c.dir.half) {
        case North.half:
            return c.y * (2 * w - 1) + c.x + w - 1;
        case East.half:
            return c.y * (2 * w - 1) + c.x;
        case South.half:
            return c.y * (2 * w - 1) + c.x - w;
        case West.half:
            return c.y * (2 * w - 1) + c.x - 1;
        }
        return std::numeric_limits<TNodeId>::max();
    }
    Coordinates coordByNodeId(TNodeId id) const
    {
        uint8_t w = Base::maze.getWidth();
        Coordinates c;
        TNodeId tmp = id % (2 * w - 1);
        TNodeId tmp2 = id / (2 * w - 1);
        bool isNorth = tmp > w - 2;
        c.dir = isNorth ? North : East;
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
