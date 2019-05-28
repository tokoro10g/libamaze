#pragma once

#include "maze.h"
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_set>
#include <utility>

namespace Amaze {

template <typename TCost = uint8_t, typename TNodeId = uint16_t, int W = 32>
class MazeGraph {
public:
    using Cost = TCost;
    using NodeId = TNodeId;
    MazeGraph(const Maze& maze)
        : maze(maze)
    {
    }
    virtual TCost distance(TNodeId id_from, TNodeId id_to) const = 0;
    virtual void neighbors(TNodeId id, std::vector<TNodeId>& v) const = 0;
    virtual void affectedEdges(const std::vector<Coordinates>& coordinates, std::vector<std::pair<TNodeId, TNodeId>>& edges) const
    {
        std::unordered_set<TNodeId> visited;
        for (Coordinates c : coordinates) {
            TNodeId id = nodeIdByCoordinates(c);
            if (visited.find(id) == visited.end()) {
                visited.insert(id);
                std::vector<NodeId> v;
                neighbors(nodeIdByCoordinates(c), v);
                for (auto n : v) {
                    edges.push_back({ id, n });
                }
            }
        }
    }
    std::pair<bool, TCost> getEdgeWithHypothesis(TNodeId id_from, TNodeId id_to, bool blocked) const;
    virtual std::pair<bool, TCost> getEdge(TNodeId id_from, TNodeId id_to) const = 0;
    virtual std::pair<bool, TCost> getEdge(Coordinates from, Coordinates to) const
    {
        return getEdge(nodeIdByCoordinates(from), nodeIdByCoordinates(to));
    }
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
    virtual TNodeId getStartNodeId() const
    {
        return nodeIdByCoordinates(maze.getStart());
    }
    virtual TNodeId getGoalNodeId() const
    {
        return nodeIdByCoordinates(maze.getGoal());
    }
    const Maze& getCMaze() const
    {
        return maze;
    }

    static constexpr TNodeId size = 0;
    static constexpr TCost INF = std::numeric_limits<TCost>::max();

protected:
    const Maze& maze;
};

template <typename TCost = uint8_t, typename TNodeId = uint16_t, int W = 32>
class FourWayStepMapGraph : public MazeGraph<TCost, TNodeId, W> {
public:
    using Base = MazeGraph<TCost, TNodeId, W>;

    FourWayStepMapGraph(const Maze& maze)
        : Base(maze)
    {
    }
    constexpr TNodeId maxNodeCount(uint8_t wh)
    {
        return wh * wh;
    }
    TCost distance(TNodeId id_from, TNodeId id_to) const
    {
        if (id_from >= size || id_to >= size) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! " << __FILE__ << ":" << __LINE__ << std::endl;
        }
        Coordinates c1, c2;
        c1 = coordByNodeId(id_from);
        c2 = coordByNodeId(id_to);
        return std::max(abs((int)c1.x - c2.x), abs((int)c1.y - c2.y));
    }
    void neighbors(TNodeId id, std::vector<TNodeId>& v) const
    {
        if (id >= size) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! " << __FILE__ << ":" << __LINE__ << std::endl;
        }
        std::array<int8_t, 4> diff = { -1, 1, W, -W };
        for (auto d : diff) {
            if (id + d >= 0 && id + d < size && Base::edgeExist(id, id + d)) {
                v.push_back(id + d);
            }
        }
    }
    std::pair<bool, TCost> getEdgeWithHypothesis(TNodeId id_from, TNodeId id_to, bool blocked) const
    {
        if (id_from >= size || id_to >= size) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! " << id_from << ", " << id_to << " " << __FILE__ << ":" << __LINE__ << std::endl;
        }
        Coordinates c1, c2;
        c1 = coordByNodeId(id_from);
        c2 = coordByNodeId(id_to);
        TCost maxcost = 0;
        if (blocked) {
            maxcost = Base::INF;
        }
        if ((abs((int)c1.x - c2.x) == 1) ^ (abs((int)c1.y - c2.y) == 1)) {
            return { true, std::max(TCost(1), maxcost) };
        }
        return { false, Base::INF };
    }
    std::pair<bool, TCost> getEdge(TNodeId id_from, TNodeId id_to) const
    {
        Coordinates c;
        c.x = id_from % W;
        c.y = id_from / W;
        int diff = (int)id_to - id_from;
        if (diff == W) {
            c.dir = North;
        } else if (diff == -W) {
            c.dir = South;
        } else if (diff == 1) {
            c.dir = East;
        } else if (diff == -1) {
            c.dir = West;
        }
        return getEdgeWithHypothesis(id_from, id_to, Base::maze.isSetWall(c));
    }
    std::pair<bool, TCost> getEdge(Coordinates from, Coordinates to) const
    {
        return getEdge(nodeIdByCoordinates(from), nodeIdByCoordinates(to));
    }
    TNodeId nodeIdByCoordinates(Coordinates c) const
    {
        return TNodeId(c.x + c.y * W);
    }
    Coordinates coordByNodeId(TNodeId id) const
    {
        if (id >= size) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! " << __FILE__ << ":" << __LINE__ << std::endl;
        }
        uint8_t x = id % W;
        uint8_t y = id / W;
        return { x, y, { 0 } };
    }
    static constexpr TNodeId size = W * W;
};

template <typename TCost = uint16_t, typename TNodeId = uint16_t, int W = 32>
class SixWayWallNodeGraph : public MazeGraph<TCost, TNodeId, W> {
public:
    using Base = MazeGraph<TCost, TNodeId, W>;

    SixWayWallNodeGraph(const Maze& maze)
        : Base(maze)
    {
    }
    TCost distance(TNodeId id_from, TNodeId id_to) const
    {
        if (id_from >= size || id_to >= size) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! " << __FILE__ << ":" << __LINE__ << std::endl;
        }
        Coordinates c1 = coordByNodeId(id_from);
        Coordinates c2 = coordByNodeId(id_to);
        return TCost(2) * std::max(abs((int)c1.x - c2.x), abs((int)c1.y - c2.y)) + (c1.x == c2.x && c1.y == c2.y && c1.dir.half != c2.dir.half);
    }
    void neighbors(TNodeId id, std::vector<TNodeId>& v) const
    {
        if (id >= size) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! " << __FILE__ << ":" << __LINE__ << std::endl;
        }
        std::array<int8_t, 8> diff = { -1, 1, W, -W, W - 1, -W + 1, 2 * W - 1, -2 * W + 1 };
        for (auto d : diff) {
            if (id + d >= 0 && id + d < size && Base::edgeExist(id, id + d)) {
                v.push_back(id + d);
            }
        }
    }
    std::pair<bool, TCost> getEdgeWithHypothesis(TNodeId id_from, TNodeId id_to, bool blocked) const
    {
        if (id_from >= size || id_to >= size) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! " << __FILE__ << ":" << __LINE__ << std::endl;
        }
        Coordinates c1, c2;
        c1 = coordByNodeId(id_from);
        c2 = coordByNodeId(id_to);

        TCost maxcost = 0;
        if (blocked) {
            maxcost = Base::INF;
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
        return { false, Base::INF };
    }
    std::pair<bool, TCost> getEdge(TNodeId id_from, TNodeId id_to) const
    {
        Coordinates c1, c2;
        c1 = coordByNodeId(id_from);
        c2 = coordByNodeId(id_to);
        return getEdgeWithHypothesis(id_from, id_to, Base::maze.isSetWall(c1) || Base::maze.isSetWall(c2));
    }
    std::pair<bool, TCost> getEdge(Coordinates from, Coordinates to) const
    {
        return getEdge(nodeIdByCoordinates(from), nodeIdByCoordinates(to));
    }
    TNodeId nodeIdByCoordinates(Coordinates c) const
    {
        switch (c.dir.half) {
        case North.half:
            return c.y * (2 * W - 1) + c.x + W - 1;
        case East.half:
            return c.y * (2 * W - 1) + c.x;
        case South.half:
            return c.y * (2 * W - 1) + c.x - W;
        case West.half:
            return c.y * (2 * W - 1) + c.x - 1;
        }
        return Base::INF;
    }
    Coordinates coordByNodeId(TNodeId id) const
    {
        if (id >= size) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! " << __FILE__ << ":" << __LINE__ << std::endl;
        }
        Coordinates c;
        TNodeId tmp = id % (2 * W - 1);
        TNodeId tmp2 = id / (2 * W - 1);
        bool isNorth = tmp > W - 2;
        c.dir = isNorth ? North : East;
        if (isNorth) {
            c.x = tmp - W + 1;
            c.y = tmp2;
        } else {
            c.x = tmp;
            c.y = tmp2;
        }
        return c;
    }
    static constexpr TNodeId size = 2 * W * (W - 1);
};

}
