#pragma once

#include "maze.h"
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_set>
#include <utility>

namespace Amaze {

template <typename TCost = uint8_t, typename TNodeId = uint16_t, uint8_t W = 32>
class MazeGraph {
public:
    using Cost = TCost;
    using NodeId = TNodeId;
    MazeGraph(const Maze<W>& maze)
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
        Position p = maze.getStart();
        return nodeIdByCoordinates({ p, NoDirection });
    }
    virtual TNodeId getGoalNodeId() const
    {
        Position p = maze.getGoal();
        return nodeIdByCoordinates({ p, NoDirection });
    }
    const Maze<W>& getCMaze() const
    {
        return maze;
    }

    static constexpr TNodeId size = 0;
    static constexpr TCost INF = std::numeric_limits<TCost>::max();
    static constexpr TNodeId INVALID_NODE = std::numeric_limits<TNodeId>::max();

protected:
    const Maze<W>& maze;
};

template <typename TCost = uint8_t, typename TNodeId = uint16_t, uint8_t W = 32>
class FourWayStepMapGraph : public MazeGraph<TCost, TNodeId, W> {
public:
    using Base = MazeGraph<TCost, TNodeId, W>;

    FourWayStepMapGraph(const Maze<W>& maze)
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
            std::cerr << "Out of bounds!!! (id_from: " << (int)id_from << ", id_to: " << (int)id_to << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return Base::INF;
        }
        Coordinates c1, c2;
        c1 = coordByNodeId(id_from);
        c2 = coordByNodeId(id_to);
        return std::max(abs((int)c1.pos.x - c2.pos.x) / 2, abs((int)c1.pos.y - c2.pos.y) / 2);
    }
    void neighbors(TNodeId id, std::vector<TNodeId>& v) const
    {
        if (id >= size) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! (id: " << (int)id << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return;
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
            std::cerr << "Out of bounds!!! (id_from: " << (int)id_from << ", id_to:" << (int)id_to << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return { false, Base::INF };
        }
        Coordinates c1, c2;
        c1 = coordByNodeId(id_from);
        c2 = coordByNodeId(id_to);
        TCost maxcost = 0;
        if (blocked) {
            maxcost = Base::INF;
        }
        if ((abs((int)c1.pos.x - c2.pos.x) == 2 && abs((int)c1.pos.y - c2.pos.y) == 0) || (abs((int)c1.pos.y - c2.pos.y) == 2 && abs((int)c1.pos.x - c2.pos.x) == 0)) {
            return { true, std::max(TCost(1), maxcost) };
        }
        return { false, Base::INF };
    }
    std::pair<bool, TCost> getEdge(TNodeId id_from, TNodeId id_to) const
    {
        Position p;
        p.x = (id_from % W) * 2;
        p.y = (id_from / W) * 2;
        int diff = (int)id_to - id_from;
        if (diff == W) {
            p.y += 1;
        } else if (diff == -W) {
            p.y -= 1;
        } else if (diff == 1) {
            p.x += 1;
        } else if (diff == -1) {
            p.x -= 1;
        }
        return getEdgeWithHypothesis(id_from, id_to, Base::maze.isSetWall(p));
    }
    std::pair<bool, TCost> getEdge(Coordinates from, Coordinates to) const
    {
        return getEdge(nodeIdByCoordinates(from), nodeIdByCoordinates(to));
    }
    TNodeId nodeIdByCoordinates(Coordinates c) const
    {
        if (c.pos.x % 2 != 0 || c.pos.y % 2 != 0) {
            // wall or pillar
            std::cerr << "Out of bounds!!! (pos: " << (int)c.pos.x << ", " << (int)c.pos.y << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return Base::INVALID_NODE;
        }
        return TNodeId(c.pos.x / 2 + c.pos.y / 2 * W);
    }
    Coordinates coordByNodeId(TNodeId id) const
    {
        if (id >= size) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! (id: " << (int)id << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return { { static_cast<uint8_t>(-1), static_cast<uint8_t>(-1) }, { 0 } };
        }
        uint8_t x = id % W;
        uint8_t y = id / W;
        return { { static_cast<uint8_t>(x * 2), static_cast<uint8_t>(y * 2) }, { 0 } };
    }
    static constexpr TNodeId size = W * W;
};

template <typename TCost = uint16_t, typename TNodeId = uint16_t, uint8_t W = 32>
class SixWayWallNodeGraph : public MazeGraph<TCost, TNodeId, W> {
public:
    using Base = MazeGraph<TCost, TNodeId, W>;

    SixWayWallNodeGraph(const Maze<W>& maze)
        : Base(maze)
    {
    }
    TCost distance(TNodeId id_from, TNodeId id_to) const
    {
        if (id_from >= size || id_to >= size) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! (id_from: " << (int)id_from << ", id_to: " << (int)id_to << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return Base::INF;
        }
        Coordinates c1 = coordByNodeId(id_from);
        Coordinates c2 = coordByNodeId(id_to);
        // FIXME: may contain miscalculaions
        return std::max(abs((int)c1.pos.x - c2.pos.x), abs((int)c1.pos.y - c2.pos.y));
    }
    void neighbors(TNodeId id, std::vector<TNodeId>& v) const
    {
        if (id >= size) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! (id: " << (int)id << __FILE__ << ":" << __LINE__ << std::endl;
            return;
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
            std::cerr << "Out of bounds!!! (id_from: " << (int)id_from << ", id_to: " << (int)id_to << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return { false, Base::INF };
        }
        Coordinates c1, c2;
        c1 = coordByNodeId(id_from);
        c2 = coordByNodeId(id_to);

        TCost maxcost = 0;
        if (blocked) {
            maxcost = Base::INF;
        }

        // FIXME: may contain bugs
        if (abs((int)c1.pos.x - c2.pos.x) == 1 && abs((int)c1.pos.y - c2.pos.y) == 1 && c1.pos.x % 2 != c1.pos.y % 2) {
            return { true, std::max(TCost(2), maxcost) };
        } else if ((abs((int)c1.pos.x - c2.pos.x) == 2 && c1.pos.x % 2 == 1 && c1.pos.y % 2 == 0) || (abs((int)c1.pos.y - c2.pos.y) == 2 && c1.pos.x % 2 == 0 && c1.pos.y % 2 == 1)) {
            return { true, std::max(TCost(3), maxcost) };
        }
        return { false, Base::INF };
    }
    std::pair<bool, TCost> getEdge(TNodeId id_from, TNodeId id_to) const
    {
        Coordinates c1, c2;
        c1 = coordByNodeId(id_from);
        c2 = coordByNodeId(id_to);
        return getEdgeWithHypothesis(id_from, id_to, Base::maze.isSetWall(c1.pos) || Base::maze.isSetWall(c2.pos));
    }
    std::pair<bool, TCost> getEdge(Coordinates from, Coordinates to) const
    {
        return getEdge(nodeIdByCoordinates(from), nodeIdByCoordinates(to));
    }
    TNodeId nodeIdByCoordinates(Coordinates c) const
    {
        if (c.pos.x % 2 == c.pos.y % 2) {
            // cell or pillar
            return Base::INVALID_NODE;
        }
        // FIXME: may contain bugs
        if (c.pos.y % 2 == 0) {
            // East node
            return c.pos.y / 2 * (2 * W - 1) + c.pos.x / 2;
        } else {
            // North node
            return c.pos.y / 2 * (2 * W - 1) + c.pos.x / 2 + W - 1;
        }
    }
    Coordinates coordByNodeId(TNodeId id) const
    {
        // FIXME: may contain bugs
        if (id >= size) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! (id: " << (int)id << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return { { static_cast<uint8_t>(-1), static_cast<uint8_t>(-1) }, { 0 } };
        }
        Coordinates c;
        TNodeId tmp = id % (2 * W - 1);
        TNodeId tmp2 = id / (2 * W - 1);
        bool isNorth = tmp > W - 2;
        if (isNorth) {
            c.pos.x = (tmp - W + 1) * 2;
            c.pos.y = tmp2 * 2 + 1;
        } else {
            c.pos.x = tmp * 2 + 1;
            c.pos.y = tmp2 * 2;
        }
        c.dir = NoDirection;
        return c;
    }
    TNodeId getStartNodeId() const
    {
        Position p = Base::maze.getStart();
        if (p.x % 2 == 0 && p.y % 2 == 0) {
            p.y++;
        }
        return nodeIdByCoordinates({ p, North });
    }
    virtual TNodeId getGoalNodeId() const
    {
        Position p = Base::maze.getGoal();
        if (p.x % 2 == 0 && p.y % 2 == 0) {
            p.y++;
        }
        return nodeIdByCoordinates({ p, North });
    }
    static constexpr TNodeId size = 2 * W * (W - 1);
};

}
