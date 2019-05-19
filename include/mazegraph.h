#pragma once

#include "maze.h"

namespace Amaze {

template <typename TNodeId, typename TCost>
class MazeGraph {
public:
    class Node {
    public:
        Node(TNodeId id)
            : id(id)
        {
        }
        virtual ~Node() {};
        virtual void predecessors(std::vector<TNodeId>& v) const {}
        virtual void successors(std::vector<TNodeId>& v) const {}
        virtual TCost distanceTo(TNodeId id_to) const { return TCost(0); }

        TNodeId id;
    };

    MazeGraph(const Maze& maze)
        : maze(maze)
    {
    }
    virtual bool isConnected(TNodeId id_from, TNodeId id_to) const { return false; }
    virtual TCost distance(TNodeId id_from, TNodeId id_to) const { return TCost(0); }
    virtual Node queryNodeByCoord(Coord c) const { return Node(0); }
    virtual TNodeId queryNodeIdByCoord(Coord c) const { return TNodeId(0); }

    unsigned int size;

protected:
    const Maze& maze;
};

class GridMazeGraph : public MazeGraph<uint16_t, uint8_t> {
public:
    GridMazeGraph(const Maze& maze)
        : MazeGraph(maze)
    {
        size = maze.getWidth() * maze.getHeight();
    }

private:
    /* data */
};

}
