#pragma once

#include "mazegraph.h"

namespace Amaze {

template <typename TMazeGraph>
class Solver {
public:
    Solver(const TMazeGraph& mg)
        : mg(mg)
    {
    }
    virtual Coordinates getNextCoordinates() const
    {
        return mg.coordByNodeId(getNextNodeId());
    }
    virtual Coordinates getCurrentCoordinates() const
    {
        return mg.coordByNodeId(getCurrentNodeId());
    }
    virtual typename TMazeGraph::NodeId getNextNodeId() const = 0;
    virtual typename TMazeGraph::NodeId getCurrentNodeId() const = 0;
    virtual void preSense() = 0;
    virtual void postSense(const std::vector<Coordinates>& sensed_coordinates) = 0;
    virtual void reset() = 0;
    virtual void initialize() = 0;

protected:
    const TMazeGraph& mg;
};

}
