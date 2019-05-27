#pragma once

#include "mazegraph.h"

namespace Amaze {

template <typename TMazeGraph>
class Solver {
public:
    Solver(TMazeGraph& mg)
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
    virtual typename TMazeGraph::NodeId getNextNodeId() const;
    virtual typename TMazeGraph::NodeId getCurrentNodeId() const;
    virtual void preSense();
    virtual void postSense(const std::vector<Coordinates>& sensed_coordinates);
    virtual void reset();
    virtual void initialize();

protected:
    TMazeGraph& mg;
};

}
