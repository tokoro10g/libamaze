#pragma once

#include "mazegraph.h"

namespace Amaze {

template <template <typename, typename> typename TMazeGraph, typename TCost, typename TNodeId>
class Solver {
public:
    Solver(TMazeGraph<TCost, TNodeId>& mg)
        : mg(mg)
    {
    }
    virtual Coord getNextLocation() const
    {
        return mg.coordByNodeId(getNextNodeId());
    }
    virtual Coord getCurrentLocation() const
    {
        return mg.coordByNodeId(getCurrentNodeId());
    }
    virtual TNodeId getNextNodeId() const;
    virtual TNodeId getCurrentNodeId() const;
    virtual void preSense();
    virtual void postSense();
    virtual void reset();

protected:
    TMazeGraph<TCost, TNodeId>& mg;
};

}
