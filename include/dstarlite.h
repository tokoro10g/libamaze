#pragma once

#include "solver.h"
#include <algorithm>
#include <vector>
#include <limits>

namespace Amaze {

template <template <typename, typename> typename TMazeGraph, typename TCost, typename TNodeId>
class DStarLite : public Solver<TMazeGraph, TCost, TNodeId> {
public:
    using Base = Solver<TMazeGraph, TCost, TNodeId>;
    DStarLite(TMazeGraph<TCost, TNodeId>& mg)
        : Solver<TMazeGraph, TCost, TNodeId>(mg)
    {
    }
    void computeShortestPath()
    {
        //int examined_nodes = 0;
        //int max_heap_size = 0;
        //Node* n = mg.getNode(id_start);
        //while (!open_list.empty() && (open_list.front().first < n->calculateKey() || n->rhs > n->g)) {
        //    auto uid = open_list.front().second;
        //    Node* u = mg.getNode(uid);
        //    auto kold = open_list.front().first;
        //    auto knew = u->calculateKey();
        //    examined_nodes++;
        //    if (kold < knew) {
        //        updateHeap(u->id, knew);
        //    } else if (u->g > u->rhs) {
        //        u->g = u->rhs;
        //        pop_heap(open_list.begin(), open_list.end(), KeyCompare());
        //        open_list.pop_back();
        //        in_open_list[u->id]--;
        //        for (auto s : u->neighbors) {
        //            if (s->rhs != 0) {
        //                s->rhs = min(s->rhs, satSum(mg.getEdgeCost(u->id, s->id), u->g));
        //            }
        //            updateVertex(s->id);
        //        }
        //    } else {
        //        Cost gold = u->g;
        //        u->g = numeric_limits<TCost>::max();
        //        auto f = [=](Node* s) {
        //            if (s->rhs == satSum(mg.getEdgeCost(s->id, u->id), gold)) {
        //                if (s->rhs != 0) {
        //                    Cost mincost = numeric_limits<TCost>::max();
        //                    for (auto sp : s->neighbors) {
        //                        mincost = min(mincost, satSum(mg.getEdgeCost(sp->id, s->id), sp->g));
        //                    }
        //                    s->rhs = mincost;
        //                }
        //            }
        //            updateVertex(s->id);
        //        };
        //        for_each(u->neighbors.begin(), u->neighbors.end(), f);
        //        f(u);
        //    }
        //    make_heap(open_list.begin(), open_list.end(), KeyCompare());
        //    if (open_list.size() > max_heap_size) {
        //        max_heap_size = open_list.size();
        //    }
        //}
        //cout << "The number of examined nodes in this round: " << examined_nodes << endl;
        //cout << "Maximum size of the open list: " << max_heap_size << endl;
    }
    TNodeId getNextNodeId() const
    {
    }
    TNodeId getCurrentNodeId() const
    {
    }
    void preSense()
    {
        return;
    }
    void postSense()
    {

    }
    void reset()
    {
    }
};

}
