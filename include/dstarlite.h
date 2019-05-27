#pragma once

#include "solver.h"
#include <algorithm>
#include <limits>
#include <vector>
#include <bitset>

namespace Amaze {

template <typename TMazeGraph>
class DStarLite : public Solver<TMazeGraph> {
public:
    using Base = Solver<TMazeGraph>;
    using NodeId = typename TMazeGraph::NodeId;
    using Cost = typename TMazeGraph::Cost;
    static constexpr Cost INF = std::numeric_limits<Cost>::max();

    DStarLite(TMazeGraph& mg)
        : Solver<TMazeGraph>(mg)
        , key_modifier(0)
        , id_start_orig(Base::mg.getCMaze().getWidth() - 1)
        , id_start(id_start_orig)
        , id_goal(0)
        , id_last(id_start)
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
    NodeId getNextNodeId() const
    {
        //TODO: Implement
        return NodeId(0);
    }
    NodeId getCurrentNodeId() const
    {
        //TODO: Implement
        return NodeId(0);
    }
    void preSense()
    {
        //TODO: Implement
        return;
    }
    void postSense()
    {
        //TODO: Implement
        return;
    }
    void reset()
    {
        key_modifier = Cost(0);
        id_start = id_start_orig;
        id_goal = Base::mg.getGoalNodeId();
        id_last = id_start;
        //setNodeRhs(id_goal, 0);
        return;
    }

private:
    Cost key_modifier;

    const NodeId id_start_orig;
    NodeId id_start;
    NodeId id_goal;
    NodeId id_last;

    std::vector<NodeId> open_list;
    std::bitset<10000> in_open_list;
};

}
