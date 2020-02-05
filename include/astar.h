#pragma once

#include "solver.h"
#include <algorithm>
#include <array>
#include <bitset>
//#include <iostream>
#include <set>
#include <vector>

namespace Amaze {

/// \~japanese
/// A* ソルバ
///
/// \~english
/// A* solver
template <typename TCost, typename TNodeId, uint8_t W, TNodeId NodeCount>
class AStar : public Solver<TCost, TNodeId, W, NodeCount> {
public:
    using MazeGraphBase = MazeGraph<TCost, TNodeId, W, NodeCount>;
    using Base = Solver<TCost, TNodeId, W, NodeCount>;
    using NodeId = TNodeId;
    using Cost = TCost;
    using HeapKey = Cost;

    /// \~japanese
    /// ヒープ内のキー値のコンパレータ構造体．
    ///
    /// \~english
    /// Comparator structure for the key value of the heap.
    struct KeyCompare {
        bool operator()(const std::pair<HeapKey, NodeId>& lhs, const std::pair<HeapKey, NodeId>& rhs) const
        {
            if (lhs.first < rhs.first) {
                return true;
            } else if (lhs.first == rhs.first && lhs.second > rhs.second) {
                return true;
            } else {
                return false;
            }
        }
    };

private:
    /// \~japanese 現在のノードのID
    /// \~english Current node ID
    NodeId id_current;
    /// \~japanese 終点ノードのID
    /// \~english Destination node ID
    std::set<NodeId> ids_destination;
    /// \~japanese 一手前のノードのID
    /// \~english Last node ID
    NodeId id_last;
    /// \~japanese つぎに訪れる候補ノードのID
    /// \~english Next candidate node ID
    NodeId id_candidate;

    /// g value
    std::array<Cost, NodeCount> g;

    /// Open list
    std::set<std::pair<HeapKey, NodeId>, KeyCompare> open_list;
    /// \~japanese Open listにノードが入っているかどうかのフラグ
    /// \~english Flags whether nodes are in the open list
    std::bitset<NodeCount> in_open_list;

public:
    using Base::changeDestinations;
    using Base::changeMazeGraph;

    /// \~japanese 無限コストとみなす値
    /// \~english Cost value assumed to be infinity
    static constexpr Cost kInf = MazeGraphBase::kInf;

    explicit AStar(const MazeGraphBase* mg)
        : Solver<TCost, TNodeId, W, NodeCount>(mg)
        , id_current(mg->startNodeId())
        , ids_destination()
        , id_last(id_current)
        , id_candidate(id_current)
        , g()
        , open_list()
        , in_open_list(0)
    {
        ids_destination = mg->goalNodeIds();
        g.fill(kInf);
    }
    /// \~japanese
    /// ヒープ内の要素を更新します．
    ///
    /// \param[in] id ノードID
    /// \param[in] k 新しいキー値
    ///
    /// \~english
    /// Updates an element in the heap.
    ///
    /// \param[in] id Node ID
    /// \param[in] k New key value
    void updateHeap(NodeId id, HeapKey k)
    {
        if (id > NodeCount) {
            return;
        }
        auto it = find_if(open_list.begin(), open_list.end(), [=](auto p) { return p.second == id; });
        if (it == open_list.end() || it->first == k) {
            return;
        }
        open_list.erase(it);
        open_list.insert({ k, id });
    }

    /// \~japanese
    /// 現在のノードから終点ノードまでの最短経路を計算します．
    ///
    /// \~english
    /// Computes the shortest path from the current node to the destination node.
    void computeShortestPath()
    {
        NodeId examined_nodes = 0;
        NodeId max_heap_size = 0;
        while (!open_list.empty()) {
            auto [kold, uid] = *open_list.begin();
            examined_nodes++;
            if (uid == id_current) {
                //std::cout << "The number of examined nodes in this round: " << examined_nodes << std::endl;
                //std::cout << "Maximum size of the open list: " << max_heap_size << std::endl;
                return;
            }
            open_list.erase(open_list.begin());
            in_open_list.reset(uid);
            for (auto& [sid, scost] : Base::mg->neighborEdges(uid)) {
                Cost tentative_g = satSum(g[uid], scost);
                if (tentative_g < g[sid]) {
                    g[sid] = tentative_g;
                    if (in_open_list.test(sid)) {
                        updateHeap(sid, calculateKey(sid));
                    } else {
                        open_list.insert({ calculateKey(sid), sid });
                        in_open_list.set(sid);
                    }
                }
            }
            if (open_list.size() > max_heap_size) {
                max_heap_size = NodeId(open_list.size());
            }
        }
        // cannot reach
    }
    /// \~japanese
    /// ヒープに用いるキー値を計算します．
    ///
    /// \param[in] id ノードID
    ///
    /// \~english
    /// Calculates key value for the heap.
    ///
    /// \param[in] id Node ID
    HeapKey calculateKey(NodeId id) const
    {
        if (id > NodeCount) {
            return kInf;
        }
        // commonly referred to as "fScore"
        return satSum(g[id], Base::mg->distance(id_current, id));
    }
    NodeId nextNodeId() const { return NodeId(0); }
    NodeId currentNodeId() const { return id_current; }
    NodeId lastNodeId() const { return id_last; }
    std::set<NodeId> destinationNodeIds() const { return ids_destination; }

    std::pair<NodeId, Cost> lowestNeighbor(NodeId id) const
    {
        NodeId argmin = id;
        Cost mincost = kInf;
        for (auto& [spid, spcost] : Base::mg->neighborEdges(id)) {
            Cost cost = satSum(spcost, g[spid]);
            if (mincost >= cost) {
                mincost = cost;
                argmin = spid;
            }
        }
        return { argmin, mincost };
    }

    std::vector<AgentState> reconstructPath(NodeId id_from, const std::set<NodeId>& ids_to) const
    {
        std::vector<AgentState> path;
        path.push_back(Base::mg->agentStateByNodeId(id_from));
        NodeId id_current_on_path = id_from;
        NodeId id_last_on_path = id_from;
        while (ids_to.find(id_current_on_path) == ids_to.end()) {
            auto [nid, ncost] = lowestNeighbor(id_current_on_path);
            if (ncost == kInf) {
                return std::vector<AgentState>();
            }
            id_last_on_path = id_current_on_path;
            id_current_on_path = nid;
            path.push_back(Base::mg->agentStateByEdge(id_last_on_path, id_current_on_path));
        }
        return path;
    }

    void preSense(const std::vector<Position>& sense_positions __attribute__((unused)))
    {
        auto ln = lowestNeighbor(id_current);
        id_candidate = ln.first;
    }
    void postSense(const std::vector<Position>& changed_positions __attribute__((unused)))
    {
        if (ids_destination.find(id_current) != ids_destination.end()) {
            // reached the destination
            // TODO: implement
            std::cout << "Reached the destination" << std::endl;
        } else {
            if (Base::mg->edgeCost(id_current, id_candidate) == kInf) {
                //if (open_list.empty()) {
                //    // no route
                //    // TODO: implement
                //    std::cerr << "No route" << std::endl;
                //    return;
                //}
                resetCostsAndLists();
                for (auto id : ids_destination) {
                    g[id] = 0;
                    open_list.insert({ Base::mg->distance(id_current, id), id });
                    in_open_list.set(id);
                }
                computeShortestPath();
            }
            id_last = id_current;
            id_current = lowestNeighbor(id_current).first;
        }
    }

    void resetCostsAndLists()
    {
        g.fill(kInf);

        open_list.clear();
        in_open_list.reset();
    }
    void reset()
    {
        resetCostsAndLists();
        id_current = Base::mg->startNodeId();
        ids_destination = Base::mg->goalNodeIds();
        id_last = id_current;
    }
    void initialize()
    {
        reset();
        for (auto id : ids_destination) {
            g[id] = 0;
            open_list.insert({ Base::mg->distance(id_current, id), id });
            in_open_list.set(id);
        }
        computeShortestPath();
    }
    void changeDestinations(const std::set<NodeId>& ids)
    {
        resetCostsAndLists();
        // id_current = id_current
        ids_destination = ids;
        id_last = id_current;
        for (auto id : ids_destination) {
            g[id] = 0;
            open_list.insert({ Base::mg->distance(id_current, id), id });
            in_open_list.set(id);
        }
        computeShortestPath();
    }
};

} // namespace Amaze
