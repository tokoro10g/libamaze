#pragma once

#include "solver.h"
#include <algorithm>
#include <array>
#include <bitset>
#include <iostream>
#include <limits>
#include <set>
#include <vector>

namespace Amaze {

/// \~japanese
/// D* Liteソルバ
///
/// 実装は論文(http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf)をもとにしています．
///
/// \tparam TMazeGraph MazeGraphを実装した型
///
/// \~english
/// D* Lite solver
///
/// Implementation is based on the paper (http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf).
///
/// \tparam TMazeGraph Class implements MazeGraph
///
/// \~
/// Sven Koenig and Maxim Likhachev, "D* Lite", 2002.
template <typename TMazeGraph>
class DStarLite : public Solver<TMazeGraph> {
public:
    using Base = Solver<TMazeGraph>;
    using NodeId = typename Base::NodeId;
    using Cost = typename Base::Cost;
    using HeapKey = std::pair<Cost, Cost>;

    /// \~japanese
    /// ヒープ内のキー値のコンパレータ構造体．
    ///
    /// \~english
    /// Comparator structure for the key value of the heap.
    struct KeyCompare {
        bool operator()(const std::pair<HeapKey, NodeId>& a, const std::pair<HeapKey, NodeId>& b) const
        {
            return a.first < b.first;
        }
    };

private:
    /// \~japanese 現在のノードのID
    /// \~english Current node ID
    NodeId id_current;
    /// \~japanese 終点ノードのID
    /// \~english Destination node ID
    std::vector<NodeId> ids_destination;
    /// \~japanese 一手前のノードのID
    /// \~english Last node ID
    NodeId id_last;

    /// Key modifier
    Cost key_modifier;
    /// g value
    std::array<Cost, TMazeGraph::kSize> g;
    /// rhs value
    std::array<Cost, TMazeGraph::kSize> rhs;

    /// Open list
    std::multiset<std::pair<HeapKey, NodeId>, KeyCompare> open_list;
    /// \~japanese Open listにノードが入っているかどうかのフラグ
    /// \~english Flags whether nodes are in the open list
    std::bitset<TMazeGraph::kSize> in_open_list;

public:
    /// \~japanese 無限コストとみなす値
    /// \~english Cost value assumed to be infinity
    static constexpr Cost kInf = TMazeGraph::kInf;

    DStarLite(const TMazeGraph& mg)
        : Solver<TMazeGraph>(mg)
        , id_current(mg.getStartNodeId())
        , ids_destination()
        , id_last(id_current)
        , key_modifier(0)
        , g()
        , rhs()
        , open_list()
        , in_open_list(0)
    {
        mg.getGoalNodeIds(ids_destination);
        g.fill(kInf);
        rhs.fill(kInf);
    }
    ~DStarLite() {}
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
        if (id > TMazeGraph::kSize) {
            return;
        }
        for (auto it = open_list.begin(); it != open_list.end(); it++) {
            std::pair<HeapKey, NodeId> p = *it;
            if (p.second == id) {
                open_list.erase(it);
                p.first = k;
                open_list.insert(p);
                return;
            }
        }
    }
    /// \~japanese
    /// ノードに関するデータを更新します．
    ///
    /// \p g の値が \p rhs の値と異なる場合は変更/追加を行い，等しい場合にはヒープから削除します．
    ///
    /// \param[in] id ノードID
    ///
    /// \~english
    /// Updates node data.
    ///
    /// If the value of \p g is different from \p rhs, then the element is updated or inserted. If they are equal, the element is erased from the heap.
    ///
    /// \param[in] id Node ID
    void updateNode(NodeId id)
    {
        if (id > TMazeGraph::kSize) {
            return;
        }
        if (g[id] != rhs[id] && in_open_list[id]) {
            updateHeap(id, calculateKey(id));
        } else if (g[id] != rhs[id] && !in_open_list[id]) {
            open_list.insert({ calculateKey(id), id });
            in_open_list.set(id);
        } else if (g[id] == rhs[id] && in_open_list[id]) {
            auto it = find_if(open_list.begin(), open_list.end(), [=](auto p) { return p.second == id; });
            if (it == open_list.end()) {
                // should not reach here
                return;
            }
            open_list.erase(it);
            in_open_list.reset(id);
        }
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
        while (!open_list.empty() && (open_list.begin()->first < calculateKey(id_current) || rhs[id_current] > g[id_current])) {
            auto uid = open_list.begin()->second;
            auto kold = open_list.begin()->first;
            auto knew = calculateKey(uid);
            //std::cout<<(int)kold.first<<","<<(int)kold.second<<" "<<(int)knew.first<<","<<(int)knew.second<<std::endl;
            //std::cout<<uid<<": "<<(int)g[uid]<<","<<(int)rhs[uid]<<std::endl;
            examined_nodes++;
            if (kold < knew) {
                updateHeap(uid, knew);
            } else if (g[uid] > rhs[uid]) {
                g[uid] = rhs[uid];
                open_list.erase(open_list.begin());
                if (!in_open_list.test(uid)) {
                    // this node is counted twice
                    continue;
                }
                in_open_list.reset(uid);

                std::vector<NodeId> v;
                Base::mg.neighbors(uid, v);
                for (auto sid : v) {
                    if (rhs[sid] != 0) {
                        rhs[sid] = std::min(rhs[sid], satSum(Base::mg.edgeCost(uid, sid), g[uid]));
                    }
                    updateNode(sid);
                }
            } else {
                Cost gold = g[uid];
                g[uid] = kInf;
                auto f = [=](NodeId sid) {
                    if (rhs[sid] == satSum(Base::mg.edgeCost(sid, uid), gold)) {
                        if (rhs[sid] != 0) {
                            Cost mincost = kInf;
                            std::vector<NodeId> v;
                            Base::mg.neighbors(sid, v);
                            for (auto spid : v) {
                                mincost = std::min(mincost, satSum(Base::mg.edgeCost(spid, sid), g[spid]));
                            }
                            rhs[sid] = mincost;
                        }
                    }
                    updateNode(sid);
                };
                std::vector<NodeId> v;
                Base::mg.neighbors(uid, v);
                for_each(v.begin(), v.end(), f);
                f(uid);
            }
            if (open_list.size() > max_heap_size) {
                max_heap_size = NodeId(open_list.size());
            }
        }
        //std::cout << "The number of examined nodes in this round: " << examined_nodes << std::endl;
        //std::cout << "Maximum size of the open list: " << max_heap_size << std::endl;
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
        if (id > TMazeGraph::kSize) {
            return { kInf, kInf };
        }
        return { satSum(satSum(std::min(g[id], rhs[id]), Base::mg.distance(id_current, id)), key_modifier), std::min(g[id], rhs[id]) };
    }
    NodeId getNextNodeId() const
    {
        //TODO: Implement
        return NodeId(0);
    }
    NodeId getCurrentNodeId() const
    {
        return id_current;
    }
    void getDestinationNodeIds(std::vector<NodeId>& ids) const
    {
        ids.insert(ids.end(), ids_destination.begin(), ids_destination.end());
    }

    std::pair<NodeId, Cost> lowestNeighbor(NodeId id) const
    {
        NodeId argmin = id;
        Cost mincost = kInf;
        std::vector<NodeId> v;
        Base::mg.neighbors(id, v);
        for (auto spid : v) {
            Cost cost = satSum(Base::mg.edgeCost(id, spid), g[spid]);
            if (mincost >= cost) {
                mincost = cost;
                argmin = spid;
            }
        }
        return { argmin, mincost };
    }

    bool reconstructPath(NodeId id_from, const std::vector<NodeId>& ids_to, std::vector<AgentState>& path) const
    {
        while (std::find(ids_to.begin(), ids_to.end(), id_from) == ids_to.end()) {
            path.push_back(Base::mg.agentStateByNodeId(id_from));
            auto n = lowestNeighbor(id_from);
            if (n.second == kInf) {
                return false;
            }
            id_from = n.first;
        }
        path.push_back(Base::mg.agentStateByNodeId(id_from));
        return true;
    }
    bool reconstructPath(NodeId id_from, const std::vector<NodeId>& ids_to, std::vector<NodeId>& path) const
    {
        while (std::find(ids_to.begin(), ids_to.end(), id_from) == ids_to.end()) {
            path.push_back(id_from);
            auto n = lowestNeighbor(id_from);
            if (n.second == kInf) {
                return false;
            }
            id_from = n.first;
        }
        path.push_back(id_from);
        return true;
    }

    void preSense() {}
    void postSense(const std::vector<Position>& changed_positions)
    {
        //TODO: Implement
        if (rhs[id_current] == 0) {
            // reached the destination
            // TODO: implement
            std::cout << "Reached the destination" << std::endl;
        } else {
            if (rhs[id_current] == kInf) {
                // no route
                // TODO: implement
                std::cerr << "No route" << std::endl;
                return;
            }

            if (!changed_positions.empty()) {
                key_modifier = satSum(key_modifier, Base::mg.distance(id_last, id_current));
                id_last = id_current;
                std::vector<std::pair<NodeId, NodeId>> changed_edges;
                Base::mg.affectedEdges(changed_positions, changed_edges);
                for (auto e : changed_edges) {
                    Cost cold;
                    if (Base::mg.edgeCost(e.first, e.second) == kInf) {
                        // assume that the path is NOT blocked in the previous step
                        cold = Base::mg.getEdgeWithHypothesis(e.first, e.second, false).second;
                    } else {
                        // assume that the path IS blocked in the previous step
                        cold = Base::mg.getEdgeWithHypothesis(e.first, e.second, true).second;
                    }
                    auto uid = e.first;
                    auto vid = e.second;
                    if (cold > kInf) {
                        // TODO: probably unnecessary
                        if (rhs[uid] != 0) {
                            rhs[uid] = std::min(rhs[uid], satSum(kInf, g[vid]));
                        }
                    } else if (rhs[uid] == satSum(cold, g[vid])) {
                        if (rhs[uid] != 0) {
                            Cost mincost = kInf;
                            std::vector<NodeId> v;
                            Base::mg.neighbors(uid, v);
                            for (auto spid : v) {
                                mincost = std::min(mincost, satSum(Base::mg.edgeCost(spid, uid), g[spid]));
                            }
                            rhs[uid] = mincost;
                        }
                    }
                    updateNode(uid);
                    if (cold > kInf) {
                        // TODO: probably unnecessary
                        if (rhs[vid] != 0) {
                            rhs[vid] = std::min(rhs[vid], satSum(kInf, g[uid]));
                        }
                    } else if (rhs[vid] == satSum(cold, g[uid])) {
                        if (rhs[vid] != 0) {
                            Cost mincost = kInf;
                            std::vector<NodeId> v;
                            Base::mg.neighbors(vid, v);
                            for (auto spid : v) {
                                mincost = std::min(mincost, satSum(Base::mg.edgeCost(spid, vid), g[spid]));
                            }
                            rhs[vid] = mincost;
                        }
                    }
                    updateNode(vid);
                }
                computeShortestPath();
            }
            id_current = lowestNeighbor(id_current).first;
        }
    }

    void resetCostsAndLists()
    {
        key_modifier = Cost(0);

        g.fill(kInf);
        rhs.fill(kInf);

        open_list.clear();
        in_open_list.reset();
    }
    void reset()
    {
        resetCostsAndLists();
        id_current = Base::mg.getStartNodeId();
        ids_destination.clear();
        Base::mg.getGoalNodeIds(ids_destination);
        id_last = id_current;
    }
    void initialize()
    {
        reset();
        for (auto id : ids_destination) {
            rhs[id] = 0;
            open_list.insert({ { Base::mg.distance(id_current, id), 0 }, id });
            in_open_list.set(id);
        }
        computeShortestPath();
    }
    void changeDestinations(const std::vector<NodeId>& ids)
    {
        resetCostsAndLists();
        // id_current = id_current
        ids_destination.clear();
        ids_destination.insert(ids_destination.end(), ids.begin(), ids.end());
        id_last = id_current;
        for (auto id : ids_destination) {
            rhs[id] = 0;
            open_list.insert({ { Base::mg.distance(id_current, id), 0 }, id });
            in_open_list.set(id);
        }
        computeShortestPath();
    }
};

}
