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
template <typename TCost, typename TNodeId, uint8_t W, TNodeId NodeCount>
class DStarLite : public Solver<TCost, TNodeId, W, NodeCount> {
public:
    using MazeGraphBase = MazeGraph<TCost, TNodeId, W, NodeCount>;
    using Base = Solver<TCost, TNodeId, W, NodeCount>;
    using NodeId = TNodeId;
    using Cost = TCost;
    using HeapKey = std::pair<Cost, Cost>;

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
    std::vector<NodeId> ids_destination;
    /// \~japanese 一手前のノードのID
    /// \~english Last node ID
    NodeId id_last;
    NodeId id_last_modified;

    /// Key modifier
    Cost key_modifier;
    /// g value
    std::array<Cost, NodeCount> g;
    /// rhs value
    std::array<Cost, NodeCount> rhs;

    /// Open list
    std::set<std::pair<HeapKey, NodeId>, KeyCompare> open_list;
    /// \~japanese Open listにノードが入っているかどうかのフラグ
    /// \~english Flags whether nodes are in the open list
    std::bitset<NodeCount> in_open_list;

public:
    using Base::changeMazeGraph;

    /// \~japanese 無限コストとみなす値
    /// \~english Cost value assumed to be infinity
    static constexpr Cost kInf = MazeGraphBase::kInf;

    explicit DStarLite(const MazeGraphBase* mg)
        : Solver<TCost, TNodeId, W, NodeCount>(mg)
        , id_current(mg->startNodeId())
        , ids_destination()
        , id_last(id_current)
        , id_last_modified(id_current)
        , key_modifier(0)
        , g()
        , rhs()
        , open_list()
        , in_open_list(0)
    {
        ids_destination = mg->goalNodeIds();
        g.fill(kInf);
        rhs.fill(kInf);
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
        if (id > NodeCount) {
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

                std::vector<std::pair<NodeId, Cost>> v = Base::mg->neighborEdges(uid);
                for (auto p : v) {
                    NodeId sid = p.first;
                    Cost scost = p.second;
                    if (rhs[sid] != 0) {
                        rhs[sid] = std::min(rhs[sid], satSum(scost, g[uid]));
                    }
                    updateNode(sid);
                }
            } else {
                Cost gold = g[uid];
                g[uid] = kInf;
                auto f = [&](std::pair<NodeId, Cost> edge) {
                    NodeId sid = edge.first;
                    Cost scost = edge.second;
                    if (rhs[sid] == satSum(scost, gold)) {
                        if (rhs[sid] != 0) {
                            Cost mincost = kInf;
                            std::vector<std::pair<NodeId, Cost>> v = Base::mg->neighborEdges(sid);
                            for (auto p : v) {
                                NodeId spid = p.first;
                                Cost spcost = p.second;
                                mincost = std::min(mincost, satSum(spcost, g[spid]));
                            }
                            rhs[sid] = mincost;
                        }
                    }
                    updateNode(sid);
                };
                std::vector<std::pair<NodeId, Cost>> v = Base::mg->neighborEdges(uid);
                for_each(v.begin(), v.end(), f);
                f({ uid, 0 });
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
        if (id > NodeCount) {
            return { kInf, kInf };
        }
        return { satSum(satSum(std::min(g[id], rhs[id]), Base::mg->distance(id_current, id)), key_modifier), std::min(g[id], rhs[id]) };
    }
    NodeId nextNodeId() const { return NodeId(0); }
    NodeId currentNodeId() const { return id_current; }
    NodeId lastNodeId() const { return id_last; }
    std::vector<NodeId> destinationNodeIds() const { return ids_destination; }

    std::pair<NodeId, Cost> lowestNeighbor(NodeId id) const
    {
        NodeId argmin = id;
        Cost mincost = kInf;
        std::vector<std::pair<NodeId, Cost>> v = Base::mg->neighborEdges(id);
        for (auto edge : v) {
            NodeId spid = edge.first;
            Cost spcost = edge.second;
            Cost cost = satSum(spcost, g[spid]);
            if (mincost >= cost) {
                mincost = cost;
                argmin = spid;
            }
        }
        return { argmin, mincost };
    }

    std::vector<AgentState> reconstructPath(NodeId id_from, const std::vector<NodeId>& ids_to) const
    {
        std::vector<AgentState> path;
        path.push_back(Base::mg->agentStateByNodeId(id_from));
        NodeId id_current_on_path = id_from;
        NodeId id_last_on_path = id_from;
        while (std::find(ids_to.begin(), ids_to.end(), id_current_on_path) == ids_to.end()) {
            auto n = lowestNeighbor(id_current_on_path);
            if (n.second == kInf) {
                return std::vector<AgentState>();
            }
            id_last_on_path = id_current_on_path;
            id_current_on_path = n.first;
            path.push_back(Base::mg->agentStateByEdge(id_last_on_path, id_current_on_path));
        }
        return path;
    }

    void preSense(const std::vector<Position>& sense_positions __attribute__((unused))) {}
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
                key_modifier = satSum(key_modifier, Base::mg->distance(id_last_modified, id_current));
                id_last_modified = id_current;
                std::vector<std::tuple<NodeId, NodeId, Cost>> changed_edges = Base::mg->affectedEdges(changed_positions);
                for (auto e : changed_edges) {
                    Cost cold;
                    if (std::get<2>(e) == kInf) {
                        // assume that the path is NOT blocked in the previous step
                        cold = Base::mg->edgeWithHypothesis(std::get<0>(e), std::get<1>(e), false).second;
                    } else {
                        // assume that the path IS blocked in the previous step
                        cold = Base::mg->edgeWithHypothesis(std::get<0>(e), std::get<1>(e), true).second;
                    }
                    auto uid = std::get<0>(e);
                    auto vid = std::get<1>(e);
                    if (cold > kInf) {
                        // TODO: probably unnecessary
                        if (rhs[uid] != 0) {
                            rhs[uid] = std::min(rhs[uid], satSum(kInf, g[vid]));
                        }
                    } else if (rhs[uid] == satSum(cold, g[vid])) {
                        if (rhs[uid] != 0) {
                            Cost mincost = kInf;
                            std::vector<std::pair<NodeId, Cost>> v = Base::mg->neighborEdges(uid);
                            for (auto edge : v) {
                                NodeId spid = edge.first;
                                Cost spcost = edge.second;
                                mincost = std::min(mincost, satSum(spcost, g[spid]));
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
                            std::vector<std::pair<NodeId, Cost>> v = Base::mg->neighborEdges(vid);
                            for (auto edge : v) {
                                NodeId spid = edge.first;
                                Cost spcost = edge.second;
                                mincost = std::min(mincost, satSum(spcost, g[spid]));
                            }
                            rhs[vid] = mincost;
                        }
                    }
                    updateNode(vid);
                }
                computeShortestPath();
            }
            id_last = id_current;
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
        id_current = Base::mg->startNodeId();
        ids_destination = Base::mg->goalNodeIds();
        id_last = id_current;
        id_last_modified = id_current;
    }
    void initialize()
    {
        reset();
        for (auto id : ids_destination) {
            rhs[id] = 0;
            open_list.insert({ { Base::mg->distance(id_current, id), 0 }, id });
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
        id_last_modified = id_current;
        for (auto id : ids_destination) {
            rhs[id] = 0;
            open_list.insert({ { Base::mg->distance(id_current, id), 0 }, id });
            in_open_list.set(id);
        }
        computeShortestPath();
    }
};

} // namespace Amaze
