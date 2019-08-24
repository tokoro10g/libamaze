#pragma once

#include "solver.h"
#include <algorithm>
#include <array>
#include <bitset>
#include <iostream>
#include <limits>
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
    using NodeId = typename TMazeGraph::NodeId;
    using Cost = typename TMazeGraph::Cost;
    using HeapKey = std::pair<Cost, Cost>;

private:
    Cost key_modifier;

    /// \~japanese 現在のスタートノードのID
    /// \~english Current start node ID
    NodeId id_start;
    /// \~japanese ゴールノードのID
    /// \~english Goal node ID
    NodeId id_goal;
    /// \~japanese 一手前のノードのID
    /// \~english Last node ID
    NodeId id_last;

    /// g value
    std::array<Cost, TMazeGraph::size> g;
    /// rhs value
    std::array<Cost, TMazeGraph::size> rhs;

    /// Open list
    std::vector<std::pair<HeapKey, NodeId>> open_list;
    /// \~japanese Open listにノードが入っているかどうかのフラグ
    /// \~english Flags whether nodes are in the open list
    std::bitset<TMazeGraph::size> in_open_list;

public:
    /// \~japanese 無限コストとみなす値
    /// \~english Cost value assumed to be infinity
    static constexpr Cost INF = TMazeGraph::INF;

    /// \~japanese
    /// ヒープ内のキー値のコンパレータ構造体．
    ///
    /// \~english
    /// Comparator structure for the key value of the heap.
    struct KeyCompare {
        bool operator()(const std::pair<HeapKey, NodeId>& a, const std::pair<HeapKey, NodeId>& b) const
        {
            return a.first > b.first;
        }
    };

    DStarLite(const TMazeGraph& mg)
        : Solver<TMazeGraph>(mg)
        , key_modifier(0)
        , id_start(mg.getStartNodeId())
        , id_goal(mg.getGoalNodeId())
        , id_last(id_start)
        , g()
        , rhs()
        , open_list()
        , in_open_list(0)
    {
        g.fill(INF);
        rhs.fill(INF);
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
        if (id > TMazeGraph::size) {
            return;
        }
        for (auto& p : open_list) {
            if (p.second == id) {
                p.first = k;
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
        if (id > TMazeGraph::size) {
            return;
        }
        if (g[id] != rhs[id] && in_open_list[id]) {
            updateHeap(id, calculateKey(id));
        } else if (g[id] != rhs[id] && !in_open_list[id]) {
            open_list.push_back({ calculateKey(id), id });
            push_heap(open_list.begin(), open_list.end(), KeyCompare());
            in_open_list.set(id);
        } else if (g[id] == rhs[id] && in_open_list[id]) {
            auto it = find_if(open_list.begin(), open_list.end(), [=](auto p) { return p.second == id; });
            open_list.erase(it);
            in_open_list.reset(id);
        }
    }

    /// \~japanese
    /// 現在のスタートからゴールまでの最短経路を計算します．
    ///
    /// \~english
    /// Computes the shortest path from the current start node to the goal.
    void computeShortestPath()
    {
        NodeId examined_nodes = 0;
        NodeId max_heap_size = 0;
        while (!open_list.empty() && (open_list.front().first < calculateKey(id_start) || rhs[id_start] > g[id_start])) {
            auto uid = open_list.front().second;
            auto kold = open_list.front().first;
            auto knew = calculateKey(uid);
            //std::cout<<(int)kold.first<<","<<(int)kold.second<<" "<<(int)knew.first<<","<<(int)knew.second<<std::endl;
            //std::cout<<uid<<": "<<(int)g[uid]<<","<<(int)rhs[uid]<<std::endl;
            examined_nodes++;
            if (kold < knew) {
                updateHeap(uid, knew);
            } else if (g[uid] > rhs[uid]) {
                g[uid] = rhs[uid];
                pop_heap(open_list.begin(), open_list.end(), KeyCompare());
                open_list.pop_back();
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
                g[uid] = INF;
                auto f = [=](NodeId sid) {
                    if (rhs[sid] == satSum(Base::mg.edgeCost(sid, uid), gold)) {
                        if (rhs[sid] != 0) {
                            Cost mincost = INF;
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
            make_heap(open_list.begin(), open_list.end(), KeyCompare());
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
        if (id > TMazeGraph::size) {
            return { INF, INF };
        }
        return { satSum(satSum(std::min(g[id], rhs[id]), Base::mg.distance(id_start, id)), key_modifier), std::min(g[id], rhs[id]) };
    }
    NodeId getNextNodeId() const
    {
        //TODO: Implement
        return NodeId(0);
    }
    NodeId getCurrentNodeId() const
    {
        return id_start;
    }
    void preSense()
    {
        if (rhs[id_start] == 0) {
            // reached the goal
            // TODO: implement
            std::cout << "Reached the goal" << std::endl;
        } else {
            if (rhs[id_start] == INF) {
                // no route
                // TODO: implement
                std::cerr << "No route" << std::endl;
                return;
            }
            NodeId argmin = id_start;
            Cost mincost = INF;
            std::vector<NodeId> v;
            Base::mg.neighbors(id_start, v);
            for (auto spid : v) {
                Cost cost = satSum(Base::mg.edgeCost(id_start, spid), g[spid]);
                //std::cout << spid << ": " << (int)cost << std::endl;
                if (mincost > cost) {
                    mincost = cost;
                    argmin = spid;
                }
            }
            id_start = argmin;
        }
        return;
    }
    void postSense(const std::vector<Coordinates>& changed_coordinates)
    {
        //TODO: Implement
        if (rhs[id_start] == 0) {
            // reached the goal
            // TODO: implement
            std::cout << "Reached the goal" << std::endl;
        } else {
            if (rhs[id_start] == INF) {
                // no route
                // TODO: implement
                std::cerr << "No route" << std::endl;
                return;
            }

            if (!changed_coordinates.empty()) {
                key_modifier = satSum(key_modifier, Base::mg.distance(id_last, id_start));
                id_last = id_start;
                std::vector<std::pair<NodeId, NodeId>> changed_edges;
                Base::mg.affectedEdges(changed_coordinates, changed_edges);
                for (auto e : changed_edges) {
                    Cost cold;
                    if (Base::mg.edgeCost(e.first, e.second) == INF) {
                        // assume that the path is NOT blocked in the previous step
                        cold = Base::mg.getEdgeWithHypothesis(e.first, e.second, false).second;
                    } else {
                        // assume that the path IS blocked in the previous step
                        cold = Base::mg.getEdgeWithHypothesis(e.first, e.second, true).second;
                    }
                    auto uid = e.first;
                    auto vid = e.second;
                    if (cold > INF) {
                        // TODO: probably unnecessary
                        if (rhs[uid] != 0) {
                            rhs[uid] = std::min(rhs[uid], satSum(INF, g[vid]));
                        }
                    } else if (rhs[uid] == satSum(cold, g[vid])) {
                        if (rhs[uid] != 0) {
                            Cost mincost = INF;
                            std::vector<NodeId> v;
                            Base::mg.neighbors(uid, v);
                            for (auto spid : v) {
                                mincost = std::min(mincost, satSum(Base::mg.edgeCost(spid, uid), g[spid]));
                            }
                            rhs[uid] = mincost;
                        }
                    }
                    updateNode(uid);
                    if (cold > INF) {
                        // TODO: probably unnecessary
                        if (rhs[vid] != 0) {
                            rhs[vid] = std::min(rhs[vid], satSum(INF, g[uid]));
                        }
                    } else if (rhs[vid] == satSum(cold, g[uid])) {
                        if (rhs[vid] != 0) {
                            Cost mincost = INF;
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
                make_heap(open_list.begin(), open_list.end(), KeyCompare());
                computeShortestPath();
            }
        }
    }
    void reset()
    {
        key_modifier = Cost(0);
        id_start = Base::mg.getStartNodeId();
        id_goal = Base::mg.getGoalNodeId();
        id_last = id_start;

        g.fill(INF);
        rhs.fill(INF);

        open_list.clear();
        in_open_list.reset();
    }
    void initialize()
    {
        reset();
        rhs[id_goal] = 0;
        open_list.push_back({ { Base::mg.distance(id_start, id_goal), 0 }, id_goal });
        in_open_list.set(id_goal);
        computeShortestPath();
    }
};

}
