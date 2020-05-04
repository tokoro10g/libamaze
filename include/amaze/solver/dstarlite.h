/**
 * Copyright (c) 2020 Tokoro
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef INCLUDE_AMAZE_SOLVER_DSTARLITE_H_
#define INCLUDE_AMAZE_SOLVER_DSTARLITE_H_

#include <algorithm>
#include <array>
#include <bitset>
#include <cassert>
#include <set>
#include <unordered_set>
#include <vector>

#include "amaze/solver/solver_base.h"

namespace amaze {
namespace solver {

/// \~japanese
/// D* Liteソルバ
///
/// 実装は論文(http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf)をもとにしています．
///
/// \~english
/// D* Lite solver
///
/// Implementation is based on the paper
/// (http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf).
///
/// \~
/// Sven Koenig and Maxim Likhachev, "D* Lite", 2002.
template <typename TCost, typename TNodeId, uint8_t W, TNodeId NodeCount>
class DStarLite : public SolverBase<TCost, TNodeId, W, NodeCount>,
                  public IncrementalSolverBase {
 public:
  using MazeGraph = maze_graph::MazeGraphBase<TCost, TNodeId, W, NodeCount>;
  using Base = SolverBase<TCost, TNodeId, W, NodeCount>;
  using NodeId = TNodeId;
  using Cost = TCost;
  using HeapKey = std::pair<Cost, Cost>;

  using Base::changeDestinations;
  using Base::kMaxCost;
  using Base::reconstructPath;

  /// \~japanese
  /// ヒープ内のキー値のコンパレータ構造体．
  ///
  /// \~english
  /// Comparator structure for the key value of the heap.
  struct KeyCompare {
    bool operator()(const std::pair<HeapKey, NodeId> &lhs,
                    const std::pair<HeapKey, NodeId> &rhs) const {
      if (lhs.first < rhs.first) {
        return true;
      } else if (lhs.first == rhs.first && lhs.second > rhs.second) {
        return true;
      } else {
        return false;
      }
    }
  };

  explicit DStarLite(const MazeGraph *mg)
      : SolverBase<TCost, TNodeId, W, NodeCount>(mg),
        id_current(mg->startNodeId()),
        id_last(id_current),
        id_last_modified(id_current),
        ids_destination(),
        key_modifier(0),
        g(),
        rhs(),
        open_list(),
        in_open_list(0) {
    ids_destination = mg->goalNodeIds();
    g.fill(kMaxCost);
    rhs.fill(kMaxCost);
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
  void updateHeap(NodeId id, HeapKey k) {
    if (id > NodeCount) /* [[unlikely]] */ {
      return;
    }
    auto it = find_if(open_list.begin(), open_list.end(),
                      [=](auto p) { return p.second == id; });
    if (it == open_list.end() || it->first == k) {
      return;
    }
    open_list.erase(it);
    open_list.insert({k, id});
  }

  /// \~japanese
  /// ノードに関するデータを更新します．
  ///
  /// \p g の値が \p rhs
  /// の値と異なる場合は変更/追加を行い，等しい場合にはヒープから削除します．
  ///
  /// \param[in] id ノードID
  ///
  /// \~english
  /// Updates node data.
  ///
  /// If the value of \p g is different from \p rhs, then the element is updated
  /// or inserted. If they are equal, the element is erased from the heap.
  ///
  /// \param[in] id Node ID
  void updateNode(NodeId id) {
    if (id > NodeCount) /* [[unlikely]] */ {
      return;
    }
    if (g[id] != rhs[id] && in_open_list[id]) {
      updateHeap(id, calculateKey(id));
    } else if (g[id] != rhs[id] && !in_open_list[id]) {
      open_list.insert({calculateKey(id), id});
      in_open_list.set(id);
    } else if (g[id] == rhs[id] && in_open_list[id]) {
      auto it = find_if(open_list.begin(), open_list.end(),
                        [=](auto p) { return p.second == id; });
      if (it == open_list.end()) {
        __builtin_unreachable();  // should not reach here
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
  void computeShortestPath(
      std::unordered_map<Position, bool> wall_overrides = {}) {
    NodeId examined_nodes = 0;
    NodeId max_heap_size = 0;
    while (!open_list.empty() &&
           (open_list.begin()->first < calculateKey(id_current) ||
            rhs[id_current] > g[id_current])) {
      auto [kold, uid] = *open_list.begin();
      auto knew = calculateKey(uid);
#if 0
      std::cout << static_cast<int>(kold.first) << ","
                << static_cast<int>(kold.second) << " "
                << static_cast<int>(knew.first) << ","
                << static_cast<int>(knew.second) << std::endl;
      std::cout << uid << ": " << static_cast<int>(g[uid]) << ","
                << static_cast<int>(rhs[uid]) << std::endl;
#endif
      examined_nodes++;
      if (kold < knew) {
        updateHeap(uid, knew);
      } else if (g[uid] > rhs[uid]) {
        g[uid] = rhs[uid];
        open_list.erase(open_list.begin());
        if (!in_open_list.test(uid)) {
          // this node (uid) is processed twice
          continue;
        }
        in_open_list.reset(uid);

        for (auto &[sid, scost] :
             Base::mg->neighborEdges(uid, wall_overrides)) {
          if (rhs[sid] != 0) {
            rhs[sid] = std::min(rhs[sid], satSum(scost, g[uid]));
          }
          updateNode(sid);
        }
      } else {
        Cost gold = g[uid];
        g[uid] = kMaxCost;
        auto v = Base::mg->neighborEdges(uid, wall_overrides);
        v.push_back({uid, 0});
        for (auto &[sid, scost] : v) {
          if (rhs[sid] == satSum(scost, gold)) {
            if (rhs[sid] != 0) {
              Cost mincost = kMaxCost;
              for (auto &[spid, spcost] :
                   Base::mg->neighborEdges(sid, wall_overrides)) {
                mincost = std::min(mincost, satSum(spcost, g[spid]));
              }
              rhs[sid] = mincost;
            }
          }
          updateNode(sid);
        }
      }
      if (open_list.size() > max_heap_size) {
        max_heap_size = NodeId(open_list.size());
      }
    }
    // computation finished
#if 0
    std::cout << "The number of examined nodes in this round: "
              << examined_nodes << std::endl;
    std::cout << "Maximum size of the open list: " << max_heap_size
              << std::endl;
#endif
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
  HeapKey calculateKey(NodeId id) const {
    if (id > NodeCount) /* [[unlikely]] */ {
      return {kMaxCost, kMaxCost};
    }
    return {satSum(satSum(std::min(g[id], rhs[id]),
                          Base::mg->distance(id_current, id)),
                   key_modifier),
            std::min(g[id], rhs[id])};
  }

  NodeId currentNodeId() const override { return id_current; }

  NodeId lastNodeId() const override { return id_last; }

  std::set<NodeId> destinationNodeIds() const override {
    return ids_destination;
  }

  SolverState currentSolverState() const override {
    if (rhs[id_current] == 0) {
      return SolverState::kReached;
    }
    if (rhs[id_current] == kMaxCost) {
      return SolverState::kFailed;
    }
    return SolverState::kInProgress;
  }

  std::pair<NodeId, Cost> lowestNeighbor(
      NodeId id, std::unordered_map<Position, bool> wall_overrides = {}) const {
    NodeId argmin = id;
    Cost mincost = kMaxCost;
#define AMAZE_DEBUG 0
#if AMAZE_DEBUG
    std::cout << "neighbors of " << id << ": ";
#endif
    for (auto &[spid, spcost] : Base::mg->neighborEdges(id, wall_overrides)) {
      Cost cost = satSum(spcost, g[spid]);
#if AMAZE_DEBUG
      std::cout << spid << "(" << cost << "), ";
#endif
      if (mincost >= cost) {
        mincost = cost;
        argmin = spid;
      }
    }
#if AMAZE_DEBUG
    std::cout << std::endl;
#endif
    return {argmin, mincost};
  }

  std::vector<AgentState> reconstructPath(
      NodeId id_from, const std::set<NodeId> &ids_to) const override {
    std::vector<AgentState> path;
    path.push_back(Base::mg->agentStateByNodeId(id_from));
    NodeId id_current_on_path = id_from;
    NodeId id_last_on_path = id_from;
    while (ids_to.find(id_current_on_path) == ids_to.end()) {
      auto [nid, ncost] = lowestNeighbor(id_current_on_path);
      if (ncost == kMaxCost) /* [[unlikely]] */ {
        return std::vector<AgentState>();
      }
      id_last_on_path = id_current_on_path;
      id_current_on_path = nid;
      path.push_back(
          Base::mg->agentStateByEdge(id_last_on_path, id_current_on_path));
    }
    return path;
  }

  void processBeforeReplanning(
      const std::unordered_map<Position, bool> &wall_overrides [[maybe_unused]],
      const Position &last_key [[maybe_unused]]) override {
    auto it = wall_overrides.find(last_key);
    if (it == wall_overrides.end()) {
      return;
    }

    key_modifier =
        satSum(key_modifier, Base::mg->distance(id_last_modified, id_current));
    id_last_modified = id_current;

    Position position = it->first;
    bool wall_state = it->second;

    for (auto &[uid, vid] : Base::mg->affectedEdges(position)) {
      Cost cold = Base::mg->edgeWithHypothesis(uid, vid, !wall_state).cost;
      Cost cnew = Base::mg->edgeWithHypothesis(uid, vid, wall_state).cost;
      if (cold > cnew) /* [[unlikely]] */ {
        if (rhs[uid] != 0) {
          rhs[uid] = std::min(rhs[uid], satSum(cnew, g[vid]));
        }
      } else if (rhs[uid] == satSum(cold, g[vid])) {
        if (rhs[uid] != 0) {
          Cost mincost = kMaxCost;
          for (auto &[spid, spcost] :
               Base::mg->neighborEdges(uid, wall_overrides)) {
            mincost = std::min(mincost, satSum(spcost, g[spid]));
          }
          rhs[uid] = mincost;
        }
      }
      updateNode(uid);
      if (cold > cnew) /* [[unlikely]] */ {
        if (rhs[vid] != 0) {
          rhs[vid] = std::min(rhs[vid], satSum(cnew, g[uid]));
        }
      } else if (rhs[vid] == satSum(cold, g[uid])) {
        if (rhs[vid] != 0) {
          Cost mincost = kMaxCost;
          for (auto &[spid, spcost] :
               Base::mg->neighborEdges(vid, wall_overrides)) {
            mincost = std::min(mincost, satSum(spcost, g[spid]));
          }
          rhs[vid] = mincost;
        }
      }
      updateNode(vid);
    }
  }

  void processAfterReplanning(
      const std::unordered_map<Position, bool> &wall_overrides) {
    for (const auto &[key, value] : wall_overrides) {
      processBeforeReplanning(wall_overrides, key);
    }
  }

  void preSense(const std::unordered_set<Position> &sense_positions
                [[maybe_unused]]) override {}

  void postSense(
      const std::unordered_map<Position, bool> &wall_overrides) override {
    if (currentSolverState() == SolverState::kReached) {
#if 0
      std::cout << "Reached the destination" << std::endl;
#endif
      return;
    }
    if (currentSolverState() == SolverState::kFailed) /* [[unlikely]] */ {
#if 0
      std::cerr << "No route" << std::endl;
#endif
      return;
    }

    if (!wall_overrides.empty()) {
      processAfterReplanning(wall_overrides);
      computeShortestPath();
    }
    id_last = id_current;
    id_current = lowestNeighbor(id_current).first;
  }

  void resetCostsAndLists() {
    key_modifier = Cost(0);

    g.fill(kMaxCost);
    rhs.fill(kMaxCost);

    open_list.clear();
    in_open_list.reset();
  }

  void reset() override {
    resetCostsAndLists();
    id_current = Base::mg->startNodeId();
    ids_destination = Base::mg->goalNodeIds();
    id_last = id_current;
    id_last_modified = id_current;
  }

  void initialize() {
    reset();
    for (auto id : ids_destination) {
      rhs[id] = 0;
      open_list.insert({{Base::mg->distance(id_current, id), 0}, id});
      in_open_list.set(id);
    }
    computeShortestPath();
  }

  void changeDestinations(const std::set<NodeId> &ids) override {
    resetCostsAndLists();
    // id_current = id_current
    ids_destination = ids;
    id_last = id_current;
    id_last_modified = id_current;
    for (auto id : ids_destination) {
      rhs[id] = 0;
      open_list.insert({{Base::mg->distance(id_current, id), 0}, id});
      in_open_list.set(id);
    }
    computeShortestPath();
  }

 protected:
  /// \~japanese 現在のノードのID
  /// \~english Current node ID
  NodeId id_current;
  /// \~japanese 一手前のノードのID
  /// \~english Last node ID
  NodeId id_last;
  NodeId id_last_modified;
  /// \~japanese 終点ノードのID
  /// \~english Destination node ID
  std::set<NodeId> ids_destination;

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
};

}  // namespace solver
}  // namespace amaze
#endif  // INCLUDE_AMAZE_SOLVER_DSTARLITE_H_
