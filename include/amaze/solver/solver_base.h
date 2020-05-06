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

#ifndef AMAZE_SOLVER_SOLVER_BASE_H_
#define AMAZE_SOLVER_SOLVER_BASE_H_

#include <set>
#include <unordered_set>
#include <vector>

#include "amaze/maze_graph/maze_graph_base.h"

namespace amaze {
namespace solver {

/// \~japanese
/// ソルバの結果を表す列挙型．
///
/// \~english
/// Enumerator that represents the result of a solver.
enum class SolverState { kInProgress = 0, kReached, kFailed };

/// \~japanese
/// 最短経路ソルバの抽象クラス．
///
/// \tparam TMazeGraph MazeGraphを実装した型
///
/// \~english
/// An abstract class for shortest path solvers.
///
/// \tparam TMazeGraph Class implements MazeGraph
template <typename TCost, typename TNodeId, uint8_t W, TNodeId NodeCount>
class SolverBase {
 public:
  using MazeGraph = maze_graph::MazeGraphBase<TCost, TNodeId, W, NodeCount>;

 protected:
  /// \~japanese 迷路のグラフ表現
  /// \~english Graph representation of the maze
  const MazeGraph *mg;

  /// \~japanese 現在のノードのID
  /// \~english Current node ID
  TNodeId id_current;
  /// \~japanese 一手前のノードのID
  /// \~english Last node ID
  TNodeId id_last;
  /// \~japanese 終点ノードのID
  /// \~english Destination node ID
  std::set<TNodeId> ids_destination;

 public:
  using NodeId = TNodeId;
  using Cost = TCost;

  explicit SolverBase(const MazeGraph *mg)
      : mg(mg),
        id_current(mg->startNodeId()),
        id_last(id_current),
        ids_destination() {}

  SolverBase(const SolverBase &s) = default;

  SolverBase &operator=(const SolverBase &other) {
    if (this != &other) {
      this->mg = other.mg;
    }
    return *this;
  }

  virtual ~SolverBase() = default;

  static constexpr uint8_t kMaxWidth = W;
  static constexpr NodeId kNodeCount = NodeCount;

  /// \~japanese 無限コストとみなす値
  /// \~english Cost value assumed to be infinity
  static constexpr Cost kMaxCost = MazeGraph::kMaxCost;

  /// \~japanese
  /// 迷路のグラフ表現を置き換えます．
  /// \param[in] new_mg 新たな迷路グラフのポインタ
  ///
  /// \~english
  /// Replaces graph representation of the maze.
  /// \param[in] new_mg Pointer to the new maze graph.
  inline void changeMazeGraph(const MazeGraph *new_mg) {
    mg = new_mg;
    initialize();
  }

  /// \~japanese
  /// 現在のノードのエージェント状態を返します．
  /// \returns 現在のエージェント状態
  ///
  /// \~english
  /// Returns the agent state of the current node.
  /// \returns current agent state.
  inline AgentState currentAgentState() const {
    if (lastNodeId() == currentNodeId()) {
      return mg->agentStateByNodeId(currentNodeId());
    } else {
      return mg->agentStateByEdge(lastNodeId(), currentNodeId());
    }
  }

  /// \~japanese
  /// 終点のノードのエージェント状態を返します．
  /// \returns 終点のエージェント状態のリスト
  ///
  /// \~english
  /// Returns the agent states of the destination nodes.
  /// \returns List of agent states at the destinations.
  inline std::unordered_set<AgentState> destinationAgentStates() const {
    std::unordered_set<AgentState> states;
    for (auto id : destinationNodeIds()) {
      states.insert(mg->agentStateByNodeId(id));
    }
    return states;
  }

  /// \~japanese
  /// 現在のノードのIDを返します．
  /// \returns 現在のID
  ///
  /// \~english
  /// Returns the ID of the current node.
  /// \returns current ID.
  virtual NodeId currentNodeId() const = 0;

  /// \~japanese
  /// 1ステップ前のノードのIDを返します．
  /// \returns 1ステップ前のID
  ///
  /// \~english
  /// Returns the ID of the last node.
  /// \returns last ID.
  virtual NodeId lastNodeId() const = 0;

  /// \~japanese
  /// 終点ノードのIDを返します．
  /// \returns 終点のIDのリスト
  ///
  /// \~english
  /// Returns the ID of the destination node.
  /// \returns List of destination IDs.
  virtual std::set<NodeId> destinationNodeIds() const = 0;

  /// \~japanese
  /// ソルバの結果を返します．
  /// \returns ソルバの結果
  ///
  /// \~english
  /// Returns the solver result.
  /// \returns Solver result
  virtual SolverState currentSolverState() const = 0;

  /// \~japanese
  /// 最もコストの低い隣接ノードのIDとコストを返します．
  /// \param[in] id ノードID
  /// \returns 隣接ノードのIDとコスト
  ///
  /// \~english
  /// Returns the ID and cost of the lowest-cost neighbor.
  /// \param[in] id Node ID
  /// \returns ID and cost of the neighbor node.
  virtual std::pair<NodeId, Cost> lowestNeighbor(
      NodeId id,
      std::unordered_map<Position, bool> wall_overrides = {}) const = 0;

  /// \~japanese
  /// 現在のグラフの情報から最短経路を構成します．
  /// \param[in] id_from, id_to 経路の始点と終点
  /// \returns 経路を格納する \p std::vector
  ///
  /// \~english
  /// Reconstructs the shortest path based on the current graph data.
  /// \param[in] id_from, id_to Start and end of the path
  /// \returns \p std::vector to return the path
  inline std::vector<AgentState> reconstructPath(NodeId id_from,
                                                 NodeId id_to) const {
    std::set<NodeId> s;
    s.insert(id_to);
    return reconstructPath(id_from, s);
  }

  /// \~japanese
  /// 現在のグラフの情報から最短経路を構成します．
  /// \param[in] id_from, ids_to 経路の始点と終点
  /// \returns 経路を格納する \p std::vector
  ///
  /// \~english
  /// Reconstructs the shortest path based on the current graph data.
  /// \param[in] id_from, ids_to Start and ends of the path
  /// \returns \p std::vector to return the path
  virtual std::vector<AgentState> reconstructPath(
      NodeId id_from, const std::set<NodeId> &ids_to) const = 0;

  virtual void processBeforeReplanning() = 0;

  /// \~japanese
  /// 壁センシング前の処理を行います．
  /// \param [in] sensed_positions センシングする壁等の位置
  ///
  /// \~english
  /// Performs pre-sensing routines.
  /// \param [in] sense_positions Positions of walls etc. which are going to be
  /// sensed
  virtual void preSense(
      const std::unordered_set<Position> &sense_positions) = 0;

  /// \~japanese
  /// 壁センシング後の処理を行います．
  ///
  /// \param [in] wall_overrides センシングした壁等の位置
  ///
  /// \~english
  /// Performs post-sensing routines.
  /// \param [in] wall_overrides Positions of sensed walls etc.
  virtual void postSense(
      const std::unordered_map<Position, bool> &wall_overrides) = 0;

  /// \~japanese
  /// ソルバの内部状態をすべて初期値クリアします．
  ///
  /// \~english
  /// Resets all the internal variables to the initial value.
  virtual void reset() = 0;

  /// \~japanese
  /// ソルバの内部状態を探索開始直前の状態に初期化します．
  ///
  /// \~english
  /// Initializes internal variables to be ready for a new search from the
  /// start.
  virtual void initialize() = 0;

  /// \~japanese
  /// 終点を変更し，現在のノードを起点とした探索開始直前の状態に初期化します．
  /// \param[in] ids 終点のノードIDのリスト
  ///
  /// \~english
  /// Changes destination and initializes internal variables to be ready for a
  /// new search originated from the current node. \param[in] ids List of node
  /// IDs at the destinations
  virtual void changeDestinations(const std::set<NodeId> &ids) = 0;

  /// \~japanese
  /// 終点を変更し，現在のノードを起点とした探索開始直前の状態に初期化します．
  /// \param[in] id 終点のノードID
  ///
  /// \~english
  /// Changes destination and initializes internal variables to be ready for a
  /// new search originated from the current node. \param[in] id Node ID at the
  /// destination
  inline void changeDestinations(NodeId id) {
    std::set<NodeId> s;
    s.insert(id);
    changeDestinations(s);
  }
};

struct AllAtOnceSolverBase {
  static constexpr bool kIsIncremental = false;
  virtual ~AllAtOnceSolverBase() = default;
};

struct IncrementalSolverBase {
  static constexpr bool kIsIncremental = true;
  virtual ~IncrementalSolverBase() = default;
};

}  // namespace solver
}  // namespace amaze
#endif  // AMAZE_SOLVER_SOLVER_BASE_H_
