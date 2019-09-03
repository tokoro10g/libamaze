#pragma once

#include "mazegraph.h"

namespace Amaze {

/// \~japanese
/// 最短経路ソルバの抽象クラス．
///
/// \tparam TMazeGraph MazeGraphを実装した型
///
/// \~english
/// An abstract class for shortest path solvers.
///
/// \tparam TMazeGraph Class implements MazeGraph
template <typename TMazeGraph>
class Solver {
protected:
    /// \~japanese 迷路のグラフ表現
    /// \~english Graph representation of the maze
    const TMazeGraph& mg;

public:
    using NodeId = typename TMazeGraph::NodeId;
    using Cost = typename TMazeGraph::Cost;

    Solver(const TMazeGraph& mg)
        : mg(mg)
    {
    }
    virtual ~Solver() {}

    /// \~japanese
    /// 次に訪れるノードのエージェント状態を返します．
    /// \returns 次のエージェント状態
    ///
    /// \~english
    /// Returns the agent state of the node the solver is going to visit next.
    /// \returns next agent state.
    virtual AgentState getNextAgentState() const
    {
        return mg.agentStateByNodeId(getNextNodeId());
    }
    /// \~japanese
    /// 現在のノードのエージェント状態を返します．
    /// \returns 現在のエージェント状態
    ///
    /// \~english
    /// Returns the agent state of the current node.
    /// \returns current agent state.
    virtual AgentState getCurrentAgentState() const
    {
        return mg.agentStateByNodeId(getCurrentNodeId());
    }
    /// \~japanese
    /// 終点のノードのエージェント状態を返します．
    /// \returns 終点のエージェント状態
    ///
    /// \~english
    /// Returns the agent state of the destination node.
    /// \returns agent state at the destination.
    virtual AgentState getDestinationAgentState() const
    {
        return mg.agentStateByNodeId(getDestinationNodeId());
    }
    /// \~japanese
    /// 次に訪れるノードのIDを返します．
    /// \returns 次のID
    ///
    /// \~english
    /// Returns the ID of the node the solver is going to visit next.
    /// \returns next ID.
    virtual NodeId getNextNodeId() const = 0;
    /// \~japanese
    /// 現在のノードのIDを返します．
    /// \returns 現在のID
    ///
    /// \~english
    /// Returns the ID of the current node.
    /// \returns current ID.
    virtual NodeId getCurrentNodeId() const = 0;
    /// \~japanese
    /// 終点ノードのIDを返します．
    /// \returns 次のID
    ///
    /// \~english
    /// Returns the ID of the destination node.
    /// \returns next ID.
    virtual NodeId getDestinationNodeId() const = 0;

    /// \~japanese
    /// 現在のグラフの情報から最短経路を構成します．
    /// \param[in] id_from, id_to 経路の始点と終点
    /// \param[out] path 経路を格納する \p std::vector
    /// \returns 成功した場合 \p true を返します．
    ///
    /// \~english
    /// Reconstructs the shortest path based on the current graph data.
    /// \param[in] id_from, id_to Start and end of the path
    /// \param[out] path \p std::vector to return the path
    /// \returns \p true if success
    virtual bool reconstructPath(NodeId id_from, NodeId id_to, std::vector<AgentState>& path) const = 0;
    /// \~japanese
    /// 現在のグラフの情報から最短経路を構成します．
    /// \param[in] id_from, id_to 経路の始点と終点
    /// \param[out] path 経路を格納する \p std::vector
    /// \returns 成功した場合 \p true を返します．
    ///
    /// \~english
    /// Reconstructs the shortest path based on the current graph data.
    /// \param[in] id_from, id_to Start and end of the path
    /// \param[out] path \p std::vector to return the path
    /// \returns \p true if success
    virtual bool reconstructPath(NodeId id_from, NodeId id_to, std::vector<NodeId>& path) const = 0;

    /// \~japanese
    /// 壁センシング前の処理を行います．
    ///
    /// \~english
    /// Performs pre-sensing routines.
    virtual void preSense() = 0;
    /// \~japanese
    /// 壁センシング後の処理を行います．
    ///
    /// \param [in] sensed_positions センシングした壁等の位置
    ///
    /// \~english
    /// Performs post-sensing routines.
    /// \param [in] sensed_positions Positions of sensed walls etc.
    virtual void postSense(const std::vector<Position>& sensed_positions) = 0;

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
    /// Initializes internal variables to be ready for a new search from the start.
    virtual void initialize() = 0;
    /// \~japanese
    /// 終点を変更し，現在のノードを起点とした探索開始直前の状態に初期化します．
    ///
    /// \~english
    /// Changes destination and initializes internal variables to be ready for a new search originated from the current node.
    virtual void changeDestination(NodeId id) = 0;
};

}
