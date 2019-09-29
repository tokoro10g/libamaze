#pragma once

#include "maze.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>

namespace Amaze {

/// \~japanese
/// 迷路のグラフ表現を扱う抽象クラス．
///
/// \tparam kExplore 探索用のグラフのとき \p true
/// \tparam TCost コストの型
/// \tparam TNodeId ノードIDの型
/// \tparam W 迷路の幅
///
/// \~english
/// An abstract class for the graph representation of the maze.
///
/// \tparam kExplore \p true if explore mode
/// \tparam TCost Type of the cost
/// \tparam TNodeId Type of the node ID
/// \tparam W Maze width
template <bool kExplore = true, typename TCost = uint16_t, typename TNodeId = uint16_t, uint8_t W = kDefaultMazeWidth>
class MazeGraph {
protected:
    /// \~japanese 迷路データ
    /// \~english Maze data
    const Maze<W>& maze;

public:
    using Cost = TCost;
    using NodeId = TNodeId;

    /// \~japanese グラフのサイズ
    /// \~english Cardinality of the graph
    static constexpr TNodeId kSize = 0;
    /// \~japanese 無限コストとみなす値
    /// \~english Cost value assumed to be infinity
    static constexpr TCost kInf = std::numeric_limits<TCost>::max();
    /// \~japanese 無効なノードID
    /// \~english ID for invalid nodes
    static constexpr TNodeId kInvalidNode = std::numeric_limits<TNodeId>::max();

    explicit MazeGraph(const Maze<W>& maze)
        : maze(maze)
    {
    }
    virtual ~MazeGraph() = default;

    /// \~japanese
    /// ノード \p id_from と \p id_to の間の楽観的距離を計算します．
    ///
    /// 戻り値は実際のグラフ上の2つのノード間の最短コストを超えてはなりません．
    ///
    /// \param[in] id_from, id_to ノードID
    /// \returns 距離．ノードのIDが無効な場合は \link MazeGraph::kInf kInf \endlink を返します．
    /// \warning 2つのノード間に経路が存在しない場合にも有効な値を返します．
    ///
    /// \~english
    /// Calculates optimistic distance between nodes \p id_from and \p id_to.
    ///
    /// The return value must not exceed the minimum travelling cost through the graph.
    ///
    /// \param[in] id_from, id_to Node IDs at the ends of the edge
    /// \returns distance; \link MazeGraph::kInf kInf \endlink when either node is invalid.
    /// \warning Returns valid value even if there is no route between the two nodes.
    virtual TCost distance(TNodeId id_from, TNodeId id_to) const = 0;

    /// \~japanese
    /// ノード \p id の隣接ノードのIDを \p v に格納します．
    ///
    /// \param[in] id 原点とするノードのID
    /// \returns 隣接ノードのIDを格納する\p std::vector
    ///
    /// \~english
    /// Fills \p v with neighbor IDs of the node \p id.
    ///
    /// \param[in] id Node ID of the source
    /// \returns \p std::vector filled with neighbors' IDs
    virtual std::vector<TNodeId> neighbors(TNodeId id) const
    {
        std::vector<TNodeId> v;
        std::vector<std::pair<TNodeId, TCost>> edges = neighborEdges(id);
        for (auto edge : edges) {
            v.push_back(edge.first);
        }
        return v;
    }

    /// \~japanese
    /// ノード \p id からのエッジを \p v に格納します．
    ///
    /// \param[in] id 原点とするノードのID
    /// \returns エッジを格納する\p std::vector
    ///
    /// \~english
    /// Fills \p v with edges from the node \p id.
    ///
    /// \param[in] id Node ID of the source
    /// \returns \p std::vector filled with edges
    virtual std::vector<std::pair<TNodeId, TCost>> neighborEdges(TNodeId id) const = 0;

    /// \~japanese
    /// 与えた位置への迷路情報の変更の影響を受けるエッジを列挙します．
    ///
    /// \param[in] positions 位置のリスト
    /// \returns エッジを格納する\p std::vector
    ///
    /// \~english
    /// Enumerates edges affected by the change to the maze data for given positions.
    ///
    /// \param[in] positions List of positions
    /// \returns \p std::vector filled with edges
    virtual std::vector<std::tuple<TNodeId, TNodeId, TCost>> affectedEdges(const std::vector<Position>& positions) const
    {
        std::vector<std::tuple<TNodeId, TNodeId, TCost>> edges;
        std::vector<TNodeId> visited;
        for (Position p : positions) {
            std::vector<TNodeId> ids = nodeIdsByPosition(p);
            for (auto id : ids) {
                if (id == kInvalidNode) {
                    continue;
                }
                if (std::find(visited.begin(), visited.end(), id) == visited.end()) {
                    visited.push_back(id);
                    std::vector<std::pair<NodeId, Cost>> v = neighborEdges(id);
                    for (auto n : v) {
                        edges.push_back({ id, n.first, n.second });
                    }
                }
            }
        }
        return edges;
    }

    /// \~japanese
    /// エッジがブロックされているかどうかを仮定してエッジの存在性とコストを計算します．
    ///
    /// \param[in] from, to エッジの両端の状態
    /// \param[in] blocked \p id_from と \p id_to の間のエッジがブロックされていると仮定するとき \p true
    /// \returns エッジの存在性とコストを格納した \p std::pair を返します．エッジやノードが無効のとき，コストは \link MazeGraph::kInf kInf \endlink になります．
    ///
    /// \~english
    /// Calculates the existence and cost of the edge based on the given assumption \p blocked.
    ///
    /// \param[in] from, to Agent states at the ends of the edge
    /// \param[in] blocked \p true if the edge between \p id_from and \p id_to is assumed to be blocked
    /// \returns \p std::pair which consists of the existence and cost of the edge. The cost is \link MazeGraph::kInf kInf \endlink if the edge is blocked or either node is invalid.
    virtual std::pair<bool, TCost> getEdgeWithHypothesis(AgentState from, AgentState to, bool blocked) const = 0;
    /// \~japanese
    /// エッジがブロックされているかどうかを仮定してエッジの存在性とコストを計算します．
    ///
    /// \param[in] id_from, id_to エッジの両端ノードのID
    /// \param[in] blocked \p id_from と \p id_to の間のエッジがブロックされていると仮定するとき \p true
    /// \returns エッジの存在性とコストを格納した \p std::pair を返します．エッジやノードが無効のとき，コストは \link MazeGraph::kInf kInf \endlink になります．
    ///
    /// \~english
    /// Calculates the existence and cost of the edge based on the given assumption \p blocked.
    ///
    /// \param[in] id_from, id_to Node IDs at the ends of the edge
    /// \param[in] blocked \p true if the edge between \p id_from and \p id_to is assumed to be blocked
    /// \returns \p std::pair which consists of the existence and cost of the edge. The cost is \link MazeGraph::kInf kInf \endlink if the edge is blocked or either node is invalid.
    virtual std::pair<bool, TCost> getEdgeWithHypothesis(TNodeId id_from, TNodeId id_to, bool blocked) const { return getEdgeWithHypothesis(agentStateByNodeId(id_from), agentStateByNodeId(id_to), blocked); }
    /// \~japanese
    /// 迷路データに基づいてエッジの存在性とコストを計算します．
    ///
    /// \param[in] from, to エッジの両端ノードのエージェント状態
    /// \returns エッジの存在性とコストを格納した \p std::pair を返します．エッジやノードが無効のとき，コストは \link MazeGraph::kInf kInf \endlink になります．
    ///
    /// \~english
    /// Calculates the existence and cost of the edge according to the maze data.
    ///
    /// \param[in] from, to Agent states at the ends of the edge
    /// \returns \p std::pair which consists of the existence and cost of the edge. The cost is \link MazeGraph::kInf kInf \endlink if the edge is blocked or either node is invalid.
    virtual std::pair<bool, TCost> getEdge(AgentState from, AgentState to) const = 0;
    /// \~japanese
    /// 迷路データに基づいてエッジの存在性とコストを計算します．
    ///
    /// このメソッドの戻り値は，\p id_from から \p id_to へ移動するときに訪れる壁の位置 \p p に対して<tt>getEdgeWithHypothesis(id_from, id_to, maze.isSetWall(p))</tt>と等価でなければなりません．
    ///
    /// \param[in] id_from, id_to エッジの両端ノードのID
    /// \returns エッジの存在性とコストを格納した \p std::pair を返します．エッジやノードが無効のとき，コストは \link MazeGraph::kInf kInf \endlink になります．
    ///
    /// \~english
    /// Calculates the existence and cost of the edge according to the maze data.
    ///
    /// The return value must be consistent with that of <tt>getEdgeWithHypothesis(id_from, id_to, maze.isSetWall(p))</tt> where \p p is the position of the wall visited by travelling from \p id_from to \p id_to.
    ///
    /// \param[in] id_from, id_to Node IDs at the ends of the edge
    /// \returns \p std::pair which consists of the existence and cost of the edge. The cost is \link MazeGraph::kInf kInf \endlink if the edge is blocked or either node is invalid.
    virtual std::pair<bool, TCost> getEdge(TNodeId id_from, TNodeId id_to) const { return getEdge(agentStateByNodeId(id_from), agentStateByNodeId(id_to)); }
    /// \~japanese
    /// 迷路データに基づいてエッジの存在性を返します．
    ///
    /// \param[in] id_from, id_to エッジの両端ノードのID
    /// \returns エッジの存在性を返します．
    ///
    /// \~english
    /// Returns the existence of the edge according to the maze data.
    ///
    /// \param[in] id_from, id_to Node IDs at the ends of the edge
    /// \returns the existence of the edge.
    virtual bool edgeExist(TNodeId id_from, TNodeId id_to) const { return getEdge(id_from, id_to).first; }
    /// \~japanese
    /// 迷路データに基づいてエッジのコストを返します．
    ///
    /// \param[in] id_from, id_to エッジの両端ノードのID
    /// \returns エッジのコストを返します．
    ///
    /// \~english
    /// Returns the cost of the edge according to the maze data.
    ///
    /// \param[in] id_from, id_to Node IDs at the ends of the edge
    /// \returns the cost of the edge.
    virtual TCost edgeCost(TNodeId id_from, TNodeId id_to) const { return getEdge(id_from, id_to).second; }

    /// \~japanese
    /// エージェントの状態から対応するノードのIDを計算します．
    ///
    /// \param[in] as エージェントの状態
    /// \returns ノードのIDを返します．
    ///
    /// \~english
    /// Calculates the node ID corresponding to the agent state.
    ///
    /// \param[in] as Agent state
    /// \returns the node ID.
    virtual TNodeId nodeIdByAgentState(AgentState as) const = 0;
    /// \~japanese
    /// 位置に対応するノードのIDのリストを計算します．
    ///
    /// \param[in] p 位置
    /// \param[out] ids ノードのIDのリストが格納されます
    ///
    /// \~english
    /// Calculates the node ID corresponding to the position.
    ///
    /// \param[in] p Position
    /// \param[out] ids List of node ID.
    virtual std::vector<NodeId> nodeIdsByPosition(Position p) const = 0;
    /// \~japanese
    /// ノードIDから対応するエージェントの状態を計算します．
    ///
    /// ノードIDのみから方向や\p attribute が一意に定まらない場合は，方向を \p Amaze::kNoDirection に，\p attribute を0に設定します．
    ///
    /// \param[in] id ノードID
    /// \returns エージェントの状態を返します．
    ///
    /// \~english
    /// Calculates the agent state corresponding to the node ID.
    ///
    /// The direction is set to \p Amaze::kNoDirection if it is not uniquely determined by the node ID. \p attribute is also default to 0.
    ///
    /// \param[in] id Node ID
    /// \returns the agent state.
    virtual AgentState agentStateByNodeId(TNodeId id) const = 0;
    /// \~japanese
    /// \p id_from から \p id_to へのエッジから，終点に対応するエージェントの状態を計算します．
    ///
    /// \param[in] id_from, id_to ノードID
    /// \returns エージェントの状態を返します．
    ///
    /// \~english
    /// Calculates the agent state corresponding to the terminal node of the edge from \p id_from to \p id_to.
    ///
    /// \param[in] id_from, id_to Node ID
    /// \returns the agent state.
    virtual AgentState agentStateByEdge(TNodeId id_from, TNodeId id_to) const = 0;

    /// \~japanese
    /// 与えられたエージェントの状態から次に観測する壁の位置を返します．
    ///
    /// \param[in] as エージェントの状態
    /// \returns 位置を格納した \p std::vector
    ///
    /// \~english
    /// Returns positions of walls to sense next for a given agent state.
    ///
    /// \param[in] as Agent state
    /// \returns \p std::vector contains positions
    virtual std::vector<Position> nextWalls(AgentState as) const = 0;

    /// \~japanese
    /// 与えられたノードIDの順列が引き返しを行う動作かどうかを判別します．
    ///
    /// \param[in] seq ノードIDの配列
    /// \returns 引き返しを行う場合 \p true を返します．
    ///
    /// \~english
    /// Determines whether the sequence represents pull-back action.
    ///
    /// \param[in] seq Sequence of node IDs
    /// \returns \p true if the sequence represents pull-back.
    virtual bool isPullBackSequence(std::array<TNodeId, 3> seq) const = 0;

    /// \~japanese
    /// スタートのノードIDを返します．
    ///
    /// \returns スタートのノードIDを返します．
    ///
    /// \~english
    /// Returns the start node ID.
    ///
    /// \returns the start node ID.
    virtual TNodeId getStartNodeId() const { return nodeIdByAgentState({ maze.getStart(), kNoDirection, 0 }); }
    /// \~japanese
    /// ゴールのノードIDのリストを返します．
    ///
    /// \param[out] ids ゴールのノードIDのリスト
    ///
    /// \~english
    /// Returns the list of goal node IDs.
    ///
    /// \param[out] ids List of goal node IDs
    virtual std::vector<TNodeId> getGoalNodeIds() const
    {
        std::vector<TNodeId> ids;
        std::vector<Position> positions = maze.getGoals();
        for (auto p : positions) {
            TNodeId id = nodeIdByAgentState({ p, kNoDirection, 0 });
            if (id != kInvalidNode) {
                ids.push_back(id);
            }
        }
        return ids;
    }

    /// \~japanese
    /// \p maze の const 参照を返します．
    ///
    /// \returns \p maze の const 参照を返します．
    ///
    /// \~english
    /// Returns the const reference of \p maze.
    ///
    /// \returns the const reference of \p maze.
    const Maze<W>& getCMaze() const { return maze; }
};

} // namespace Amaze
