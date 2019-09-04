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
template <bool kExplore = true, typename TCost = uint16_t, typename TNodeId = uint16_t, uint8_t W = 32>
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

    MazeGraph(const Maze<W>& maze)
        : maze(maze)
    {
    }
    virtual ~MazeGraph() {}

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
    /// \param[out] v 隣接ノードのIDを格納する\p std::vector
    ///
    /// \~english
    /// Fills \p v with neighbor IDs of the node \p id.
    ///
    /// \param[in] id Node ID of the source
    /// \param[out] v \p std::vector filled with neighbors' IDs
    virtual void neighbors(TNodeId id, std::vector<TNodeId>& v) const = 0;

    /// \~japanese
    /// 与えた位置への迷路情報の変更の影響を受けるエッジを列挙します．
    ///
    /// \param[in] positions 位置のリスト
    /// \param[out] edges エッジを格納する\p std::vector
    ///
    /// \~english
    /// Enumerates edges affected by the change to the maze data for given positions.
    ///
    /// \param[in] positions List of positions
    /// \param[out] edges \p std::vector filled with edges
    virtual void affectedEdges(const std::vector<Position>& positions, std::vector<std::pair<TNodeId, TNodeId>>& edges) const
    {
        std::vector<TNodeId> visited;
        for (Position p : positions) {
            std::vector<TNodeId> ids;
            nodeIdsByPosition(p, ids);
            for (auto id : ids) {
                if (id == kInvalidNode) {
                    continue;
                }
                if (std::find(visited.begin(), visited.end(), id) == visited.end()) {
                    visited.push_back(id);
                    std::vector<NodeId> v;
                    neighbors(id, v);
                    for (auto n : v) {
                        edges.push_back({ id, n });
                    }
                }
            }
        }
    }

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
    virtual std::pair<bool, TCost> getEdgeWithHypothesis(TNodeId id_from, TNodeId id_to, bool blocked) const = 0;
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
    virtual std::pair<bool, TCost> getEdge(TNodeId id_from, TNodeId id_to) const = 0;
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
    virtual std::pair<bool, TCost> getEdge(AgentState from, AgentState to) const
    {
        return getEdge(nodeIdByAgentState(from), nodeIdByAgentState(to));
    }
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
    virtual bool edgeExist(TNodeId id_from, TNodeId id_to) const
    {
        std::pair<bool, TCost> e = getEdge(id_from, id_to);
        return e.first;
    }
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
    virtual TCost edgeCost(TNodeId id_from, TNodeId id_to) const
    {
        std::pair<bool, TCost> e = getEdge(id_from, id_to);
        return e.second;
    }

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
    virtual void nodeIdsByPosition(Position p, std::vector<NodeId>& ids) const = 0;
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
    virtual TNodeId getStartNodeId() const
    {
        Position p = maze.getStart();
        return nodeIdByAgentState({ p, kNoDirection, 0 });
    }
    /// \~japanese
    /// ゴールのノードIDのリストを返します．
    ///
    /// \param[out] ids ゴールのノードIDのリスト
    ///
    /// \~english
    /// Returns the list of goal node IDs.
    ///
    /// \param[out] ids List of goal node IDs
    virtual void getGoalNodeIds(std::vector<TNodeId>& ids) const
    {
        std::vector<Position> positions;
        maze.getGoals(positions);
        for (auto p : positions) {
            TNodeId id = nodeIdByAgentState({ p, kNoDirection, 0 });
            if (id != kInvalidNode) {
                ids.push_back(id);
            }
        }
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
    const Maze<W>& getCMaze() const
    {
        return maze;
    }
};

/// \~japanese
/// 迷路の4方向歩数マップによるグラフ表現．
///
/// このグラフ表現では区画の中心にノードを置き，区画どうしを結ぶ4方向のエッジを考えます．
///
/// \tparam TCost コストの型
/// \tparam TNodeId ノードIDの型
/// \tparam W 迷路の幅
///
/// \~english
/// A 4-way step map graph representation of the maze.
///
/// This representation considers nodes at the center of cells and 4 edges from each node towards the adjacent cells.
///
/// \tparam TCost Type of the cost
/// \tparam TNodeId Type of the node ID
/// \tparam W Maze width
template <bool kExplore = true, typename TCost = uint16_t, typename TNodeId = uint16_t, uint8_t W = 32>
class FourWayStepMapGraph : public MazeGraph<kExplore, TCost, TNodeId, W> {
public:
    using Base = MazeGraph<kExplore, TCost, TNodeId, W>;
    using Base::getEdge;

    /// \~japanese グラフのサイズ
    /// \~english Cardinality of the graph
    static constexpr TNodeId kSize = W * W;

    FourWayStepMapGraph(const Maze<W>& maze)
        : Base(maze)
    {
    }
    TCost distance(TNodeId id_from, TNodeId id_to) const
    {
        if (id_from >= kSize || id_to >= kSize) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! (id_from: " << (int)id_from << ", id_to: " << (int)id_to << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return Base::kInf;
        }
        AgentState as1, as2;
        as1 = agentStateByNodeId(id_from);
        as2 = agentStateByNodeId(id_to);
        return TCost(std::max(abs((int)as1.pos.x - as2.pos.x) / 2, abs((int)as1.pos.y - as2.pos.y) / 2));
    }
    void neighbors(TNodeId id, std::vector<TNodeId>& v) const
    {
        if (id >= kSize) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! (id: " << (int)id << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return;
        }
        std::array<int8_t, 4> diff { { -1, 1, W, -W } };
        for (auto d : diff) {
            if (id + d >= 0 && id + d < kSize && Base::edgeExist(id, TNodeId(id + d))) {
                v.push_back(TNodeId(id + d));
            }
        }
    }
    std::pair<bool, TCost> getEdgeWithHypothesis(TNodeId id_from, TNodeId id_to, bool blocked) const
    {
        if (id_from >= kSize || id_to >= kSize) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! (id_from: " << (int)id_from << ", id_to:" << (int)id_to << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return { false, Base::kInf };
        }
        AgentState as1, as2;
        as1 = agentStateByNodeId(id_from);
        as2 = agentStateByNodeId(id_to);
        TCost maxcost = 0;
        if (blocked) {
            maxcost = Base::kInf;
        }
        if ((abs((int)as1.pos.x - as2.pos.x) == 2 && abs((int)as1.pos.y - as2.pos.y) == 0) || (abs((int)as1.pos.y - as2.pos.y) == 2 && abs((int)as1.pos.x - as2.pos.x) == 0)) {
            return { true, std::max(TCost(1), maxcost) };
        }
        return { false, Base::kInf };
    }
    std::pair<bool, TCost> getEdge(TNodeId id_from, TNodeId id_to) const
    {
        if (id_from >= kSize || id_to >= kSize) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! (id_from: " << (int)id_from << ", id_to:" << (int)id_to << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return { false, Base::kInf };
        }
        Position p;
        p.x = uint8_t((id_from % W) * 2);
        p.y = uint8_t((id_from / W) * 2);
        int diff = (int)id_to - id_from;
        switch (diff) {
        case W:
            p.y++;
            break;
        case -W:
            p.y--;
            break;
        case 1:
            p.x++;
            break;
        case -1:
            p.x--;
            break;
        default:
            // TODO: handle error
            break;
        }
        if (!kExplore && !Base::maze.isCheckedWall(p)) {
            return { false, Base::kInf };
        }
        return getEdgeWithHypothesis(id_from, id_to, Base::maze.isSetWall(p));
    }
    TNodeId nodeIdByAgentState(AgentState as) const
    {
        if (as.pos.x % 2 != 0 || as.pos.y % 2 != 0) {
            // wall or pillar
            //std::cerr << "Invalid state!!! (pos: " << (int)as.pos.x << ", " << (int)as.pos.y << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return Base::kInvalidNode;
        }
        return TNodeId(as.pos.x / 2 + as.pos.y / 2 * W);
    }
    void nodeIdsByPosition(Position p, std::vector<TNodeId>& ids) const
    {
        ids.push_back(nodeIdByAgentState({ p, kNoDirection, 0 }));
    }
    AgentState agentStateByNodeId(TNodeId id) const
    {
        if (id >= kSize) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! (id: " << (int)id << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return kInvalidAgentState;
        }
        uint8_t x = uint8_t(id % W);
        uint8_t y = uint8_t(id / W);
        return { { uint8_t(x * 2), uint8_t(y * 2) }, kNoDirection, 0 };
    }
    AgentState agentStateByEdge(TNodeId id_from, TNodeId id_to) const
    {
        if (!Base::edgeExist(id_from, id_to)) {
            return kInvalidAgentState;
        }
        AgentState ret = agentStateByNodeId(id_to);
        int diff = (int)id_to - id_from;
        switch (diff) {
        case W:
            ret.dir = kNorth;
            break;
        case -W:
            ret.dir = kSouth;
            break;
        case 1:
            ret.dir = kEast;
            break;
        case -1:
            ret.dir = kWest;
            break;
        default:
            // TODO: handle error
            break;
        }
        return ret;
    }

    bool isPullBackSequence(std::array<TNodeId, 3> seq) const
    {
        return (seq[0] == seq[2] && Base::edgeExist(seq[0], seq[1]));
    }
};

/// \~japanese
/// 迷路の壁ノード6方向グラフ表現．
///
/// このグラフ表現では壁の座標にノードを置き，隣接する壁どうしをつなぐ6方向のエッジを考えます．
///
/// \tparam TCost コストの型
/// \tparam TNodeId ノードIDの型
/// \tparam W 迷路の幅
///
/// \~english
/// A 6-way wall node graph representation of the maze.
///
/// This representation considers nodes at wall coordinates and 6 edges from each node towards adjacent walls.
///
/// \tparam TCost Type of the cost
/// \tparam TNodeId Type of the node ID
/// \tparam W Maze width
template <bool kExplore = true, typename TCost = uint16_t, typename TNodeId = uint16_t, uint8_t W = 32>
class SixWayWallNodeGraph : public MazeGraph<kExplore, TCost, TNodeId, W> {
public:
    using Base = MazeGraph<kExplore, TCost, TNodeId, W>;
    using Base::getEdge;

    /// \~japanese グラフのサイズ
    /// \~english Cardinality of the graph
    static constexpr TNodeId kSize = 2 * W * (W - 1);

    SixWayWallNodeGraph(const Maze<W>& maze)
        : Base(maze)
    {
    }
    TCost distance(TNodeId id_from, TNodeId id_to) const
    {
        if (id_from >= kSize || id_to >= kSize) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! (id_from: " << (int)id_from << ", id_to: " << (int)id_to << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return Base::kInf;
        }
        AgentState as1 = agentStateByNodeId(id_from);
        AgentState as2 = agentStateByNodeId(id_to);
        // FIXME: may contain miscalculaions
        return TCost(std::max(abs((int)as1.pos.x - as2.pos.x), abs((int)as1.pos.y - as2.pos.y)));
    }
    void neighbors(TNodeId id, std::vector<TNodeId>& v) const
    {
        if (id >= kSize) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! (id: " << (int)id << __FILE__ << ":" << __LINE__ << std::endl;
            return;
        }
        std::array<int8_t, 8> diff { { -1, 1, W, -W, W - 1, -W + 1, 2 * W - 1, -2 * W + 1 } };
        for (auto d : diff) {
            if (id + d >= 0 && id + d < kSize && Base::edgeExist(id, TNodeId(id + d))) {
                v.push_back(TNodeId(id + d));
            }
        }
    }
    std::pair<bool, TCost> getEdgeWithHypothesis(TNodeId id_from, TNodeId id_to, bool blocked) const
    {
        if (id_from >= kSize || id_to >= kSize) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! (id_from: " << (int)id_from << ", id_to: " << (int)id_to << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return { false, Base::kInf };
        }
        AgentState as1, as2;
        as1 = agentStateByNodeId(id_from);
        as2 = agentStateByNodeId(id_to);

        TCost maxcost = 0;
        if (blocked) {
            maxcost = Base::kInf;
        }

        // FIXME: may contain bugs
        if (abs((int)as1.pos.x - as2.pos.x) == 1 && abs((int)as1.pos.y - as2.pos.y) == 1 && as1.pos.x % 2 != as1.pos.y % 2) {
            return { true, std::max(TCost(2), maxcost) };
        } else if ((abs((int)as1.pos.x - as2.pos.x) == 2 && as1.pos.y == as2.pos.y && as1.pos.x % 2 == 1 && as1.pos.y % 2 == 0) || (abs((int)as1.pos.y - as2.pos.y) == 2 && as1.pos.x == as2.pos.x && as1.pos.x % 2 == 0 && as1.pos.y % 2 == 1)) {
            return { true, std::max(TCost(3), maxcost) };
        }
        return { false, Base::kInf };
    }
    std::pair<bool, TCost> getEdge(TNodeId id_from, TNodeId id_to) const
    {
        if (id_from >= kSize || id_to >= kSize) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! (id_from: " << (int)id_from << ", id_to:" << (int)id_to << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return { false, Base::kInf };
        }
        AgentState as1, as2;
        as1 = agentStateByNodeId(id_from);
        as2 = agentStateByNodeId(id_to);
        if (!kExplore && (!Base::maze.isCheckedWall(as1.pos) || !Base::maze.isCheckedWall(as2.pos))) {
            return { false, Base::kInf };
        }
        return getEdgeWithHypothesis(id_from, id_to, Base::maze.isSetWall(as1.pos) || Base::maze.isSetWall(as2.pos));
    }
    TNodeId nodeIdByAgentState(AgentState as) const
    {
        if (as.pos.x % 2 == as.pos.y % 2) {
            // cell or pillar
            return Base::kInvalidNode;
        }
        // FIXME: may contain bugs
        if (as.pos.y % 2 == 0) {
            // East node
            return TNodeId(as.pos.y / 2 * (2 * W - 1) + as.pos.x / 2);
        } else {
            // North node
            return TNodeId(as.pos.y / 2 * (2 * W - 1) + as.pos.x / 2 + W - 1);
        }
    }
    void nodeIdsByPosition(Position p, std::vector<TNodeId>& ids) const
    {
        ids.push_back(nodeIdByAgentState({ p, kNoDirection, 0 }));
    }
    AgentState agentStateByNodeId(TNodeId id) const
    {
        // FIXME: may contain bugs
        if (id >= kSize) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! (id: " << (int)id << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return kInvalidAgentState;
        }
        AgentState as;
        TNodeId tmp = TNodeId(id % (2 * W - 1));
        TNodeId tmp2 = TNodeId(id / (2 * W - 1));
        bool isNorth = tmp > W - 2;
        if (isNorth) {
            as.pos.x = uint8_t((tmp - W + 1) * 2);
            as.pos.y = uint8_t(tmp2 * 2 + 1);
        } else {
            as.pos.x = uint8_t(tmp * 2 + 1);
            as.pos.y = uint8_t(tmp2 * 2);
        }
        as.dir = kNoDirection;
        as.attribute = 0;
        return as;
    }
    AgentState agentStateByEdge(TNodeId id_from, TNodeId id_to) const
    {
        if (!Base::edgeExist(id_from, id_to)) {
            return kInvalidAgentState;
        }
        AgentState as1, as2;
        as1 = agentStateByNodeId(id_from);
        as2 = agentStateByNodeId(id_to);
        AgentState ret = as2;
        Difference d = as2.pos - as1.pos;
        if (d.x == 0 && abs(d.y) == 2) {
            if (d.y < 0) {
                ret.dir = kSouth;
            } else {
                ret.dir = kNorth;
            }
        } else if (d.y == 0 && abs(d.x) == 2) {
            if (d.x < 0) {
                ret.dir = kWest;
            } else {
                ret.dir = kEast;
            }
        } else if (as1.pos.y % 2 == 0 /* from east node */ && abs(d.x) == 1 && abs(d.y) == 1) {
            if (d.y < 0) {
                ret.dir = kSouth;
            } else {
                ret.dir = kNorth;
            }
        } else if (as1.pos.x % 2 == 0 /* from north node */ && abs(d.x) == 1 && abs(d.y) == 1) {
            if (d.x < 0) {
                ret.dir = kWest;
            } else {
                ret.dir = kEast;
            }
        }
        return ret;
    }

    bool isPullBackSequence(std::array<TNodeId, 3> seq) const
    {
        if (!Base::edgeExist(seq[0], seq[1]) || !Base::edgeExist(seq[1], seq[2])) {
            return false;
        }
        return (seq[0] == seq[2] || Base::edgeExist(seq[0], seq[2]));
    }

    TNodeId getStartNodeId() const
    {
        Position p = Base::maze.getStart();
        if (p.x % 2 == 0 && p.y % 2 == 0) {
            p.y++;
        }
        return nodeIdByAgentState({ p, kNoDirection, 0 });
    }
};

}
