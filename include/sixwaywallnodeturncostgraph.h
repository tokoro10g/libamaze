#pragma once

#include "sixwaywallnodegraph.h"

namespace Amaze {

/// \~japanese
/// 迷路の壁ノード6方向グラフ表現(ターン遷移コスト付き)．
///
/// このグラフ表現では壁の座標にノードを置き，隣接する壁どうしをつなぐ6方向のエッジを考えます．
///
/// \tparam TCost コストの型
/// \tparam TNodeId ノードIDの型
/// \tparam W 迷路の幅
///
/// \~english
/// A 6-way wall node graph representation of the maze (with costs of turn state transitions).
///
/// This representation considers nodes at wall coordinates and 6 edges from each node towards adjacent walls.
///
/// \tparam TCost Type of the cost
/// \tparam TNodeId Type of the node ID
/// \tparam W Maze width
template <bool kExplore = true, typename TCost = uint16_t, typename TNodeId = uint16_t, uint8_t W = kDefaultMazeWidth>
class SixWayWallNodeTurnCostGraph : public MazeGraph<kExplore, TCost, TNodeId, W> {
public:
    using Base = MazeGraph<kExplore, TCost, TNodeId, W>;
    using Base::getEdge;
    using Base::getEdgeWithHypothesis;

    /// \~japanese グラフのサイズ
    /// \~english Cardinality of the graph
    static constexpr TNodeId kSize = 2 * 2 * W * (W - 1);

    explicit SixWayWallNodeTurnCostGraph(const Maze<W>& maze)
        : Base(maze)
    {
    }
    TCost distance(TNodeId id_from, TNodeId id_to) const
    {
        if (id_from >= kSize || id_to >= kSize) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! (id_from: " << int(id_from) << ", id_to: " << int(id_to) << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return Base::kInf;
        }
        if (id_from == id_to) {
            return 0;
        }
        AgentState as1 = agentStateByNodeId(id_from);
        AgentState as2 = agentStateByNodeId(id_to);
        // FIXME: may contain miscalculaions
        return TCost(abs(int(as1.pos.x) - as2.pos.x) + abs(int(as1.pos.y) - as2.pos.y) + (as1.attribute != as2.attribute));
    }

    std::vector<std::pair<TNodeId, TCost>> neighborEdges(TNodeId id) const
    {
        std::vector<std::pair<TNodeId, TCost>> v;
        if (id >= kSize) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! (id: " << int(id) << __FILE__ << ":" << __LINE__ << std::endl;
            return v; // should be empty
        }
        constexpr int8_t dx[8] = { 0, 1, 2, 1, 0, -1, -2, -1 };
        constexpr int8_t dy[8] = { 2, 1, 0, -1, -2, -1, 0, 1 };
        AgentState as = agentStateByNodeId(id);
        for (int i = 0; i < 8; i++) {
            if (as.pos.x == 0 && dx[i] < 0)
                continue;
            if (as.pos.y == 0 && dy[i] < 0)
                continue;
            if (as.pos.x + dx[i] > 2 * (W - 1))
                continue;
            if (as.pos.y + dy[i] > 2 * (W - 1))
                continue;
            AgentState as_tmp = as;
            as_tmp.pos.x = uint8_t(as_tmp.pos.x + dx[i]);
            as_tmp.pos.y = uint8_t(as_tmp.pos.y + dy[i]);
            std::pair<bool, TNodeId> edge = getEdge(as, as_tmp);
            if (edge.first) {
                v.push_back({ nodeIdByAgentState(as_tmp), edge.second });
            }
        }
        AgentState as_tmp = as;
        as_tmp.attribute ^= 0x1;
        std::pair<bool, TNodeId> edge = getEdge(as, as_tmp);
        if (edge.first) {
            v.push_back({ nodeIdByAgentState(as_tmp), edge.second });
        }
        return v;
    }

    std::pair<bool, TCost> getEdgeWithHypothesis(AgentState as1, AgentState as2, bool blocked) const
    {
        if (as1 == kInvalidAgentState || as2 == kInvalidAgentState) {
            std::cerr << "Out of bounds!!! (from: " << as1 << ", to: " << as2 << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return { false, Base::kInf };
        }

        TCost maxcost = 0;
        if (blocked) {
            maxcost = Base::kInf;
        }

        // FIXME: may contain bugs
        if (as1.attribute & 0x1) {
            if (abs(int(as1.pos.x) - as2.pos.x) == 1 && abs(int(as1.pos.y) - as2.pos.y) == 1 && as1.pos.x % 2 != as1.pos.y % 2 && (as2.attribute & 0x1)) {
                return { true, std::max(TCost(2), maxcost) };
            }
            if (as1.pos.x == as2.pos.x && as1.pos.y == as2.pos.y && !(as2.attribute & 0x1)) {
                return { true, std::max(TCost(1), maxcost) };
            }
        } else {
            if ((abs(int(as1.pos.x) - as2.pos.x) == 2 && as1.pos.y == as2.pos.y && as1.pos.x % 2 == 1 && as1.pos.y % 2 == 0) || (abs(int(as1.pos.y) - as2.pos.y) == 2 && as1.pos.x == as2.pos.x && as1.pos.x % 2 == 0 && as1.pos.y % 2 == 1)) {
                if (!(as2.attribute & 0x1)) {
                    return { true, std::max(TCost(2), maxcost) };
                }
            }
            if (as1.pos.x == as2.pos.x && as1.pos.y == as2.pos.y && (as2.attribute & 0x1)) {
                return { true, std::max(TCost(1), maxcost) };
            }
        }
        return { false, Base::kInf };
    }
    std::pair<bool, TCost> getEdge(AgentState as1, AgentState as2) const
    {
        if (as1 == kInvalidAgentState || as2 == kInvalidAgentState) {
            std::cerr << "Out of bounds!!! (from: " << as1 << ", to: " << as2 << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return { false, Base::kInf };
        }
        if (!kExplore && (!Base::maze.isCheckedWall(as1.pos) || !Base::maze.isCheckedWall(as2.pos))) {
            return { false, Base::kInf };
        }
        return getEdgeWithHypothesis(as1, as2, Base::maze.isSetWall(as1.pos) || Base::maze.isSetWall(as2.pos));
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
            return TNodeId(as.pos.y / 2 * (2 * W - 1) + as.pos.x / 2 + (as.attribute & 0x1) * kSize / 2);
        } else {
            // North node
            return TNodeId(as.pos.y / 2 * (2 * W - 1) + as.pos.x / 2 + W - 1 + (as.attribute & 0x1) * kSize / 2);
        }
    }
    std::vector<TNodeId> nodeIdsByPosition(Position p) const
    {
        std::vector<TNodeId> ids;
        ids.push_back(nodeIdByAgentState({ p, kNoDirection, 0 }));
        ids.push_back(nodeIdByAgentState({ p, kNoDirection, 1 }));
        return ids;
    }
    AgentState agentStateByNodeId(TNodeId id) const
    {
        // FIXME: may contain bugs
        if (id >= kSize) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! (id: " << int(id) << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return kInvalidAgentState;
        }
        AgentState as;
        as.attribute = id >= kSize / 2;
        if (id >= kSize / 2) {
            id = TNodeId(id - kSize / 2);
        }
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
        ret.attribute = id_to >= kSize / 2;
        return ret;
    }

    bool isPullBackSequence(std::array<TNodeId, 3> seq) const
    {
        if (!Base::edgeExist(seq[0], seq[1]) || !Base::edgeExist(seq[1], seq[2])) {
            return false;
        }
        return (seq[0] == seq[2] || Base::edgeExist(seq[0], seq[2]));
    }

    std::vector<Position> nextWalls(AgentState as) const
    {
        std::vector<Position> positions;
        if(as.dir == kNorth){
            positions.push_back(as.pos + Difference{0, 2});
            positions.push_back(as.pos + Difference{1, 1});
            positions.push_back(as.pos + Difference{-1, 1});
        } else if(as.dir == kEast){
            positions.push_back(as.pos + Difference{2, 0});
            positions.push_back(as.pos + Difference{1, 1});
            positions.push_back(as.pos + Difference{1, -1});
        } else if(as.dir == kSouth){
            positions.push_back(as.pos + Difference{0, -2});
            positions.push_back(as.pos + Difference{1, -1});
            positions.push_back(as.pos + Difference{-1, -1});
        } else if(as.dir == kWest){
            positions.push_back(as.pos + Difference{-2, 0});
            positions.push_back(as.pos + Difference{-1, 1});
            positions.push_back(as.pos + Difference{-1, -1});
        } else {
            return std::vector<Position>();
        }
        return positions;
    }

    TNodeId getStartNodeId() const
    {
        Position p = Base::maze.getStart();
        if (p.x % 2 == 0 && p.y % 2 == 0) {
            p.y++;
        }
        return nodeIdByAgentState({ p, kNoDirection, 0 });
    }
    std::vector<TNodeId> getGoalNodeIds() const
    {
        std::vector<TNodeId> ids;
        std::vector<Position> positions = Base::maze.getGoals();
        for (auto p : positions) {
            TNodeId id = nodeIdByAgentState({ p, kNoDirection, 0 });
            if (id != Base::kInvalidNode) {
                ids.push_back(id);
            }
            id = nodeIdByAgentState({ p, kNoDirection, 1 });
            if (id != Base::kInvalidNode) {
                ids.push_back(id);
            }
        }
        return ids;
    }
};

} // namespace Amaze
