#pragma once

#include "mazegraph.h"

namespace Amaze {

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
    using Base::getEdgeWithHypothesis;

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
        return TCost(abs((int)as1.pos.x - as2.pos.x) + abs((int)as1.pos.y - as2.pos.y));
    }
    void neighborEdges(TNodeId id, std::vector<std::pair<TNodeId, TCost>>& v) const
    {
        if (id >= kSize) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! (id: " << (int)id << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return;
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
        if (abs((int)as1.pos.x - as2.pos.x) == 1 && abs((int)as1.pos.y - as2.pos.y) == 1 && as1.pos.x % 2 != as1.pos.y % 2) {
            return { true, std::max(TCost(2), maxcost) };
        } else if ((abs((int)as1.pos.x - as2.pos.x) == 2 && as1.pos.y == as2.pos.y && as1.pos.x % 2 == 1 && as1.pos.y % 2 == 0) || (abs((int)as1.pos.y - as2.pos.y) == 2 && as1.pos.x == as2.pos.x && as1.pos.x % 2 == 0 && as1.pos.y % 2 == 1)) {
            return { true, std::max(TCost(3), maxcost) };
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
