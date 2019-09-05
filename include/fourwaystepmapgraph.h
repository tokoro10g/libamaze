#pragma once

#include "mazegraph.h"

namespace Amaze {

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
}