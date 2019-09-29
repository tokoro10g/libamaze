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
template <bool kExplore = true, typename TCost = uint16_t, typename TNodeId = uint16_t, uint8_t W = kDefaultMazeWidth>
class FourWayStepMapGraph : public MazeGraph<kExplore, TCost, TNodeId, W> {
public:
    using Base = MazeGraph<kExplore, TCost, TNodeId, W>;
    using Base::getEdge;
    using Base::getEdgeWithHypothesis;

    /// \~japanese グラフのサイズ
    /// \~english Cardinality of the graph
    static constexpr TNodeId kSize = W * W;

    explicit FourWayStepMapGraph(const Maze<W>& maze)
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
        return TCost(abs((int)as1.pos.x - as2.pos.x) / 2 + abs((int)as1.pos.y - as2.pos.y) / 2);
    }
    std::vector<std::pair<TNodeId, TCost>> neighborEdges(TNodeId id) const
    {
        std::vector<std::pair<TNodeId, TCost>> v;
        if (id >= kSize) {
            // TODO: implement exception handling
            std::cerr << "Out of bounds!!! (id: " << (int)id << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return v; // should be empty
        }
        std::array<int8_t, 4> diff { { -1, 1, W, -W } };
        for (auto d : diff) {
            if (id < -d || id + d >= kSize) {
                continue;
            }
            std::pair<bool, TNodeId> edge = getEdge(id, TNodeId(id + d));
            if (id + d >= 0 && id + d < kSize && edge.first) {
                v.push_back({ TNodeId(id + d), edge.second });
            }
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
        if ((abs((int)as1.pos.x - as2.pos.x) == 2 && abs((int)as1.pos.y - as2.pos.y) == 0) || (abs((int)as1.pos.y - as2.pos.y) == 2 && abs((int)as1.pos.x - as2.pos.x) == 0)) {
            return { true, std::max(TCost(1), maxcost) };
        }
        return { false, Base::kInf };
    }
    std::pair<bool, TCost> getEdge(AgentState as1, AgentState as2) const
    {
        if (as1 == kInvalidAgentState || as2 == kInvalidAgentState) {
            std::cerr << "Out of bounds!!! (from: " << as1 << ", to: " << as2 << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return { false, Base::kInf };
        }
        Difference d = as2.pos - as1.pos;
        d.x /= 2;
        d.y /= 2;
        Position p = as1.pos + d;
        if (!kExplore && !Base::maze.isCheckedWall(p)) {
            return { false, Base::kInf };
        }
        return getEdgeWithHypothesis(as1, as2, Base::maze.isSetWall(p));
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
    std::vector<TNodeId> nodeIdsByPosition(Position p) const
    {
        std::vector<TNodeId> ids;
        ids.push_back(nodeIdByAgentState({ p, kNoDirection, 0 }));
        return ids;
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
    std::vector<Position> nextWalls(AgentState as) const
    {
        std::vector<Position> positions;
        if(as.dir == kNorth){
            positions.push_back(as.pos + Difference{0, 3});
            positions.push_back(as.pos + Difference{1, 2});
            positions.push_back(as.pos + Difference{-1, 2});
        } else if(as.dir == kEast){
            positions.push_back(as.pos + Difference{3, 0});
            positions.push_back(as.pos + Difference{2, 1});
            positions.push_back(as.pos + Difference{2, -1});
        } else if(as.dir == kSouth){
            positions.push_back(as.pos + Difference{0, -3});
            positions.push_back(as.pos + Difference{1, -2});
            positions.push_back(as.pos + Difference{-1, -2});
        } else if(as.dir == kWest){
            positions.push_back(as.pos + Difference{-3, 0});
            positions.push_back(as.pos + Difference{-2, 1});
            positions.push_back(as.pos + Difference{-2, -1});
        } else {
            return std::vector<Position>();
        }
        return positions;
    }

    bool isPullBackSequence(std::array<TNodeId, 3> seq) const { return (seq[0] == seq[2] && Base::edgeExist(seq[0], seq[1])); }
};

} // namespace Amaze
