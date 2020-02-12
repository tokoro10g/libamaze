#pragma once

#include "mazegraph.h"

namespace Amaze {

/// \~japanese
/// 迷路の4方向歩数マップによるグラフ表現．
///
/// このグラフ表現では区画の中心にノードを置き，区画どうしを結ぶ4方向のエッジを考えます．
///
/// \tparam kExplore 探索モードのとき \p true
/// \tparam TCost コストの型
/// \tparam TNodeId ノードIDの型
/// \tparam W 迷路の幅
/// \tparam NodeCount ノードの個数 = W * W (変更不可)
///
/// \~english
/// A 4-way step map graph representation of the maze.
///
/// This representation considers nodes at the center of cells and 4 edges from each node towards the adjacent cells.
///
/// \tparam kExplore \p true if exploration mode
/// \tparam TCost Type of the cost
/// \tparam TNodeId Type of the node ID
/// \tparam W Maze width
/// \tparam NodeCount The number of nodes = W * W (unmodifiable)
template <bool kExplore = true, typename TCost = uint16_t, typename TNodeId = uint16_t, uint8_t W = kDefaultMazeWidth, TNodeId NodeCount = TNodeId(W* W)>
class FourWayStepMapGraph : public MazeGraph<TCost, TNodeId, W, NodeCount> {
    static_assert(NodeCount == TNodeId(W * W), "The template parameter NodeCount has invalid value.");

public:
    using Base = MazeGraph<TCost, TNodeId, W, NodeCount>;
    using Base::edge;
    using Base::edgeWithHypothesis;

    static constexpr TNodeId kSize = NodeCount;

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
            std::pair<bool, TNodeId> e = edge(id, TNodeId(id + d));
            if (id + d >= 0 && id + d < kSize && e.first) {
                v.push_back({ TNodeId(id + d), e.second });
            }
        }
        return v;
    }
    virtual std::vector<std::tuple<TNodeId, TNodeId, TCost>> affectedEdges(const std::vector<Position>& positions) const
    {
        std::vector<std::tuple<TNodeId, TNodeId, TCost>> edges;
        for (auto p : positions) {
            if (p.x % 2 == p.y % 2) {
                // cell or pillar
                continue;
            }
            if (p.x % 2 == 0 && p.y % 2 == 1) {
                TNodeId id_from = nodeIdByAgentState({ p + Difference { 0, -1 }, kNoDirection, 0 });
                if (id_from == Base::kInvalidNode) {
                    continue;
                }
                auto e = edge(id_from, TNodeId(id_from + W));
                if (e.first) {
                    edges.push_back({ id_from, TNodeId(id_from + W), e.second });
                }

            } else /* p.x % 2 == 1 && p.y % 2 == 0 */ {
                TNodeId id_from = nodeIdByAgentState({ p + Difference { -1, 0 }, kNoDirection, 0 });
                if (id_from == Base::kInvalidNode) {
                    continue;
                }
                auto e = edge(id_from, TNodeId(id_from + 1));
                if (e.first) {
                    edges.push_back({ id_from, TNodeId(id_from + 1), e.second });
                }
            }
        }
        return edges;
    }

    std::pair<bool, TCost> edgeWithHypothesis(AgentState as1, AgentState as2, bool blocked) const
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
    std::pair<bool, TCost> edge(AgentState as1, AgentState as2) const
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
        return edgeWithHypothesis(as1, as2, Base::maze.isSetWall(p));
    }
    TNodeId nodeIdByAgentState(AgentState as) const
    {
        if (as.pos.x % 2 != 0 || as.pos.y % 2 != 0 || as.pos.x > 2 * W || as.pos.y > 2 * W) {
            // wall, pillar, or out of range
            //std::cerr << "Invalid state!!! (pos: " << (int)as.pos.x << ", " << (int)as.pos.y << ") " << __FILE__ << ":" << __LINE__ << std::endl;
            return Base::kInvalidNode;
        }
        return TNodeId(as.pos.x / 2 + as.pos.y / 2 * W);
    }
    std::vector<TNodeId> nodeIdsByPosition(Position p) const
    {
        std::vector<TNodeId> ids;
        TNodeId id = nodeIdByAgentState({ p, kNoDirection, 0 });
        if (id != Base::kInvalidNode) {
            ids.push_back(id);
        }
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

    bool isPullBackSequence(std::array<TNodeId, 3> seq) const { return (seq[0] == seq[2] && Base::edgeExist(seq[0], seq[1])); }
};

} // namespace Amaze
