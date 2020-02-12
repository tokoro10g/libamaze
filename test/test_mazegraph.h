#include "gmock/gmock-matchers.h"
#include "gmock/gmock.h"
#include "gtest/gtest-typed-test.h"
#include "gtest/gtest.h"
#include <iostream>
#include <limits>
#include <tuple>

#include "cases_maze.h"
#include "common.h"
#include "fourwaystepmapgraph.h"
#include "mazeutility.h"

using namespace Amaze;
using namespace Amaze::Utility;

class FourWayGraphUnitTest : public ::testing::TestWithParam<std::tuple<Position, unsigned int>> {
protected:
    FourWayGraphUnitTest()
        : maze()
        , mg(maze)
    {
    }
    Maze<16> maze;
    FourWayStepMapGraph<true, uint16_t, uint16_t, 16> mg;
};

template <typename ConcreteMaze>
class FourWayGraphUnitTypedTest : public ::testing::Test {
protected:
    FourWayGraphUnitTypedTest()
        : maze()
        , mg(maze)
    {
    }
    ConcreteMaze maze;
    FourWayStepMapGraph<true, uint16_t, uint16_t, ConcreteMaze::kWidth> mg;
};

TYPED_TEST_SUITE_P(FourWayGraphUnitTypedTest);
TYPED_TEST_P(FourWayGraphUnitTypedTest, ConvertsAgentStateAndNodeId)
{
    using ::testing::ElementsAre;
    using TMazeGraph = typename decltype(this->mg)::Base;

    EXPECT_EQ(TMazeGraph::kInvalidNode, this->mg.nodeIdByAgentState({ { 1, 0 }, kNoDirection, 0 }));
    EXPECT_EQ(0, this->mg.nodeIdByAgentState({ { 0, 0 }, kNoDirection, 0 }));
    EXPECT_EQ(1, this->mg.nodeIdByAgentState({ { 2, 0 }, kNoDirection, 0 }));
    EXPECT_EQ(TMazeGraph::kWidth, this->mg.nodeIdByAgentState({ { 0, 2 }, kNoDirection, 0 }));

    EXPECT_EQ(AgentState({ { 0, 0 }, kNoDirection, 0 }), this->mg.agentStateByNodeId(0));
    EXPECT_EQ(AgentState({ { 2, 0 }, kNoDirection, 0 }), this->mg.agentStateByNodeId(1));
    EXPECT_EQ(AgentState({ { 0, 2 }, kNoDirection, 0 }), this->mg.agentStateByNodeId(TMazeGraph::kWidth));
    EXPECT_EQ(kInvalidAgentState, this->mg.agentStateByNodeId(TMazeGraph::kWidth * TMazeGraph::kWidth));

    ASSERT_THAT(this->mg.nodeIdsByPosition(Position { 0, 0 }), ElementsAre(0));
    ASSERT_THAT(this->mg.nodeIdsByPosition(Position { 2, 0 }), ElementsAre(1));
    ASSERT_THAT(this->mg.nodeIdsByPosition(Position { 0, 2 }), ElementsAre(TMazeGraph::kWidth));
    EXPECT_EQ(0U, this->mg.nodeIdsByPosition(Position { 200, 200 }).size());

    EXPECT_EQ(AgentState({ { 2, 0 }, kEast, 0 }), this->mg.agentStateByEdge(0, 1));
    EXPECT_EQ(AgentState({ { 0, 0 }, kWest, 0 }), this->mg.agentStateByEdge(1, 0));
    EXPECT_EQ(AgentState({ { 0, 2 }, kNorth, 0 }), this->mg.agentStateByEdge(0, TMazeGraph::kWidth));
    EXPECT_EQ(AgentState({ { 0, 0 }, kSouth, 0 }), this->mg.agentStateByEdge(TMazeGraph::kWidth, 0));
    EXPECT_EQ(kInvalidAgentState, this->mg.agentStateByEdge(0, 2));
    EXPECT_EQ(kInvalidAgentState, this->mg.agentStateByEdge(0, TMazeGraph::kWidth / 2));
}
REGISTER_TYPED_TEST_SUITE_P(FourWayGraphUnitTypedTest, ConvertsAgentStateAndNodeId);
typedef ::testing::Types<Maze<16>, Maze<32>, Maze<64>> MazeTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(My, FourWayGraphUnitTypedTest, MazeTypes);

TEST_F(FourWayGraphUnitTest, EnumeratesEdges)
{
    auto as1 = AgentState { { 0, 2 }, kNoDirection, 0 };
    auto as2 = AgentState { { 0, 0 }, kNoDirection, 0 };

    EXPECT_EQ(std::make_pair(true, decltype(mg)::Cost(1)), mg.edge(as1, as2));
    EXPECT_EQ(std::make_pair(true, decltype(mg)::Cost(1)), mg.edge(as2, as1));
    EXPECT_EQ(std::make_pair(false, decltype(mg)::kInf), mg.edge(as1, kInvalidAgentState));
    EXPECT_EQ(std::make_pair(false, decltype(mg)::kInf), mg.edge(kInvalidAgentState, as1));
}

TEST_F(FourWayGraphUnitTest, EnumeratesEdgesWithHypothesis)
{
    auto as1 = AgentState { { 0, 2 }, kNoDirection, 0 };
    auto as2 = AgentState { { 0, 0 }, kNoDirection, 0 };

    EXPECT_EQ(std::make_pair(true, decltype(mg)::Cost(1)), mg.edgeWithHypothesis(as1, as2, false));
    EXPECT_EQ(std::make_pair(true, decltype(mg)::Cost(1)), mg.edgeWithHypothesis(as2, as1, false));
    EXPECT_EQ(std::make_pair(true, decltype(mg)::kInf), mg.edgeWithHypothesis(as1, as2, true));
    EXPECT_EQ(std::make_pair(true, decltype(mg)::kInf), mg.edgeWithHypothesis(as2, as1, true));
    EXPECT_EQ(std::make_pair(false, decltype(mg)::kInf), mg.edgeWithHypothesis(as1, kInvalidAgentState, false));
    EXPECT_EQ(std::make_pair(false, decltype(mg)::kInf), mg.edgeWithHypothesis(kInvalidAgentState, as1, false));
    EXPECT_EQ(std::make_pair(false, decltype(mg)::kInf), mg.edgeWithHypothesis(as1, kInvalidAgentState, true));
    EXPECT_EQ(std::make_pair(false, decltype(mg)::kInf), mg.edgeWithHypothesis(kInvalidAgentState, as1, true));
}

TEST_F(FourWayGraphUnitTest, GetsNeighborsAndCosts)
{
    using ::testing::ElementsAre;
    using Cost = decltype(mg)::Cost;
    using NodeId = decltype(mg)::NodeId;

    std::istringstream iss(maze_str1);
    ASSERT_TRUE(loadMazeFromStream(maze, iss));

    EXPECT_EQ(1, mg.distance(0, 1));
    EXPECT_EQ(2, mg.distance(1, 16));
    EXPECT_EQ(mg.kInf, mg.distance(0, decltype(mg)::kInvalidNode));

    auto v1 = mg.neighbors(0);
    std::sort(v1.begin(), v1.end());
    ASSERT_THAT(v1, ElementsAre(1, 16));
    auto v1e = mg.neighborEdges(0);
    std::sort(v1e.begin(), v1e.end());
    EXPECT_EQ(v1e[0], std::make_pair(NodeId(1), mg.kInf));
    EXPECT_EQ(v1e[1], std::make_pair(NodeId(16), Cost(1)));

    auto v2 = mg.neighbors(16);
    std::sort(v2.begin(), v2.end());
    ASSERT_THAT(v2, ElementsAre(0, 17, 32));
    auto v2e = mg.neighborEdges(16);
    std::sort(v2e.begin(), v2e.end());
    EXPECT_EQ(v2e[0], std::make_pair(NodeId(0), Cost(1)));
    EXPECT_EQ(v2e[1], std::make_pair(NodeId(17), mg.kInf));
    EXPECT_EQ(v2e[2], std::make_pair(NodeId(32), Cost(1)));

    auto v3 = mg.neighbors(17);
    std::sort(v3.begin(), v3.end());
    ASSERT_THAT(v3, ElementsAre(1, 16, 18, 33));
    auto v3e = mg.neighborEdges(17);
    std::sort(v3e.begin(), v3e.end());
    EXPECT_EQ(v3e[0], std::make_pair(NodeId(1), Cost(1)));
    EXPECT_EQ(v3e[1], std::make_pair(NodeId(16), mg.kInf));
    EXPECT_EQ(v3e[2], std::make_pair(NodeId(18), mg.kInf));
    EXPECT_EQ(v3e[3], std::make_pair(NodeId(33), Cost(1)));

    auto v4e = mg.neighborEdges(32768);
    EXPECT_EQ(0U, v4e.size());
}

TEST_F(FourWayGraphUnitTest, TestPullBackSequence)
{
    EXPECT_EQ(true, mg.isPullBackSequence({ { 0, 1, 0 } }));
    EXPECT_EQ(false, mg.isPullBackSequence({ { 0, 1, 2 } }));
    EXPECT_EQ(false, mg.isPullBackSequence({ { 0, 1, 16 } }));
    EXPECT_EQ(false, mg.isPullBackSequence({ { 0, 18, 0 } }));
}

TEST_P(FourWayGraphUnitTest, CalculatesAffectedEdges)
{
    Maze<16> maze;
    FourWayStepMapGraph mg(maze);
    std::vector<Position> positions;

    positions.push_back(std::get<0>(GetParam()));
    auto edges = mg.affectedEdges(positions);
    EXPECT_EQ(std::get<1>(GetParam()), edges.size());
}

INSTANTIATE_TEST_SUITE_P(General, FourWayGraphUnitTest,
    ::testing::Values(
        std::make_tuple(Position { 0, 0 }, 0U),
        std::make_tuple(Position { 1, 1 }, 0U),
        std::make_tuple(Position { 0, 1 }, 1U),
        std::make_tuple(Position { 1, 0 }, 1U)));
INSTANTIATE_TEST_SUITE_P(Invalid, FourWayGraphUnitTest,
    ::testing::Values(
        std::make_tuple(Position { 100, 101 }, 0U),
        std::make_tuple(Position { 200, 201 }, 0U),
        std::make_tuple(Position { 201, 8 }, 0U)));
