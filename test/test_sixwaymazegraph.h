#include "gmock/gmock-matchers.h"
#include "gmock/gmock.h"
#include "gtest/gtest-typed-test.h"
#include "gtest/gtest.h"
#include <iostream>
#include <limits>
#include <tuple>

#include "cases_maze.h"
#include "common.h"
#include "mazeutility.h"

#include "fixtures.h"

#include "sixwaywallnodegraph.h"
#include "sixwaywallnodeturncostgraph.h"

using namespace Amaze;
using namespace Amaze::Utility;

template <typename ConcreteMaze>
using SixWayGraphFixture = AbstractMazeGraphFixture<SixWayWallNodeGraph<true, uint16_t, uint16_t, ConcreteMaze::kWidth>, ConcreteMaze>;
template <typename ConcreteMaze>
using SixWayTCGraphFixture = AbstractMazeGraphFixture<SixWayWallNodeTurnCostGraph<true, uint16_t, uint16_t, ConcreteMaze::kWidth>, ConcreteMaze>;

template <typename ConcreteMaze>
class SixWayGraphTypedTest : public SixWayGraphFixture<ConcreteMaze>, public ::testing::Test {
};
using SixWayGraphTest = SixWayGraphTypedTest<Maze<16>>;
template <typename ConcreteMaze>
class SixWayTCGraphTypedTest : public SixWayTCGraphFixture<ConcreteMaze>, public ::testing::Test {
};
using SixWayTCGraphTest = SixWayTCGraphTypedTest<Maze<16>>;

class SixWayGraphAffectedEdgesTest : public SixWayGraphFixture<Maze<16>>, public ::testing::TestWithParam<std::tuple<Position, unsigned int>> {
};
class SixWayGraphWallOnEdgeTest : public SixWayGraphFixture<Maze<16>>, public ::testing::TestWithParam<std::tuple<uint16_t, uint16_t, Position>> {
};
class SixWayTCGraphAffectedEdgesTest : public SixWayTCGraphFixture<Maze<16>>, public ::testing::TestWithParam<std::tuple<Position, unsigned int>> {
};
class SixWayTCGraphWallOnEdgeTest : public SixWayTCGraphFixture<Maze<16>>, public ::testing::TestWithParam<std::tuple<uint16_t, uint16_t, Position>> {
};

TYPED_TEST_SUITE_P(SixWayGraphTypedTest);
TYPED_TEST_SUITE_P(SixWayTCGraphTypedTest);

TYPED_TEST_P(SixWayGraphTypedTest, ConvertsAgentStateAndNodeId)
{
    using ::testing::ElementsAre;
    using TMazeGraph = typename decltype(this->mg)::Base;

    EXPECT_EQ(TMazeGraph::kInvalidNode, this->mg.nodeIdByAgentState({ { 0, 0 }, kNoDirection, 0 }));
    EXPECT_EQ(0, this->mg.nodeIdByAgentState({ { 1, 0 }, kNoDirection, 0 }));
    EXPECT_EQ(1, this->mg.nodeIdByAgentState({ { 3, 0 }, kNoDirection, 0 }));
    EXPECT_EQ(TMazeGraph::kWidth - 1, this->mg.nodeIdByAgentState({ { 0, 1 }, kNoDirection, 0 }));

    EXPECT_EQ(AgentState({ { 1, 0 }, kNoDirection, 0 }), this->mg.agentStateByNodeId(0));
    EXPECT_EQ(AgentState({ { 3, 0 }, kNoDirection, 0 }), this->mg.agentStateByNodeId(1));
    EXPECT_EQ(AgentState({ { 0, 1 }, kNoDirection, 0 }), this->mg.agentStateByNodeId(TMazeGraph::kWidth - 1));
    EXPECT_EQ(kInvalidAgentState, this->mg.agentStateByNodeId(2 * TMazeGraph::kWidth * (TMazeGraph::kWidth - 1)));

    EXPECT_THAT(this->mg.nodeIdsByPosition(Position { 1, 0 }), ElementsAre(0));
    EXPECT_THAT(this->mg.nodeIdsByPosition(Position { 3, 0 }), ElementsAre(1));
    EXPECT_THAT(this->mg.nodeIdsByPosition(Position { 0, 1 }), ElementsAre(TMazeGraph::kWidth - 1));
    EXPECT_EQ(0U, this->mg.nodeIdsByPosition(Position { 200, 200 }).size());

    EXPECT_EQ(AgentState({ { 3, 0 }, kEast, 0 }), this->mg.agentStateByEdge(0, 1));
    EXPECT_EQ(AgentState({ { 1, 0 }, kWest, 0 }), this->mg.agentStateByEdge(1, 0));
    EXPECT_EQ(AgentState({ { 0, 1 }, kNorth, 0 }), this->mg.agentStateByEdge(0, TMazeGraph::kWidth - 1));
    EXPECT_EQ(AgentState({ { 1, 0 }, kEast, 0 }), this->mg.agentStateByEdge(TMazeGraph::kWidth - 1, 0));
    EXPECT_EQ(kInvalidAgentState, this->mg.agentStateByEdge(0, 2));
    EXPECT_EQ(kInvalidAgentState, this->mg.agentStateByEdge(0, TMazeGraph::kWidth / 2));
}

TYPED_TEST_P(SixWayTCGraphTypedTest, ConvertsAgentStateAndNodeId)
{
    using ::testing::ElementsAre;
    using TMazeGraph = typename decltype(this->mg)::Base;

    auto n = 2 * TMazeGraph::kWidth * (TMazeGraph::kWidth - 1);

    EXPECT_EQ(TMazeGraph::kInvalidNode, this->mg.nodeIdByAgentState({ { 0, 0 }, kNoDirection, 0 }));
    EXPECT_EQ(0, this->mg.nodeIdByAgentState({ { 1, 0 }, kNoDirection, 0 }));
    EXPECT_EQ(1, this->mg.nodeIdByAgentState({ { 3, 0 }, kNoDirection, 0 }));
    EXPECT_EQ(TMazeGraph::kWidth - 1, this->mg.nodeIdByAgentState({ { 0, 1 }, kNoDirection, 0 }));

    EXPECT_EQ(AgentState({ { 1, 0 }, kNoDirection, 0 }), this->mg.agentStateByNodeId(0));
    EXPECT_EQ(AgentState({ { 3, 0 }, kNoDirection, 0 }), this->mg.agentStateByNodeId(1));
    EXPECT_EQ(AgentState({ { 0, 1 }, kNoDirection, 0 }), this->mg.agentStateByNodeId(TMazeGraph::kWidth - 1));
    EXPECT_EQ(kInvalidAgentState, this->mg.agentStateByNodeId(2 * n));

    EXPECT_THAT(this->mg.nodeIdsByPosition(Position { 1, 0 }), ElementsAre(0, n));
    EXPECT_THAT(this->mg.nodeIdsByPosition(Position { 3, 0 }), ElementsAre(1, n + 1));
    EXPECT_THAT(this->mg.nodeIdsByPosition(Position { 0, 1 }), ElementsAre(TMazeGraph::kWidth - 1, n + TMazeGraph::kWidth - 1));
    EXPECT_EQ(0U, this->mg.nodeIdsByPosition(Position { 200, 200 }).size());

    EXPECT_EQ(AgentState({ { 3, 0 }, kEast, 0 }), this->mg.agentStateByEdge(0, 1));
    EXPECT_EQ(AgentState({ { 1, 0 }, kWest, 0 }), this->mg.agentStateByEdge(1, 0));
    EXPECT_EQ(AgentState({ { 1, 0 }, kNoDirection, 1 }), this->mg.agentStateByEdge(0, n));
    EXPECT_EQ(AgentState({ { 1, 0 }, kNoDirection, 0 }), this->mg.agentStateByEdge(n, 0));
    EXPECT_EQ(AgentState({ { 2, 1 }, kNorth, 1 }), this->mg.agentStateByEdge(n, n + TMazeGraph::kWidth));
    EXPECT_EQ(AgentState({ { 1, 0 }, kWest, 1 }), this->mg.agentStateByEdge(n + TMazeGraph::kWidth, n));
    EXPECT_EQ(kInvalidAgentState, this->mg.agentStateByEdge(0, TMazeGraph::kWidth));
    EXPECT_EQ(kInvalidAgentState, this->mg.agentStateByEdge(TMazeGraph::kWidth, 0));
    EXPECT_EQ(kInvalidAgentState, this->mg.agentStateByEdge(n, n + 1));
    EXPECT_EQ(kInvalidAgentState, this->mg.agentStateByEdge(n + 1, n));
    EXPECT_EQ(kInvalidAgentState, this->mg.agentStateByEdge(0, TMazeGraph::kWidth - 1));
    EXPECT_EQ(kInvalidAgentState, this->mg.agentStateByEdge(TMazeGraph::kWidth - 1, 0));
    EXPECT_EQ(kInvalidAgentState, this->mg.agentStateByEdge(0, 2));
    EXPECT_EQ(kInvalidAgentState, this->mg.agentStateByEdge(0, TMazeGraph::kWidth / 2));
}

REGISTER_TYPED_TEST_SUITE_P(SixWayGraphTypedTest, ConvertsAgentStateAndNodeId);
REGISTER_TYPED_TEST_SUITE_P(SixWayTCGraphTypedTest, ConvertsAgentStateAndNodeId);
INSTANTIATE_TYPED_TEST_SUITE_P(VariousSize, SixWayGraphTypedTest, MazeTypes);
INSTANTIATE_TYPED_TEST_SUITE_P(VariousSize, SixWayTCGraphTypedTest, MazeTypes);

// TODO: write remaining tests
