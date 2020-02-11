#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "gmock/gmock-matchers.h"
#include <iostream>
#include <limits>
#include <tuple>

#include "cases_maze.h"
#include "common.h"
#include "fourwaystepmapgraph.h"
#include "mazeutility.h"

using namespace Amaze;
using namespace Amaze::Utility;
using namespace testing;

class FourWayGraphUnitTest : public testing::TestWithParam<std::tuple<Position, unsigned int>> {
protected:
    FourWayGraphUnitTest()
        : maze()
        , mg(maze)
    {
    }
    Maze<16> maze;
    FourWayStepMapGraph<true, uint16_t, uint16_t, 16> mg;
};

TEST(FourWayGraphUnitTest, ConvertsAgentStateAndNodeId16)
{
    Maze<16> maze;
    FourWayStepMapGraph mg(maze);

    EXPECT_EQ(decltype(mg)::kInvalidNode, mg.nodeIdByAgentState({ { 1, 0 }, kNoDirection, 0 }));
    EXPECT_EQ(0, mg.nodeIdByAgentState({ { 0, 0 }, kNoDirection, 0 }));
    EXPECT_EQ(1, mg.nodeIdByAgentState({ { 2, 0 }, kNoDirection, 0 }));
    EXPECT_EQ(16, mg.nodeIdByAgentState({ { 0, 2 }, kNoDirection, 0 }));
    EXPECT_EQ(decltype(mg)::kInvalidNode, mg.nodeIdByAgentState({ { 180, 180 }, kNoDirection, 0 }));

    auto as0 = AgentState { { 0, 0 }, kNoDirection, 0 };
    EXPECT_EQ(as0, mg.agentStateByNodeId(0));
    auto as1 = AgentState { { 2, 0 }, kNoDirection, 0 };
    EXPECT_EQ(as1, mg.agentStateByNodeId(1));
    auto as16 = AgentState { { 0, 2 }, kNoDirection, 0 };
    EXPECT_EQ(as16, mg.agentStateByNodeId(16));

    ASSERT_THAT(mg.nodeIdsByPosition(Position { 0, 0 }), ElementsAre(0));
    ASSERT_THAT(mg.nodeIdsByPosition(Position { 2, 0 }), ElementsAre(1));
    ASSERT_THAT(mg.nodeIdsByPosition(Position { 0, 2 }), ElementsAre(16));

    auto as0to1 = AgentState { { 2, 0 }, kEast, 0 };
    EXPECT_EQ(as0to1, mg.agentStateByEdge(0, 1));
    auto as1to0 = AgentState { { 0, 0 }, kWest, 0 };
    EXPECT_EQ(as1to0, mg.agentStateByEdge(1, 0));
    auto as0to16 = AgentState { { 0, 2 }, kNorth, 0 };
    EXPECT_EQ(as0to16, mg.agentStateByEdge(0, 16));
    auto as16to0 = AgentState { { 0, 0 }, kSouth, 0 };
    EXPECT_EQ(as16to0, mg.agentStateByEdge(16, 0));
    EXPECT_EQ(kInvalidAgentState, mg.agentStateByEdge(0, 2));
}

TEST(FourWayGraphUnitTest, ConvertsAgentStateAndNodeId32)
{
    Maze<32> maze;
    FourWayStepMapGraph mg(maze);

    EXPECT_EQ(decltype(mg)::kInvalidNode, mg.nodeIdByAgentState({ { 1, 0 }, kNoDirection, 0 }));
    EXPECT_EQ(0, mg.nodeIdByAgentState({ { 0, 0 }, kNoDirection, 0 }));
    EXPECT_EQ(1, mg.nodeIdByAgentState({ { 2, 0 }, kNoDirection, 0 }));
    EXPECT_EQ(32, mg.nodeIdByAgentState({ { 0, 2 }, kNoDirection, 0 }));

    auto as0 = AgentState { { 0, 0 }, kNoDirection, 0 };
    EXPECT_EQ(as0, mg.agentStateByNodeId(0));
    auto as1 = AgentState { { 2, 0 }, kNoDirection, 0 };
    EXPECT_EQ(as1, mg.agentStateByNodeId(1));
    auto as32 = AgentState { { 0, 2 }, kNoDirection, 0 };
    EXPECT_EQ(as32, mg.agentStateByNodeId(32));

    ASSERT_THAT(mg.nodeIdsByPosition(Position { 0, 0 }), ElementsAre(0));
    ASSERT_THAT(mg.nodeIdsByPosition(Position { 2, 0 }), ElementsAre(1));
    ASSERT_THAT(mg.nodeIdsByPosition(Position { 0, 2 }), ElementsAre(32));
    EXPECT_EQ(0U, mg.nodeIdsByPosition(Position { 200, 200 }).size());

    auto as0to1 = AgentState { { 2, 0 }, kEast, 0 };
    EXPECT_EQ(as0to1, mg.agentStateByEdge(0, 1));
    auto as1to0 = AgentState { { 0, 0 }, kWest, 0 };
    EXPECT_EQ(as1to0, mg.agentStateByEdge(1, 0));
    auto as0to32 = AgentState { { 0, 2 }, kNorth, 0 };
    EXPECT_EQ(as0to32, mg.agentStateByEdge(0, 32));
    auto as32to0 = AgentState { { 0, 0 }, kSouth, 0 };
    EXPECT_EQ(as32to0, mg.agentStateByEdge(32, 0));
    EXPECT_EQ(kInvalidAgentState, mg.agentStateByEdge(0, 2));
    EXPECT_EQ(kInvalidAgentState, mg.agentStateByEdge(0, 16));
}

TEST(FourWayGraphUnitTest, EnumeratesEdges)
{
    Maze<32> maze;
    FourWayStepMapGraph mg(maze);

    auto as1 = AgentState { { 0, 2 }, kNoDirection, 0 };
    auto as2 = AgentState { { 0, 0 }, kNoDirection, 0 };

    EXPECT_EQ(std::make_pair(true, decltype(mg)::Cost(1)), mg.edge(as1, as2));
    EXPECT_EQ(std::make_pair(false, decltype(mg)::kInf), mg.edge(as1, kInvalidAgentState));
    EXPECT_EQ(std::make_pair(false, decltype(mg)::kInf), mg.edge(kInvalidAgentState, as1));
}

TEST(FourWayGraphUnitTest, GetsNeighborsAndCosts)
{
    std::istringstream iss(maze_str1);
    Maze<16> maze;
    ASSERT_TRUE(loadMazeFromStream(maze, iss));
    FourWayStepMapGraph mg(maze);

    using Cost = decltype(mg)::Cost;
    using NodeId = decltype(mg)::NodeId;

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

TEST_P(FourWayGraphUnitTest, CalculatesAffectedEdges)
{
    Maze<16> maze;
    FourWayStepMapGraph mg(maze);
    std::vector<Position> positions;

    positions.push_back(std::get<0>(GetParam()));
    auto edges = mg.affectedEdges(positions);
    EXPECT_EQ(std::get<1>(GetParam()), edges.size());
}

INSTANTIATE_TEST_SUITE_P(Inst1, FourWayGraphUnitTest,
    testing::Values(
        std::make_tuple(Position { 0, 0 }, 0U),
        std::make_tuple(Position { 1, 1 }, 0U),
        std::make_tuple(Position { 0, 1 }, 1U),
        std::make_tuple(Position { 1, 0 }, 1U),
        std::make_tuple(Position { 100, 101 }, 0U),
        std::make_tuple(Position { 200, 201 }, 0U)));

TEST(FourWayGraphUnitTest, pull_back)
{
    Maze<16> maze;
    FourWayStepMapGraph mg(maze);

    EXPECT_EQ(true, mg.isPullBackSequence({ { 0, 1, 0 } }));
    EXPECT_EQ(false, mg.isPullBackSequence({ { 0, 1, 2 } }));
    EXPECT_EQ(false, mg.isPullBackSequence({ { 0, 1, 16 } }));
    EXPECT_EQ(false, mg.isPullBackSequence({ { 0, 18, 0 } }));
}

GTEST_API_ int main(int argc, char** argv)
{
    printf("Running main() from %s\n", __FILE__);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
