/**
 * Copyright (c) 2020 Tokoro
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <iostream>
#include <limits>
#include <tuple>

#include "amaze/common/common_types.h"
#include "amaze/common/maze_utils.h"
#include "amaze/maze_graph/fourway_graph.h"
#include "gmock/gmock-matchers.h"
#include "gmock/gmock.h"
#include "gtest/gtest-typed-test.h"
#include "gtest/gtest.h"
#include "test_utils.h"

using namespace amaze;              // NOLINT(build/namespaces)
using namespace amaze::maze_graph;  // NOLINT(build/namespaces)
using namespace amaze::utils;       // NOLINT(build/namespaces)

template <typename ConcreteMaze>
using FourWayGraphFixture = AbstractMazeGraphFixture<
    FourWayGraph<true, uint16_t, uint16_t, ConcreteMaze::kWidth>, ConcreteMaze>;

template <typename ConcreteMaze>
class FourWayGraphTypedTest : public FourWayGraphFixture<ConcreteMaze>,
                              public ::testing::Test {};
using FourWayGraphTest = FourWayGraphTypedTest<Maze<16>>;

class FourWayGraphAffectedEdgesTest
    : public FourWayGraphFixture<Maze<16>>,
      public ::testing::TestWithParam<std::tuple<Position, unsigned int>> {};
class FourWayGraphWallOnEdgeTest
    : public FourWayGraphFixture<Maze<16>>,
      public ::testing::TestWithParam<
          std::tuple<uint16_t, uint16_t, Position>> {};

TYPED_TEST_SUITE_P(FourWayGraphTypedTest);

TYPED_TEST_P(FourWayGraphTypedTest, ConvertsAgentStateAndNodeId) {
  using ::testing::ElementsAre;
  using TMazeGraph = typename decltype(this->mg)::Base;

  EXPECT_EQ(TMazeGraph::kInvalidNode,
            this->mg.nodeIdByAgentState({{1, 0}, kNoDirection, 0}));
  EXPECT_EQ(0, this->mg.nodeIdByAgentState({{0, 0}, kNoDirection, 0}));
  EXPECT_EQ(1, this->mg.nodeIdByAgentState({{2, 0}, kNoDirection, 0}));
  EXPECT_EQ(TMazeGraph::kWidth,
            this->mg.nodeIdByAgentState({{0, 2}, kNoDirection, 0}));

  EXPECT_EQ(AgentState({{0, 0}, kNoDirection, 0}),
            this->mg.agentStateByNodeId(0));
  EXPECT_EQ(AgentState({{2, 0}, kNoDirection, 0}),
            this->mg.agentStateByNodeId(1));
  EXPECT_EQ(AgentState({{0, 2}, kNoDirection, 0}),
            this->mg.agentStateByNodeId(TMazeGraph::kWidth));
  EXPECT_EQ(kInvalidAgentState, this->mg.agentStateByNodeId(
                                    TMazeGraph::kWidth * TMazeGraph::kWidth));

  EXPECT_THAT(this->mg.nodeIdsByPosition(Position{0, 0}), ElementsAre(0));
  EXPECT_THAT(this->mg.nodeIdsByPosition(Position{2, 0}), ElementsAre(1));
  EXPECT_THAT(this->mg.nodeIdsByPosition(Position{0, 2}),
              ElementsAre(TMazeGraph::kWidth));
  EXPECT_EQ(0U, this->mg.nodeIdsByPosition(Position{200, 200}).size());

  EXPECT_EQ(AgentState({{2, 0}, kEast, 0}), this->mg.agentStateByEdge(0, 1));
  EXPECT_EQ(AgentState({{0, 0}, kWest, 0}), this->mg.agentStateByEdge(1, 0));
  EXPECT_EQ(AgentState({{0, 2}, kNorth, 0}),
            this->mg.agentStateByEdge(0, TMazeGraph::kWidth));
  EXPECT_EQ(AgentState({{0, 0}, kSouth, 0}),
            this->mg.agentStateByEdge(TMazeGraph::kWidth, 0));
  EXPECT_EQ(kInvalidAgentState, this->mg.agentStateByEdge(0, 2));
  EXPECT_EQ(kInvalidAgentState,
            this->mg.agentStateByEdge(0, TMazeGraph::kWidth / 2));
}

REGISTER_TYPED_TEST_SUITE_P(FourWayGraphTypedTest, ConvertsAgentStateAndNodeId);
INSTANTIATE_TYPED_TEST_SUITE_P(VariousSize, FourWayGraphTypedTest, MazeTypes);

TEST_F(FourWayGraphTest, EnumeratesEdges) {
  auto as1 = AgentState{{0, 2}, kNoDirection, 0};
  auto as2 = AgentState{{0, 0}, kNoDirection, 0};
  auto id1 = mg.nodeIdByAgentState(as1);
  auto id2 = mg.nodeIdByAgentState(as2);

  using ConcreteEdgeTo = EdgeTo<decltype(mg)::NodeId, decltype(mg)::Cost>;

  ExpectEq(ConcreteEdgeTo({id2, decltype(mg)::Cost(1)}), mg.edge(as1, as2));
  ExpectEq(ConcreteEdgeTo({id1, decltype(mg)::Cost(1)}), mg.edge(as2, as1));
  ExpectEq(ConcreteEdgeTo({decltype(mg)::kInvalidNode, decltype(mg)::kInf}),
           mg.edge(as1, kInvalidAgentState));
  ExpectEq(ConcreteEdgeTo({decltype(mg)::kInvalidNode, decltype(mg)::kInf}),
           mg.edge(kInvalidAgentState, as1));
}

TEST_F(FourWayGraphTest, EnumeratesEdgesWithHypothesis) {
  auto as1 = AgentState{{0, 2}, kNoDirection, 0};
  auto as2 = AgentState{{0, 0}, kNoDirection, 0};
  auto id1 = mg.nodeIdByAgentState(as1);
  auto id2 = mg.nodeIdByAgentState(as2);

  using ConcreteEdgeTo = EdgeTo<decltype(mg)::NodeId, decltype(mg)::Cost>;

  ExpectEq(ConcreteEdgeTo({id2, decltype(mg)::Cost(1)}),
           mg.edgeWithHypothesis(as1, as2, false));
  ExpectEq(ConcreteEdgeTo({id1, decltype(mg)::Cost(1)}),
           mg.edgeWithHypothesis(as2, as1, false));
  ExpectEq(ConcreteEdgeTo({id2, decltype(mg)::Cost(1)}),
           mg.edgeWithHypothesis(id1, id2, false));
  ExpectEq(ConcreteEdgeTo({id1, decltype(mg)::Cost(1)}),
           mg.edgeWithHypothesis(id2, id1, false));
  ExpectEq(ConcreteEdgeTo({id2, decltype(mg)::kInf}),
           mg.edgeWithHypothesis(as1, as2, true));
  ExpectEq(ConcreteEdgeTo({id1, decltype(mg)::kInf}),
           mg.edgeWithHypothesis(as2, as1, true));
  ExpectEq(ConcreteEdgeTo({id2, decltype(mg)::kInf}),
           mg.edgeWithHypothesis(id1, id2, true));
  ExpectEq(ConcreteEdgeTo({id1, decltype(mg)::kInf}),
           mg.edgeWithHypothesis(id2, id1, true));
  ExpectEq(ConcreteEdgeTo({decltype(mg)::kInvalidNode, decltype(mg)::kInf}),
           mg.edgeWithHypothesis(as1, kInvalidAgentState, false));
  ExpectEq(ConcreteEdgeTo({decltype(mg)::kInvalidNode, decltype(mg)::kInf}),
           mg.edgeWithHypothesis(kInvalidAgentState, as1, false));
  ExpectEq(ConcreteEdgeTo({decltype(mg)::kInvalidNode, decltype(mg)::kInf}),
           mg.edgeWithHypothesis(as1, kInvalidAgentState, true));
  ExpectEq(ConcreteEdgeTo({decltype(mg)::kInvalidNode, decltype(mg)::kInf}),
           mg.edgeWithHypothesis(kInvalidAgentState, as1, true));
}

TEST_F(FourWayGraphTest, GetsNeighborsAndCosts) {
  using ::testing::ElementsAre;
  using Cost = decltype(mg)::Cost;
  using NodeId = decltype(mg)::NodeId;

  using ConcreteEdgeTo = EdgeTo<NodeId, Cost>;

  std::istringstream iss(maze_str1);
  ASSERT_TRUE(loadMazeFromStream(maze, iss));

  EXPECT_EQ(1, mg.distance(0, 1));
  EXPECT_EQ(2, mg.distance(1, 16));
  EXPECT_EQ(mg.kInf, mg.distance(0, decltype(mg)::kInvalidNode));

  auto v1 = mg.neighbors(0);
  std::sort(v1.begin(), v1.end());
  EXPECT_THAT(v1, ElementsAre(1, 16));
  auto v1e = mg.neighborEdges(0);
  std::sort(v1e.begin(), v1e.end(), EdgeToSorter<NodeId, Cost>());
  ExpectEq(v1e[0], ConcreteEdgeTo({NodeId(1), mg.kInf}));
  ExpectEq(v1e[1], ConcreteEdgeTo({NodeId(16), Cost(1)}));

  EXPECT_EQ(mg.edgeCost(0, 1), mg.kInf);
  EXPECT_EQ(mg.edgeCost(0, 16), Cost(1));

  auto v2 = mg.neighbors(16);
  std::sort(v2.begin(), v2.end());
  EXPECT_THAT(v2, ElementsAre(0, 17, 32));
  auto v2e = mg.neighborEdges(16);
  std::sort(v2e.begin(), v2e.end(), EdgeToSorter<NodeId, Cost>());
  ExpectEq(v2e[0], ConcreteEdgeTo({NodeId(0), Cost(1)}));
  ExpectEq(v2e[1], ConcreteEdgeTo({NodeId(17), mg.kInf}));
  ExpectEq(v2e[2], ConcreteEdgeTo({NodeId(32), Cost(1)}));

  EXPECT_EQ(mg.edgeCost(16, 0), Cost(1));
  EXPECT_EQ(mg.edgeCost(16, 17), mg.kInf);
  EXPECT_EQ(mg.edgeCost(16, 32), Cost(1));

  auto v3 = mg.neighbors(17);
  std::sort(v3.begin(), v3.end());
  EXPECT_THAT(v3, ElementsAre(1, 16, 18, 33));
  auto v3e = mg.neighborEdges(17);
  std::sort(v3e.begin(), v3e.end(), EdgeToSorter<NodeId, Cost>());
  ExpectEq(v3e[0], ConcreteEdgeTo({NodeId(1), Cost(1)}));
  ExpectEq(v3e[1], ConcreteEdgeTo({NodeId(16), mg.kInf}));
  ExpectEq(v3e[2], ConcreteEdgeTo({NodeId(18), mg.kInf}));
  ExpectEq(v3e[3], ConcreteEdgeTo({NodeId(33), Cost(1)}));

  EXPECT_EQ(mg.edgeCost(17, 1), Cost(1));
  EXPECT_EQ(mg.edgeCost(17, 16), mg.kInf);
  EXPECT_EQ(mg.edgeCost(17, 18), mg.kInf);
  EXPECT_EQ(mg.edgeCost(17, 33), Cost(1));

  auto v4e = mg.neighborEdges(32768);
  EXPECT_EQ(0U, v4e.size());
}

TEST_F(FourWayGraphTest, TestPullBackSequence) {
  EXPECT_EQ(true, mg.isPullBackSequence({{0, 1, 0}}));
  EXPECT_EQ(false, mg.isPullBackSequence({{0, 1, 2}}));
  EXPECT_EQ(false, mg.isPullBackSequence({{0, 1, 16}}));
  EXPECT_EQ(false, mg.isPullBackSequence({{0, 18, 0}}));
}

TEST_F(FourWayGraphTest, GetsStartGoalNodeIds) {
  using ::testing::ElementsAre;
  using Cost = decltype(mg)::Cost;
  using NodeId = decltype(mg)::NodeId;

  std::istringstream iss(maze_str1);
  ASSERT_TRUE(loadMazeFromStream(maze, iss));
  EXPECT_EQ(mg.startNodeId(), 0);
  EXPECT_THAT(mg.goalNodeIds(), ElementsAre(119, 120, 135, 136));
}

TEST_P(FourWayGraphAffectedEdgesTest, CalculatesAffectedEdges) {
  auto edges = mg.affectedEdges(std::get<0>(GetParam()));
  EXPECT_EQ(std::get<1>(GetParam()), edges.size());
}
INSTANTIATE_TEST_SUITE_P(AffectedEdgesGeneral, FourWayGraphAffectedEdgesTest,
                         ::testing::Values(std::make_tuple(Position{0, 0}, 0U),
                                           std::make_tuple(Position{1, 1}, 0U),
                                           std::make_tuple(Position{0, 1}, 1U),
                                           std::make_tuple(Position{1, 0},
                                                           1U)));
INSTANTIATE_TEST_SUITE_P(
    AffectedEdgesInvalid, FourWayGraphAffectedEdgesTest,
    ::testing::Values(std::make_tuple(Position{100, 101}, 0U),
                      std::make_tuple(Position{200, 201}, 0U),
                      std::make_tuple(Position{201, 8}, 0U)));

TEST_P(FourWayGraphWallOnEdgeTest, CalculatesWallOnEdge) {
  Position p =
      mg.wallPositionOnEdge(std::get<0>(GetParam()), std::get<1>(GetParam()));
  EXPECT_EQ(std::get<2>(GetParam()), p);
}
INSTANTIATE_TEST_SUITE_P(
    WallOnEdgeGeneral, FourWayGraphWallOnEdgeTest,
    ::testing::Values(std::make_tuple(0, 16, Position{0, 1}),
                      std::make_tuple(0, 1, Position{1, 0}),
                      std::make_tuple(16, 0, Position{0, 1}),
                      std::make_tuple(1, 0, Position{1, 0})));
INSTANTIATE_TEST_SUITE_P(
    WallOnEdgeInvalid, FourWayGraphWallOnEdgeTest,
    ::testing::Values(std::make_tuple(0, 32, kInvalidAgentState.pos),
                      std::make_tuple(0, 2, kInvalidAgentState.pos),
                      std::make_tuple(255, 256, kInvalidAgentState.pos),
                      std::make_tuple(255, 257, kInvalidAgentState.pos),
                      std::make_tuple(20000, 20001, kInvalidAgentState.pos)));
