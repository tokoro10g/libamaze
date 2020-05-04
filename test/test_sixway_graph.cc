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
#include "amaze/maze_graph/sixway_graph.h"
#include "gmock/gmock-matchers.h"
#include "gmock/gmock.h"
#include "gtest/gtest-typed-test.h"
#include "gtest/gtest.h"
#include "test_utils.h"

using namespace amaze;              // NOLINT(build/namespaces)
using namespace amaze::maze_graph;  // NOLINT(build/namespaces)
using namespace amaze::utils;       // NOLINT(build/namespaces)

template <typename ConcreteMaze>
using SixWayGraphFixture = AbstractMazeGraphFixture<
    SixWayGraph<true, uint16_t, uint16_t, ConcreteMaze::kMaxWidth>,
    ConcreteMaze>;

template <typename ConcreteMaze>
class SixWayGraphTypedTest : public SixWayGraphFixture<ConcreteMaze>,
                             public ::testing::Test {};
using SixWayGraphTest = SixWayGraphTypedTest<Maze<16>>;

class SixWayGraphAffectedEdgesTest
    : public SixWayGraphFixture<Maze<16>>,
      public ::testing::TestWithParam<std::tuple<Position, unsigned int>> {};
class SixWayGraphWallOnEdgeTest
    : public SixWayGraphFixture<Maze<16>>,
      public ::testing::TestWithParam<
          std::tuple<uint16_t, uint16_t, Position>> {};

TYPED_TEST_SUITE_P(SixWayGraphTypedTest);

TYPED_TEST_P(SixWayGraphTypedTest, ConvertsAgentStateAndNodeId) {
  using ::testing::ElementsAre;
  using TMazeGraph = typename decltype(this->mg)::Base;

  EXPECT_EQ(TMazeGraph::kInvalidNodeId,
            this->mg.nodeIdByAgentState({{0, 0}, kNoDirection, 0}));
  EXPECT_EQ(0, this->mg.nodeIdByAgentState({{1, 0}, kNoDirection, 0}));
  EXPECT_EQ(1, this->mg.nodeIdByAgentState({{3, 0}, kNoDirection, 0}));
  EXPECT_EQ(TMazeGraph::kMaxWidth - 1,
            this->mg.nodeIdByAgentState({{0, 1}, kNoDirection, 0}));

  EXPECT_EQ(AgentState({{1, 0}, kNoDirection, 0}),
            this->mg.agentStateByNodeId(0));
  EXPECT_EQ(AgentState({{3, 0}, kNoDirection, 0}),
            this->mg.agentStateByNodeId(1));
  EXPECT_EQ(AgentState({{0, 1}, kNoDirection, 0}),
            this->mg.agentStateByNodeId(TMazeGraph::kMaxWidth - 1));
  EXPECT_EQ(kInvalidAgentState,
            this->mg.agentStateByNodeId(2 * TMazeGraph::kMaxWidth *
                                        (TMazeGraph::kMaxWidth - 1)));

  EXPECT_THAT(this->mg.nodeIdsByPosition(Position{1, 0}), ElementsAre(0));
  EXPECT_THAT(this->mg.nodeIdsByPosition(Position{3, 0}), ElementsAre(1));
  EXPECT_THAT(this->mg.nodeIdsByPosition(Position{0, 1}),
              ElementsAre(TMazeGraph::kMaxWidth - 1));
  EXPECT_EQ(0U, this->mg.nodeIdsByPosition(Position{200, 200}).size());

  EXPECT_EQ(AgentState({{3, 0}, kEast, 0}), this->mg.agentStateByEdge(0, 1));
  EXPECT_EQ(AgentState({{1, 0}, kWest, 0}), this->mg.agentStateByEdge(1, 0));
  EXPECT_EQ(AgentState({{0, 1}, kNorth, 0}),
            this->mg.agentStateByEdge(0, TMazeGraph::kMaxWidth - 1));
  EXPECT_EQ(AgentState({{1, 0}, kEast, 0}),
            this->mg.agentStateByEdge(TMazeGraph::kMaxWidth - 1, 0));
  EXPECT_EQ(kInvalidAgentState, this->mg.agentStateByEdge(0, 2));
  EXPECT_EQ(kInvalidAgentState,
            this->mg.agentStateByEdge(0, TMazeGraph::kMaxWidth / 2));
}

REGISTER_TYPED_TEST_SUITE_P(SixWayGraphTypedTest, ConvertsAgentStateAndNodeId);
INSTANTIATE_TYPED_TEST_SUITE_P(VariousSize, SixWayGraphTypedTest, MazeTypes);

// TODO(tokoro10g): write remaining tests
