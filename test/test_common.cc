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

#include "amaze/common/common_types.h"
#include "amaze/common/common_utils.h"
#include "gtest/gtest.h"

using namespace amaze;  // NOLINT(build/namespaces)

TEST(SatSumTest, SumNormal) {
  EXPECT_EQ(2, satSum(uint8_t(1), uint8_t(1)));
  EXPECT_EQ(2, satSum(uint16_t(1), uint16_t(1)));
}

TEST(SatSumTest, SumSaturated) {
  EXPECT_EQ(255, satSum(uint8_t(200), uint8_t(57)));
  EXPECT_EQ(65535, satSum(uint16_t(20000), uint16_t(55423)));
}

TEST(DifferenceTest, AddAndSubtract) {
  EXPECT_EQ(Position({1, 1}), Position({2, 1}) + Difference({-1, 0}));
  EXPECT_EQ(Difference({0, 1}), Position({2, 1}) - Position({2, 0}));
}

TEST(PositionTest, DeterminesType) {
  EXPECT_EQ(Position({0, 0}).type(), PositionType::kCell);
  EXPECT_EQ(Position({0, 1}).type(), PositionType::kWall);
  EXPECT_EQ(Position({1, 0}).type(), PositionType::kWall);
  EXPECT_EQ(Position({1, 1}).type(), PositionType::kPillar);
}

TEST(ShiftOperatorsTest, PrintsDirection) {
  std::stringstream buffer;
  buffer << kNorth << kSouth << kEast << kWest << kNoDirection;
  buffer << kFront << kBack << kRight << kLeft;
  std::string s = buffer.str();
  EXPECT_EQ(s, "NSEW0NSEW");
}

TEST(ShiftOperatorsTest, PrintsPosition) {
  std::stringstream buffer;
  buffer << Position({0, 1}) << Position({2, 1});
  std::string s = buffer.str();
  EXPECT_EQ(s, "(0, 1)(2, 1)");
}

TEST(ShiftOperatorsTest, PrintsDifference) {
  std::stringstream buffer;
  buffer << Difference({0, 1}) << Difference({2, 1});
  std::string s = buffer.str();
  EXPECT_EQ(s, "(0, 1)(2, 1)");
}

TEST(ShiftOperatorsTest, PrintsAgentState) {
  std::stringstream buffer;
  buffer << AgentState({0, 1, kNorth, 0}) << AgentState({2, 1, kSouth, 1});
  std::string s = buffer.str();
  EXPECT_EQ(s, "((0, 1), N, 0)((2, 1), S, 1)");
}
