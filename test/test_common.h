#include "common.h"
#include "gtest/gtest.h"
#include <iostream>
#include <limits>

using namespace Amaze;

TEST(SatSumTest, SumNormal)
{
    EXPECT_EQ(2, satSum(uint8_t(1), uint8_t(1)));
    EXPECT_EQ(2, satSum(uint16_t(1), uint16_t(1)));
}

TEST(SatSumTest, SumSaturated)
{
    EXPECT_EQ(255, satSum(uint8_t(200), uint8_t(57)));
    EXPECT_EQ(65535, satSum(uint16_t(20000), uint16_t(55423)));
}

TEST(DifferenceTest, AddAndSubtract)
{
    EXPECT_EQ(Position({ 1, 1 }), Position({ 2, 1 }) + Difference({ -1, 0 }));
    EXPECT_EQ(Difference({ 0, 1 }), Position({ 2, 1 }) - Position({ 2, 0 }));
}

TEST(PositionTest, DeterminesType)
{
    EXPECT_EQ(Position({ 0, 0 }).type(), PositionTypes::kCell);
    EXPECT_EQ(Position({ 0, 1 }).type(), PositionTypes::kWall);
    EXPECT_EQ(Position({ 1, 0 }).type(), PositionTypes::kWall);
    EXPECT_EQ(Position({ 1, 1 }).type(), PositionTypes::kPillar);
}

TEST(ShiftOperatorsTest, PrintsDirection)
{
    std::stringstream buffer;
    buffer << kNorth << kSouth << kEast << kWest << kNoDirection;
    buffer << kFront << kBack << kRight << kLeft;
    std::string s = buffer.str();
    EXPECT_TRUE(s == "NSEW0NSEW");
}

TEST(ShiftOperatorsTest, PrintsPosition)
{
    std::stringstream buffer;
    buffer << Position({ 0, 1 }) << Position({ 2, 1 });
    std::string s = buffer.str();
    EXPECT_TRUE(s == "(0, 1)(2, 1)");
}

TEST(ShiftOperatorsTest, PrintsDifference)
{
    std::stringstream buffer;
    buffer << Difference({ 0, 1 }) << Difference({ 2, 1 });
    std::string s = buffer.str();
    EXPECT_TRUE(s == "(0, 1)(2, 1)");
}

TEST(ShiftOperatorsTest, PrintsAgentState)
{
    std::stringstream buffer;
    buffer << AgentState({ 0, 1, kNorth, 0 }) << AgentState({ 2, 1, kSouth, 1 });
    std::string s = buffer.str();
    EXPECT_TRUE(s == "((0, 1), N, 0)((2, 1), S, 1)");
}
