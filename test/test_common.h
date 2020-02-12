#include "common.h"
#include "gtest/gtest.h"
#include <iostream>
#include <limits>

using namespace Amaze;

TEST(common_satsum, positive_case)
{
    EXPECT_EQ(2, satSum(uint8_t(1), uint8_t(1)));
    EXPECT_EQ(2, satSum(uint16_t(1), uint16_t(1)));
}

TEST(common_satsum, saturated_case)
{
    EXPECT_EQ(255, satSum(uint8_t(200), uint8_t(57)));
    EXPECT_EQ(65535, satSum(uint16_t(20000), uint16_t(55423)));
}

TEST(common_position_difference, common_case)
{
    EXPECT_EQ(Position({ 1, 1 }), Position({ 2, 1 }) + Difference({ -1, 0 }));
    EXPECT_EQ(Difference({ 0, 1 }), Position({ 2, 1 }) - Position({ 2, 0 }));
}
