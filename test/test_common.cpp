#include "common.h"
#include "gtest/gtest.h"
#include <iostream>
#include <limits>

using namespace Amaze;

TEST(common_satsum, positive_case)
{
    EXPECT_EQ(2, satSum(1, 1));
    EXPECT_EQ(2.f, satSum(1.f, 1.f));
}

TEST(common_satsum, saturated_case)
{
    EXPECT_EQ(255, satSum(static_cast<uint8_t>(200), static_cast<uint8_t>(57)));

    float fmax = std::numeric_limits<float>::max();
    EXPECT_EQ(fmax, satSum(fmax / 3.f * 2.f, fmax / 3.f * 2.f));
}

TEST(common_position_difference, common_case)
{
    EXPECT_EQ(Position({ 1, 1 }), Position({ 2, 1 }) + Difference({ -1, 0 }));
    EXPECT_EQ(Difference({ 0, 1 }), Position({ 2, 1 }) - Position({ 2, 0 }));
}

GTEST_API_ int main(int argc, char** argv)
{
    printf("Running main() from %s\n", __FILE__);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
