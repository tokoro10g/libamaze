#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <iostream>
#include <limits>

#include "cases_maze.h"
#include "fourwaystepmapgraph.h"
#include "mazeutility.h"

using namespace Amaze;
using namespace Amaze::Utility;
using namespace testing;

TEST(fourwaygraph, unit)
{
    std::istringstream iss(maze_str1);
    Maze<16> maze;
    ASSERT_TRUE(loadMazeFromStream(maze, iss));
    FourWayStepMapGraph mg(maze);
    uint16_t id_from, id_to;
    EXPECT_EQ(0, id_from = mg.nodeIdByAgentState({ { 0, 0 }, kNoDirection, 0 }));
    EXPECT_EQ(16, id_to = mg.nodeIdByAgentState({ { 0, 2 }, kNoDirection, 0 }));
    EXPECT_EQ(1, mg.distance(id_from, id_to));
    EXPECT_EQ(1, mg.distance(uint16_t(id_from + 1), id_to));

    std::vector<uint16_t> v1 = mg.neighbors(0);
    sort(v1.begin(), v1.end());
    ASSERT_THAT(v1, ElementsAre(1, 16));
    std::vector<uint16_t> v2 = mg.neighbors(16);
    sort(v2.begin(), v2.end());
    ASSERT_THAT(v2, ElementsAre(0, 17, 32));
    std::vector<uint16_t> v3 = mg.neighbors(17);
    sort(v3.begin(), v3.end());
    ASSERT_THAT(v3, ElementsAre(1, 16, 18, 33));
}

GTEST_API_ int main(int argc, char** argv)
{
    printf("Running main() from %s\n", __FILE__);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
