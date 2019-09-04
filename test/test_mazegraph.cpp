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
    EXPECT_EQ(1, mg.distance(id_from + 1, id_to));

    std::vector<uint16_t> v;
    mg.neighbors(0, v);
    sort(v.begin(), v.end());
    ASSERT_THAT(v, ElementsAre(1, 16));
    v.clear();
    mg.neighbors(16, v);
    sort(v.begin(), v.end());
    ASSERT_THAT(v, ElementsAre(0, 17, 32));
    v.clear();
    mg.neighbors(17, v);
    sort(v.begin(), v.end());
    ASSERT_THAT(v, ElementsAre(1, 16, 18, 33));
}

GTEST_API_ int main(int argc, char** argv)
{
    printf("Running main() from %s\n", __FILE__);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
