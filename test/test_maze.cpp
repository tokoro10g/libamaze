#include "gtest/gtest.h"
#include <iostream>
#include <sstream>
#include <string>

#include "maze.h"
#include "mazeutility.h"

using namespace Amaze;
using namespace Amaze::Utility;

const std::string maze_str1 = "0\n"
                              "16\n"
                              "16\n"
                              "9 5 5 1 5 5 3 f f 9 5 5 1 5 5 3\n"
                              "a f 9 2 f f c 5 5 6 f f a f f a\n"
                              "a 9 6 a f f 9 3 9 5 5 3 a f f a\n"
                              "8 4 5 2 f f a a a 9 5 6 8 5 5 2\n"
                              "a f f c 5 3 a a a a 9 5 6 9 3 a\n"
                              "e f f f f 8 6 c 6 c 2 f f a a a\n"
                              "9 3 9 5 5 6 9 5 5 3 c 1 5 2 8 6\n"
                              "a a a f f f 8 1 3 a d 4 3 a a f\n"
                              "a a e f f f a c 6 8 5 5 6 a a f\n"
                              "a 8 5 1 5 3 c 5 5 6 d 5 5 6 c 3\n"
                              "a e 9 6 f a b f f 9 3 f f f f a\n"
                              "a 9 6 d 7 a a f f a c 5 3 f f a\n"
                              "8 6 9 5 5 6 a f f a f f 8 5 5 2\n"
                              "a b a f f f c 5 5 6 f f a f f a\n"
                              "a a a d 5 1 5 1 5 3 f f a f f a\n"
                              "e e c 5 5 4 5 6 f c 5 5 4 5 5 6\n";

TEST(maze_load, load_from_string_stream)
{
    std::istringstream iss(maze_str1);
    Maze<16> maze;
    EXPECT_EQ(true, loadMazeFromStream(maze, iss));
    EXPECT_EQ(Position({ 14, 14 }), maze.getGoal());
    EXPECT_EQ(true, maze.isSetWall({ 1, 0 }));
    EXPECT_EQ(Position({ 0, 0 }), maze.getStart());
}

TEST(maze_load, load_too_large_data)
{
    std::istringstream iss(maze_str1);
    Maze<15> maze;
    EXPECT_EQ(false, loadMazeFromStream(maze, iss));
}

GTEST_API_ int main(int argc, char** argv)
{
    printf("Running main() from %s\n", __FILE__);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
