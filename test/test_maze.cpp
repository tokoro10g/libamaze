#include "gtest/gtest.h"
#include <iostream>
#include <sstream>
#include <string>

#include "maze.h"
#include "mazeutility.h"

#include "cases_maze.h"

using namespace Amaze;
using namespace Amaze::Utility;

TEST(maze_load, load_from_string_stream1)
{
    std::istringstream iss(maze_str1);
    Maze<16> maze;
    ASSERT_TRUE(loadMazeFromStream(maze, iss));
    printMaze(maze);
    EXPECT_EQ(16, maze.getWidth());
    EXPECT_EQ(Position({ 0, 0 }), maze.start);
    EXPECT_EQ(Position({ 13, 13 }), maze.goals[0]);
    EXPECT_EQ(true, maze.isSetWall({ 1, 0 }));
}

TEST(maze_load, load_from_string_stream2)
{
    std::istringstream iss(maze_str2);
    Maze<32> maze;
    ASSERT_TRUE(loadMazeFromStream(maze, iss));
    printMaze(maze);
    EXPECT_EQ(32, maze.getWidth());
    EXPECT_EQ(Position({ 0, 0 }), maze.start);
    EXPECT_EQ(Position({ 37, 39 }), maze.goals[0]);
    EXPECT_EQ(true, maze.isSetWall({ 1, 0 }));
}

TEST(maze_load, load_too_large_data)
{
    std::istringstream iss(maze_str1);
    Maze<15> maze;
    ASSERT_FALSE(loadMazeFromStream(maze, iss));
}

TEST(maze_load, load_empty)
{
    Maze<16> maze;
    loadEmptyMaze(14, 14, maze);
    EXPECT_EQ(Position({ 0, 0 }), maze.start);
    EXPECT_EQ(Position({ 14, 14 }), maze.goals[0]);
    for (int x = 0; x < 16 * 2; x += 2) {
        for (int y = 0; y < 16 * 2; y += 2) {
            EXPECT_EQ(false, maze.isSetWall({ uint8_t(x), uint8_t(y) }));
            EXPECT_EQ(false, maze.isCheckedWall({ uint8_t(x), uint8_t(y) }));
        }
    }
}

TEST(maze_ops, set_toggle)
{
    std::istringstream iss(maze_str1);
    Maze<16> maze;
    ASSERT_TRUE(loadMazeFromStream(maze, iss));

    EXPECT_EQ(true, maze.isSetWall({ 1, 0 }));
    maze.toggleWall({ 1, 0 });
    EXPECT_EQ(false, maze.isSetWall({ 1, 0 }));
    maze.setWall({ 1, 0 }, true);
    EXPECT_EQ(true, maze.isSetWall({ 1, 0 }));
    maze.setWall({ 1, 0 }, false);
    EXPECT_EQ(false, maze.isSetWall({ 1, 0 }));

    EXPECT_EQ(false, maze.isCheckedWall({ 1, 0 }));
    maze.toggleCheckedWall({ 1, 0 });
    EXPECT_EQ(true, maze.isCheckedWall({ 1, 0 }));
    maze.setCheckedWall({ 1, 0 }, false);
    EXPECT_EQ(false, maze.isCheckedWall({ 1, 0 }));
    maze.setCheckedWall({ 1, 0 }, true);
    EXPECT_EQ(true, maze.isCheckedWall({ 1, 0 }));
}

GTEST_API_ int main(int argc, char** argv)
{
    printf("Running main() from %s\n", __FILE__);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
