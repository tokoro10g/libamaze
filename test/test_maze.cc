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
#include <sstream>
#include <string>

#include "amaze/common/common_types.h"
#include "amaze/common/maze_utils.h"
#include "gtest/gtest.h"
#include "test_utils.h"

using namespace amaze;         // NOLINT(build/namespaces)
using namespace amaze::utils;  // NOLINT(build/namespaces)

TEST(MazeLoadTest, ReturnsFalseIfDataIsTooLarge) {
  std::istringstream iss(maze_str1);
  Maze<15> maze;
  ASSERT_FALSE(loadMazeFromStream(maze, iss));
}

TEST(MazeLoadTest, ReturnsFalseIfDataIsFaulty) {
  std::istringstream iss("+--\n+--\n+--\n");
  Maze<15> maze;
  ASSERT_FALSE(loadMazeFromStream(maze, iss));
}

TEST(MazeLoadTest, LoadsFromStringStream1) {
  std::istringstream iss(maze_str1);
  Maze<16> maze;
  ASSERT_TRUE(loadMazeFromStream(maze, iss));
  printMaze(maze);
  EXPECT_EQ(16, maze.getWidth());
  EXPECT_EQ(Position({0, 0}), maze.start);
  EXPECT_EQ(maze.goals.count(Position({14, 14})), 1);
  EXPECT_EQ(true, maze.isSetWall({1, 0}));
}

TEST(MazeLoadTest, LoadsFromStringStream2) {
  std::istringstream iss(maze_str2);
  Maze<32> maze;
  ASSERT_TRUE(loadMazeFromStream(maze, iss));
  printMaze(maze);
  EXPECT_EQ(32, maze.getWidth());
  EXPECT_EQ(Position({0, 0}), maze.start);
  EXPECT_EQ(maze.goals.count(Position({38, 40})), 1);
  EXPECT_EQ(true, maze.isSetWall({1, 0}));
}

TEST(MazeLoadTest, LoadsEmptyMaze) {
  Maze<16> maze;
  loadEmptyMaze(14, 14, maze);
  EXPECT_EQ(Position({0, 0}), maze.start);
  EXPECT_EQ(maze.goals.count(Position({14, 14})), 1);
  for (int y = 0; y < 16 * 2 - 1; y++) {
    for (int x = !(y % 2); x < 16 * 2 - 1; x += 2) {
      EXPECT_EQ(false, maze.isSetWall({uint8_t(x), uint8_t(y)}));
      EXPECT_EQ(false, maze.isCheckedWall({uint8_t(x), uint8_t(y)}));
    }
  }
}

TEST(MazeOperationTest, CanSetAndToggle) {
  std::istringstream iss(maze_str1);
  Maze<16> maze;
  ASSERT_TRUE(loadMazeFromStream(maze, iss));

  EXPECT_EQ(true, maze.isSetWall({1, 0}));
  maze.toggleWall({1, 0});
  EXPECT_EQ(false, maze.isSetWall({1, 0}));
  maze.setWall({1, 0}, true);
  EXPECT_EQ(true, maze.isSetWall({1, 0}));
  maze.setWall({1, 0}, false);
  EXPECT_EQ(false, maze.isSetWall({1, 0}));

  EXPECT_EQ(false, maze.isCheckedWall({1, 0}));
  maze.toggleCheckedWall({1, 0});
  EXPECT_EQ(true, maze.isCheckedWall({1, 0}));
  maze.setCheckedWall({1, 0}, false);
  EXPECT_EQ(false, maze.isCheckedWall({1, 0}));
  maze.setCheckedWall({1, 0}, true);
  EXPECT_EQ(true, maze.isCheckedWall({1, 0}));
}

TEST(MazeOperationTest, ReturnsFalseIfNotWall) {
  Maze<16> maze;
  loadEmptyMaze(14, 14, maze);

  EXPECT_EQ(false, maze.isSetWall({0, 0}));
  EXPECT_EQ(false, maze.isSetWall({1, 1}));

  EXPECT_EQ(false, maze.isCheckedWall({0, 0}));
  EXPECT_EQ(false, maze.isCheckedWall({1, 1}));
}

TEST(MazeOperationTest, NoOpIfNotWall) {
  Maze<16> maze;
  loadEmptyMaze(14, 14, maze);

  maze.setWall({0, 0}, true);
  EXPECT_EQ(false, maze.isSetWall({0, 0}));

  maze.setCheckedWall({0, 0}, true);
  EXPECT_EQ(false, maze.isCheckedWall({0, 0}));
}

TEST(MazeOperationTest, NoOpIfOutOfRange) {
  Maze<16> maze;
  loadEmptyMaze(14, 14, maze);

  maze.setWall({200, 201}, true);
  EXPECT_EQ(false, maze.isSetWall({200, 201}));

  maze.setCheckedWall({200, 201}, true);
  EXPECT_EQ(false, maze.isCheckedWall({200, 201}));
}
