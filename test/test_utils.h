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

#ifndef TEST_TEST_UTILS_H_
#define TEST_TEST_UTILS_H_

#include <string>

#include "amaze/common/common_types.h"
#include "gmock/gmock-matchers.h"
#include "gmock/gmock.h"
#include "gtest/gtest-typed-test.h"
#include "gtest/gtest.h"

typedef ::testing::Types<amaze::Maze<16>, amaze::Maze<17>, amaze::Maze<24>,
                         amaze::Maze<32>, amaze::Maze<64>>
    MazeTypes;

template <typename ConcreteMazeGraph, typename ConcreteMaze>
class AbstractMazeGraphFixture {
 protected:
  AbstractMazeGraphFixture() : maze(), mg(maze) {}
  ConcreteMaze maze;
  ConcreteMazeGraph mg;
};

/* clang-format off */

const std::string maze_str1 = "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+\n"  // NOLINT
                              "|                           |   |   |                           |\n"  // NOLINT
                              "+   +---+---+   +---+---+   +---+---+   +---+---+   +---+---+   +\n"  // NOLINT
                              "|   |   |       |   |   |               |   |   |   |   |   |   |\n"  // NOLINT
                              "+   +---+   +   +---+---+---+---+---+---+---+---+   +---+---+   +\n"  // NOLINT
                              "|   |       |   |   |   |       |               |   |   |   |   |\n"  // NOLINT
                              "+   +   +---+   +---+---+   +   +   +---+---+   +   +---+---+   +\n"  // NOLINT
                              "|               |   |   |   |   |   |           |               |\n"  // NOLINT
                              "+   +---+---+   +---+---+   +   +   +   +---+---+   +---+---+   +\n"  // NOLINT
                              "|   |   |   |           |   |   |   |   |           |       |   |\n"  // NOLINT
                              "+   +---+---+---+---+   +   +   +   +   +   +---+---+   +   +   +\n"  // NOLINT
                              "|   |   |   |   |   |       |       |       |   |   |   |   |   |\n"  // NOLINT
                              "+---+---+---+---+---+   +---+---+---+---+   +---+---+   +   +   +\n"  // NOLINT
                              "|       |               |               |               |       |\n"  // NOLINT
                              "+   +   +   +---+---+---+   +---+---+   +---+   +---+   +   +---+\n"  // NOLINT
                              "|   |   |   |   |   |   |     G   G |   |           |   |   |   |\n"  // NOLINT
                              "+   +   +   +---+---+---+   +   +   +   +---+---+   +   +   +---+\n"  // NOLINT
                              "|   |   |   |   |   |   |   | G   G |               |   |   |   |\n"  // NOLINT
                              "+   +   +---+---+---+---+   +---+---+   +---+---+---+   +   +---+\n"  // NOLINT
                              "|   |                   |               |               |       |\n"  // NOLINT
                              "+   +   +---+   +---+   +---+---+---+---+---+---+---+---+---+   +\n"  // NOLINT
                              "|   |   |       |   |   |   |   |   |       |   |   |   |   |   |\n"  // NOLINT
                              "+   +---+   +---+---+   +   +---+---+   +   +---+---+---+---+   +\n"  // NOLINT
                              "|   |       |       |   |   |   |   |   |           |   |   |   |\n"  // NOLINT
                              "+   +   +---+---+---+   +   +---+---+   +---+---+   +---+---+   +\n"  // NOLINT
                              "|       |               |   |   |   |   |   |   |               |\n"  // NOLINT
                              "+   +---+   +---+---+---+   +---+---+   +---+---+   +---+---+   +\n"  // NOLINT
                              "|   |   |   |   |   |   |               |   |   |   |   |   |   |\n"  // NOLINT
                              "+   +   +   +---+---+---+---+---+---+---+---+---+   +---+---+   +\n"  // NOLINT
                              "|   |   |   |                           |   |   |   |   |   |   |\n"  // NOLINT
                              "+   +   +   +---+---+   +---+   +---+   +---+---+   +---+---+   +\n"  // NOLINT
                              "| S |   |                       |   |                           |\n"  // NOLINT
                              "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+\n";  // NOLINT

const std::string maze_str2 = "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+\n"  // NOLINT
                              "|       |       |                   |           |                           |           |                           |           |\n"  // NOLINT
                              "+   +   +   +   +   +---+---+---+   +   +   +   +   +---+---+---+---+---+   +   +   +   +   +---+---+---+---+---+   +   +   +   +\n"  // NOLINT
                              "|       |   |   |               |   |           |                       |   |           |                       |   |           |\n"  // NOLINT
                              "+---+---+   +   +---+---+---+   +   +---+---+---+---+---+---+---+---+   +   +---+---+---+---+---+---+---+---+   +   +---+---+---+\n"  // NOLINT
                              "|           |   |               |   |                               |   |                                       |               |\n"  // NOLINT
                              "+   +---+---+   +   +---+---+---+   +---+   +---+   +---+---+---+   +   +---+---+---+---+---+---+---+---+---+---+---+---+---+   +\n"  // NOLINT
                              "|           |   |               |       |       |   |               |                                           |       |       |\n"  // NOLINT
                              "+---+   +---+   +---+---+---+   +   +   +---+   +---+   +---+---+---+---+---+---+---+---+---+   +---+---+---+   +   +   +   +   +\n"  // NOLINT
                              "|           |   |           |   |   |       |       |   |           |                       |   |           |   |   |       |   |\n"  // NOLINT
                              "+   +---+   +   +   +   +   +   +   +---+   +---+   +   +   +   +   +   +---+---+---+---+   +   +   +   +   +   +   +---+---+---+\n"  // NOLINT
                              "|           |   |           |   |       |       |   |   |           |   |                   |   |           |   |               |\n"  // NOLINT
                              "+---+   +---+   +   +   +   +   +   +   +---+   +   +   +   +   +   +   +   +---+---+---+---+   +   +   +   +   +---+---+---+   +\n"  // NOLINT
                              "|           |   |           |   |   |       |       |   |           |   |                   |   |           |   |   |       |   |\n"  // NOLINT
                              "+   +---+   +   +---+---+---+   +   +---+   +---+   +   +---+---+---+   +---+---+---+---+   +   +---+---+---+   +   +   +   +   +\n"  // NOLINT
                              "|           |               |   |                   |   |               |                   |                   |       |       |\n"  // NOLINT
                              "+---+   +---+---+   +---+---+   +---+---+---+---+   +   +---+---+---+   +   +---+---+---+---+---+---+---+   +---+   +---+---+---+\n"  // NOLINT
                              "|           |                   |                   |   |               |                   |                   |               |\n"  // NOLINT
                              "+---+---+   +   +---+---+---+   +   +---+---+---+   +   +---+---+---+   +   +---+---+---+   +   +---+   +---+   +   +---+---+---+\n"  // NOLINT
                              "|       |   |   |       |       |   |           |   |   |       |   |   |   | G   G   G |   |                   |   |           |\n"  // NOLINT
                              "+   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +---+   +---+   +---+   +   +   +   +\n"  // NOLINT
                              "|       |   |       |       |   |   |           |   |   |   |   |   |   |   | G   G   G |   |                   |   |           |\n"  // NOLINT
                              "+   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +---+   +---+   +   +   +   +   +\n"  // NOLINT
                              "|       |   |   |       |       |   |           |   |   |   |   |   |   |   | G   G   G |   |                   |   |           |\n"  // NOLINT
                              "+---+---+   +   +   +   +   +   +   +---+---+---+   +   +   +   +   +   +   +   +---+---+   +---+   +---+   +---+   +---+---+---+\n"  // NOLINT
                              "|           |       |       |   |                   |       |       |   |                                       |               |\n"  // NOLINT
                              "+   +---+---+   +---+---+---+---+   +---+---+---+---+   +---+---+---+   +---+---+---+---+---+---+---+---+---+---+---+---+---+   +\n"  // NOLINT
                              "|           |                   |                   |                   |                                       |               |\n"  // NOLINT
                              "+---+   +---+   +---+---+---+   +   +---+---+---+---+---+---+---+---+---+---+   +---+   +---+   +---+---+---+   +   +---+   +---+\n"  // NOLINT
                              "|           |   |           |   |                   |   |           |   |                   |   |           |   |               |\n"  // NOLINT
                              "+---+   +---+   +   +   +   +   +---+---+---+---+   +   +   +   +   +   +---+   +---+   +---+   +   +   +   +   +---+   +---+   +\n"  // NOLINT
                              "|           |   |           |   |                   |   |           |   |                   |   |           |   |               |\n"  // NOLINT
                              "+---+   +---+   +   +   +   +   +   +---+---+---+---+   +   +   +   +   +   +---+   +---+   +   +   +   +   +   +   +---+   +---+\n"  // NOLINT
                              "|           |   |           |   |                   |   |           |   |                   |   |           |                   |\n"  // NOLINT
                              "+---+   +---+   +---+---+---+   +---+---+---+---+   +   +---+---+---+   +   +---+   +---+   +   +---+---+---+   +---+   +---+   +\n"  // NOLINT
                              "|           |                   |                                                           |                   |               |\n"  // NOLINT
                              "+   +---+---+   +---+---+---+---+   +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+   +---+   +---+   +---+\n"  // NOLINT
                              "|           |                   |                   |                                       |       |       |   |               |\n"  // NOLINT
                              "+---+---+   +   +---+---+---+---+---+---+---+---+   +   +   +---+   +   +   +---+---+---+   +   +   +   +   +   +   +---+---+---+\n"  // NOLINT
                              "|       |   |                   |   |           |   |   |   |   |   |   |   |           |       |       |       |   |           |\n"  // NOLINT
                              "+   +   +   +---+---+---+---+   +   +   +   +   +   +   +---+   +---+   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +\n"  // NOLINT
                              "|       |   |                   |   |           |   |                   |   |           |   |       |       |   |   |           |\n"  // NOLINT
                              "+   +   +   +   +---+---+---+---+   +   +   +   +   +   +   +---+   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +\n"  // NOLINT
                              "|       |   |                   |   |           |   |   |   |   |   |   |   |           |       |       |       |   |           |\n"  // NOLINT
                              "+---+---+   +---+---+---+---+   +   +---+---+---+   +   +---+   +---+   +   +---+---+---+   +   +   +   +   +   +   +---+---+---+\n"  // NOLINT
                              "|           |                   |                   |                   |                   |       |       |                   |\n"  // NOLINT
                              "+   +---+---+   +---+---+   +---+   +---+---+---+---+---+   +---+---+---+---+---+---+   +---+   +---+---+---+---+---+---+---+   +\n"  // NOLINT
                              "|           |                   |   |       |       |                   |                   |                                   |\n"  // NOLINT
                              "+---+---+   +---+---+---+---+   +   +   +   +   +   +   +---+---+---+   +   +---+   +---+   +   +---+---+---+   +   +   +   +   +\n"  // NOLINT
                              "|       |   |   |           |   |       |       |       |           |   |                   |   |           |   |   |   |   |   |\n"  // NOLINT
                              "+   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +---+   +---+   +---+   +   +   +   +   +   +   +   +   +\n"  // NOLINT
                              "|   |   |   |   |           |   |   |       |       |   |           |   |                   |   |           |   |   |   |   |   |\n"  // NOLINT
                              "+   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +---+   +---+   +   +   +   +   +   +   +   +   +   +\n"  // NOLINT
                              "|   |       |   |           |   |       |       |   |   |           |                       |   |           |   |   |   |   |   |\n"  // NOLINT
                              "+   +---+---+   +---+---+---+   +   +   +   +   +   +   +---+---+---+   +---+   +---+   +---+   +---+---+---+   +   +   +   +   +\n"  // NOLINT
                              "|           |                   |   |       |       |                   |                                       |               |\n"  // NOLINT
                              "+   +   +---+   +---+---+---+   +---+---+   +---+---+---+---+---+   +---+---+---+---+---+---+---+---+---+---+---+---+---+---+   +\n"  // NOLINT
                              "|   |       |               |   |                   |       |       |   |                                       |   |   |   |   |\n"  // NOLINT
                              "+   +---+   +---+---+---+   +   +   +---+---+---+   +   +   +   +   +   +   +---+---+---+   +   +---+   +---+   +   +   +   +   +\n"  // NOLINT
                              "|   |   |   |               |   |   |           |   |   |       |       |   |           |   |   |       |                       |\n"  // NOLINT
                              "+   +   +   +   +---+---+---+   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +\n"  // NOLINT
                              "|   |   |   |               |   |   |           |   |       |       |   |   |           |   |       |       |   |   |   |   |   |\n"  // NOLINT
                              "+   +   +   +---+---+---+   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +   +---+   +---+   +   +---+---+   +\n"  // NOLINT
                              "| S |   |                   |       |           |       |       |           |           |   |                                   |\n"  // NOLINT
                              "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+\n";  // NOLINT

/* clang-format on */

#endif  // TEST_TEST_UTILS_H_
