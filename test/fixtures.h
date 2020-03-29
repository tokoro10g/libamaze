#pragma once

#include "gmock/gmock-matchers.h"
#include "gmock/gmock.h"
#include "gtest/gtest-typed-test.h"
#include "gtest/gtest.h"

#include "mazegraph.h"
#include "mazeutility.h"

typedef ::testing::Types<Maze<16>, Maze<17>, Maze<24>, Maze<32>, Maze<64>> MazeTypes;

template <typename ConcreteMazeGraph, typename ConcreteMaze>
class AbstractMazeGraphFixture {
protected:
    AbstractMazeGraphFixture()
        : maze()
        , mg(maze)
    {
    }
    ConcreteMaze maze;
    ConcreteMazeGraph mg;
};
