#include "gtest/gtest.h"
#include <iostream>
#include <limits>

#include "test_common.h"
#include "test_maze.h"
#include "test_mazegraph.h"

GTEST_API_ int main(int argc, char** argv)
{
    printf("Running main() from %s\n", __FILE__);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
